from bluetooth import BLE, UUID
import struct

class BLEHandler:
    def __init__(self, name="ESP32_Device", service_uuid=None, characteristic_uuid=None):
        self.ble = BLE()
        self.ble.active(True)

        self.name = name
        self.service_uuid = UUID(service_uuid or "12345678-1234-5678-1234-56789abcdef0")
        self.characteristic_uuid = UUID(characteristic_uuid or "12345678-1234-5678-1234-56789abcdef1")

        self._connected = False
        self._received_data = None
        self._register()
        self.ble.config(gap_name=self.name)
        self.ble.irq(self._ble_irq)
        self.start_advertising()

    def _register(self):
        self.services = (
            (self.service_uuid, (
                (self.characteristic_uuid, 0x0002 | 0x0008 | 0x0010),  # READ | WRITE | NOTIFY
            )),
        )
        ((self.handle,),) = self.ble.gatts_register_services(self.services)
        self.ble.gatts_write(self.handle, struct.pack("f", 0.0))  # Inicializar valor

    def _ble_irq(self, event, data):
        if event == 1:  # Conexión
            self._connected = True
            print("Device connected")
        elif event == 2:  # Desconexión
            self._connected = False
            print("Device disconnected")
            self.start_advertising()
        elif event == 3:  # Escritura
            self._received_data = self.ble.gatts_read(self.handle)
            print(f"Received data: {self._received_data}")

    def read_received_data(self):
        if self._received_data:
            data = struct.unpack("fff", self._received_data)  # Desempaqueta \( K_p \), \( K_i \), \( K_d \)
            self._received_data = None
            return data
        return None

    def send_data(self, data):
        if self._connected:
            payload = struct.pack("f", data)
            self.ble.gatts_notify(0, self.handle, payload)
            print(f"Sent data: {data}")
        else:
            print("No device connected.")

    def start_advertising(self):
        adv_data = bytes([0x02, 0x01, 0x06]) + \
                   bytes([len(self.name) + 1, 0x09]) + self.name.encode('utf-8')
        self.ble.gap_advertise(100, adv_data)
        print("Advertising started.")

    def stop_advertising(self):
        self.ble.gap_advertise(None)
        print("BLE advertising stopped.")

    def deinit(self):
        self.stop_advertising()
        self.ble.active(False)
        print("BLE has been deinitialized.")
