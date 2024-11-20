from ble_handler import BLEHandler
from Modulo_PID import PID  
import time
import struct

# Inicializar BLE
ble = BLEHandler(name="ESP32_PID",
                 service_uuid="12345678-1234-5678-1234-56789abcdef0",
                 characteristic_uuid="12345678-1234-5678-1234-56789abcdef1")

# Inicializar PID
pid = PID(Kp=1.0, Ki=0.1, Kd=0.05, Ts=0.5, setpoint=50.0)

# Variables iniciales
setpoint = 50.0
measurement = 0.0  # Inicialización del sensor simulado

try:
    while True:
        # Simular lectura de un sensor
        measurement += (setpoint - measurement) * 0.1  # Dinámica simulada
        print(f"Measured value: {measurement}")

        # Calcular salida PID
        control_signal = pid.compute(measurement)
        print(f"Control signal: {control_signal}")

        # Enviar datos al smartphone (valor medido)
        ble.send_data(measurement)

        # Leer nuevas ganancias PID desde el smartphone
        new_gains = ble.read_received_data()
        if new_gains:
            pid.Kp, pid.Ki, pid.Kd = new_gains
            print(f"Updated gains: Kp={pid.Kp}, Ki={pid.Ki}, Kd={pid.Kd}")

        # Pausar según el tiempo de muestreo
        time.sleep(pid.Ts)

except KeyboardInterrupt:
    print("Stopping program...")

finally:
    # Desactivar BLE
    ble.deinit()
