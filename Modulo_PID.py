class PID:
    def __init__(self, Kp, Ki, Kd, Ts, setpoint=0, filter_constant=0.1, output_limits=(None, None)):
        """
        Inicializa un controlador PID discreto.
        
        :param Kp: Ganancia proporcional.
        :param Ki: Ganancia integral.
        :param Kd: Ganancia derivativa.
        :param Ts: Tiempo de muestreo en segundos.
        :param setpoint: Punto de referencia o valor objetivo.
        :param filter_constant: Constante de filtrado para el término derivativo (entre 0 y 1).
        :param output_limits: Límites para la salida del PID (tupla de la forma (min, max)).
        """
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.Ts = Ts
        self.setpoint = setpoint
        self.filter_constant = filter_constant
        self.output_limits = output_limits

        self._integral = 0
        self._prev_error = 0
        self._prev_derivative = 0
        self.output = 0

    def compute(self, measurement):
        """
        Calcula la salida del controlador PID basado en la medición actual.
        
        :param measurement: Valor actual medido del sistema.
        :return: Salida del controlador PID.
        """
        # Calcular el error actual
        error = self.setpoint - measurement

        # Calcular la parte proporcional
        P = self.Kp * error

        # Calcular la parte integral con anti-windup
        self._integral += error * self.Ts
        if self.output_limits[0] is not None and self.output_limits[1] is not None:
            self._integral = max(min(self._integral, self.output_limits[1]), self.output_limits[0])
        I = self.Ki * self._integral

        # Calcular la parte derivativa con filtro
        raw_derivative = (error - self._prev_error) / self.Ts
        D = self.Kd * ((1 - self.filter_constant) * self._prev_derivative + self.filter_constant * raw_derivative)

        # Calcular la salida total
        self.output = P + I + D

        # Limitar la salida
        if self.output_limits[0] is not None and self.output_limits[1] is not None:
            self.output = max(min(self.output, self.output_limits[1]), self.output_limits[0])

        # Actualizar valores previos
        self._prev_error = error
        self._prev_derivative = raw_derivative

        return self.output

    def update_parameters(self, Kp=None, Ki=None, Kd=None, setpoint=None, output_limits=None):
        """
        Actualiza dinámicamente los parámetros del PID.
        
        :param Kp: Nueva ganancia proporcional.
        :param Ki: Nueva ganancia integral.
        :param Kd: Nueva ganancia derivativa.
        :param setpoint: Nuevo punto de referencia.
        :param output_limits: Nuevos límites para la salida del PID.
        """
        if Kp is not None:
            self.Kp = Kp
        if Ki is not None:
            self.Ki = Ki
        if Kd is not None:
            self.Kd = Kd
        if setpoint is not None:
            self.setpoint = setpoint
        if output_limits is not None:
            self.output_limits = output_limits

    def reset(self):
        """
        Reinicia el estado interno del controlador PID.
        """
        self._integral = 0
        self._prev_error = 0
        self._prev_derivative = 0
        self.output = 0
