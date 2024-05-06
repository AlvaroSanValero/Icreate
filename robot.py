from pycreate2 import Create2
import time
import math

class Robot:
    # wheel_radius=??, reduction=??, encoder_res=??, between_wheels_dist=??
    def __init__(self, port="/dev/ttyUSB0", speed=100, wheel_radius=1, reduction=1, encoder_res=1, between_wheels_dist=1):
        if not hasattr(Robot, initPos):
            Robot.initPos = (0, 0)

        if not hasattr(Robot, initOrientation):
            Robot.initOrientation = 0.

        # Iniciar robot en modo `Safe`
        self.robot = Create2(port)
        self.robot.start()
        self.robot.safe()

        self.speed = speed

        # Posición y orientación del robot (valores esperados, no los reales)
        self.x, self.y = Robot.initPos
        self.theta = Robot.initOrientation

        # Distancias a obstáculos en la parte delantera izquierda, frontal y derecha
        self.distance_sensors = [10, 10, 10]
        self.encoder_values = Robot.initPos
        self.conversion_factor: float = 2 * math.pi * wheel_radius / (reduction * encoder_res)
        self.between_wheels_dist = between_wheels_dist

    def move(self, distance):
        # Movimiento del robot en base a una distancia dada y actualización de la posición
        self.robot.drive_direct(self.speed, self.speed)
        time.sleep(distance / self.speed)
        self.robot.drive_stop()
        self.x += distance * math.cos(math.radians(self.theta))
        self.y += distance * math.sin(math.radians(self.theta))

        __update_all_sensors()

    def rotate(self, angle):
        # Rotación del robot en base a un ángulo dado y actualización de la orientación
        time.sleep(abs(angle) / (90 * (self.speed / 100)))  # Calcula el tiempo de espera dinámicamente
        self.robot.drive_stop()
        self.theta += angle

        __update_all_sensors()

    def update_distance_sensors(self, distances):
        # Actualización de las distancias de los sensores de distancia
        self.distance_sensors = distances

    def __update_all_sensors():
        sensors = self.robot.get_sensors()
        self.encoder_values = (sensors.encoder_counts_left, sensors.encoder_counts_right)

class OdometryCalculator:
    def __init__(self, robot):
        self.robot = robot

    def calculate_odometry(self):
        # Cálculo de la odometría y muestra de la información
        print(f"Odometría - Posición (x, y): ({self.robot.x}, {self.robot.y}) - Orientación: {self.robot.theta}")
        return (self.robot.encoder_values * self.robot.conversion_factor)

    # Devuelve el error en la posición y orientación del robot, restando la posición real (calculada con encoders) con los valores esperados 
    def get_error(self) -> tuple[int, int]:
        odometry = calculate_odometry()
        encoder_dist = (odometry[0][1] + odometry[0][0]) / 2
        encoder_theta = (odometry[0][1] - odometry[0][0]) / self.robot.between_wheels_dist
        real_x = encoder_dist * math.cos(self.robot.theta + encoder_theta)
        real_y = encoder_dist * math.sin(self.robot.theta + encoder_theta)

        return math.abs((self.robot.x - real_x, self.robot.y - real_y))

# Ejemplo de uso
robot = Robot()
odometry_calculator = OdometryCalculator(robot)

# Movimiento del robot y actualización de las distancias de los sensores
robot.move(500)  # Por ejemplo, mueve 500 mm hacia adelante
robot.rotate(90)  # Por ejemplo, rota 90 grados a la izquierda
robot.update_distance_sensors([8, 12, 9])  # Supongamos que los sensores detectan estas distancias

# Cálculo de la odometría después de cada movimiento
odometry_calculator.calculate_odometry()

