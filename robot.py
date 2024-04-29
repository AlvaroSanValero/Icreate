from pycreate2 import Create2
import time
import math

class Robot:
    def __init__(self, port="/dev/ttyUSB0"):
        self.robot = Create2(port)
        self.robot.start()
        self.robot.safe()

        # Posición y orientación del robot
        self.x = 0
        self.y = 0
        self.theta = 0

        # Distancias a obstáculos en la parte delantera izquierda, frontal y derecha
        self.distance_sensors = [10, 10, 10]

    def move(self, distance):
        # Movimiento del robot en base a una distancia dada y actualización de la posición
        speed = 100  # Velocidad del movimiento (en mm/s)
        self.robot.drive_direct(speed, speed)
        time.sleep(distance / speed)
        self.robot.drive_stop()
        self.x += distance * math.cos(math.radians(self.theta))
        self.y += distance * math.sin(math.radians(self.theta))

    def rotate(self, angle):
        # Rotación del robot en base a un ángulo dado y actualización de la orientación
        speed = 100  # Velocidad angular de la rotación (en mm/s)
        time.sleep(abs(angle) / (90 * (speed / 100)))  # Calcula el tiempo de espera dinámicamente
        self.robot.drive_stop()
        self.theta += angle

    def update_distance_sensors(self, distances):
        # Actualización de las distancias de los sensores de distancia
        self.distance_sensors = distances

class OdometryCalculator:
    def __init__(self, robot):
        self.robot = robot

    def calculate_odometry(self):
        # Cálculo de la odometría y muestra de la información
        print(f"Odometría - Posición (x, y): ({self.robot.x}, {self.robot.y}) - Orientación: {self.robot.theta}")

# Ejemplo de uso
robot = Robot()
odometry_calculator = OdometryCalculator(robot)

# Movimiento del robot y actualización de las distancias de los sensores
robot.move(500)  # Por ejemplo, mueve 500 mm hacia adelante
robot.rotate(90)  # Por ejemplo, rota 90 grados a la izquierda
robot.update_distance_sensors([8, 12, 9])  # Supongamos que los sensores detectan estas distancias

# Cálculo de la odometría después de cada movimiento
odometry_calculator.calculate_odometry()

