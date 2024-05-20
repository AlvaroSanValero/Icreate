from pycreate2 import Create2
import time
import math

class Robot:
    # wheel_radius=??, reduction=??, encoder_res=??, between_wheels_dist=??
    def __init__(self, port="/dev/tty.usbserial-DN04H0E4", speed=500, wheel_radius=36, reduction=508.8, encoder_res=1, between_wheels_dist=235):
        Robot.initPos = (0, 0)
        Robot.initOrientation = 0.
        Robot.errorThres = 2
        Robot.correctionFactor = 2
        Robot.errorRotate90 = 0.002

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
        self.encoder_values = (self.robot.get_sensors().encoder_counts_left, self.robot.get_sensors().encoder_counts_right)
        print(self.encoder_values)
        self.conversion_factor = 2 * math.pi * wheel_radius / (reduction * encoder_res)
        print(self.conversion_factor)
        self.between_wheels_dist = between_wheels_dist

    def log_error(self, message):
        with open("log_error.txt", 'a') as file:
            file.write(message + '\n')

    def log_result(self, message):
        with open("log_result.txt", 'a') as file:
            file.write(message + '\n')

    def __update_all_sensors(self):
        sensors = self.robot.get_sensors()
        self.encoder_values = (sensors.encoder_counts_left, sensors.encoder_counts_right)

    def rec_move(self, distance, speed, corrections):
        self.log_result(f"Iteración de movimiento n. {corrections}\nDistancia: {distance}\nVelocidad: {speed}")
        current_distance = 0
        self.robot.drive_direct(speed, speed)
        while abs(current_distance) <= distance:
                prev_encoder = self.encoder_values
                print(prev_encoder)
                self.__update_all_sensors()
                encoder_dif = (self.encoder_values[0] - prev_encoder[0], self.encoder_values[1] - prev_encoder[1])
                current_distance += self.conversion_factor * encoder_dif[0]

        self.robot.drive_stop()

        error = abs(current_distance) - distance
        self.log_error(f"El error de movimiento es: {error}")
        if error > Robot.errorThres and corrections - 1 < 20 and abs(speed) // Robot.correctionFactor > 20:
            self.rec_move(error, int((-1 if speed > 0 else 1) * (abs(speed) // Robot.correctionFactor)), corrections + 1)

    def move(self, distance):
        self.rec_move(distance * (-1 if distance < 0 else 1), self.speed * (-1 if distance < 0 else 1) // 2, 0)

    def palante(self):
        self.robot.drive_direct(self.speed, self.speed)
        time.sleep(1)
        self.robot.drive_stop()

    def patras(self):
        self.robot.drive_direct(-self.speed, -self.speed)
        time.sleep(1)
        self.robot.drive_stop()


    def rotate(self, angle):
        speedRotation = int(self.speed) // 4
        #error inversamente proporcional al valor del angulo de movimiento
        if (abs(angle) >= 90):
            errorRotation = abs(angle) * self.errorRotate90 / 90
        else:
            errorRotation = self.errorRotate90
        print(errorRotation)
        counter = 0
        speedLim = 20
        # Multiplicador del counter para reducir velocidad
        counterLim = 25
        current_rotation = 0
        angle = angle * math.pi / 180.
        # se presupone que inicialmente el error es positivo
        flag = True
        error = angle
        # girar al sentido agujas del reloj
        self.robot.drive_direct(speedRotation, -speedRotation)
        # comparamos el error hasta que sea menor que nuestro threshold
        
        while abs(error) >= errorRotation:
                #
                # comparamos hacia que lado tiene que girar en funcion de su error
                #
                # tiene que girar al contrario agujas del reloj
                if error > 0 and not flag:
                    counter += 1
                    flag = True
                    self.robot.drive_stop()
                    self.robot.drive_direct(speedRotation, -speedRotation)

                    if speedRotation > speedLim + counterLim:
                        speedRotation = speedRotation - (counterLim)
                    else: 
                        speedRotation = speedLim
                        time.sleep(0.2)

                
                # tiene que girar al sentido de agujas del reloj
                elif error < 0 and flag:
                    counter += 1                   
                    flag = False
                    self.robot.drive_stop()
                    self.robot.drive_direct(-speedRotation, speedRotation)

                    if speedRotation > speedLim + counterLim:
                        speedRotation = speedRotation - (counterLim)
                    else: 
                        speedRotation = speedLim
                        time.sleep(0.2)

                    
                prev_encoder = self.encoder_values
                self.__update_all_sensors()
                encoder_dif = (self.encoder_values[0] - prev_encoder[0], self.encoder_values[1] - prev_encoder[1])
                current_rotation -= 2 * encoder_dif[0] * self.conversion_factor / self.between_wheels_dist
                #self.log("[INFO] Encoder values: " + str(self.encoder_values))
                #self.log("[INFO] Current rotation: " + str(current_rotation))
                
                error = angle - current_rotation
                
        self.robot.drive_stop()

    def update_distance_sensors(self, distances):
        # Actualización de las distancias de los sensores de distancia
        self.distance_sensors = distances

class OdometryCalculator:
   def __init__(self, robot):
        self.robot = robot

#   def calculate_odometry(self):
        # Cálculo de la odometría y muestra de la información
#        print(f"Odometría - Posición (x, y): ({self.robot.x}, {self.robot.y}) - Orientación: {self.robot.theta}")
#        return (self.robot.encoder_values * self.robot.conversion_factor)

    # Devuelve el error en la posición y orientación del robot, restando la posición real (calculada con encoders) con los valores esperados 
#   def get_error(self) :
#        odometry = calculate_odometry()
#        encoder_dist = (odometry[0][1] + odometry[0][0]) / 2
#        encoder_theta = (odometry[0][1] - odometry[0][0]) / self.robot.between_wheels_dist
#        real_x = encoder_dist * math.cos(self.robot.theta + encoder_theta)
#        real_y = encoder_dist * math.sin(self.robot.theta + encoder_theta)

#        return math.abs((self.robot.x - real_x, self.robot.y - real_y))

# Ejemplo de uso
robot = Robot()
odometry_calculator = OdometryCalculator(robot)

log_error = "log_error.txt"
log_result = "log_result.txt"

# Movimiento del robot y actualización de las distancias de los sensores
#for _ in range(4):
# Subir a la derecha
robot.move(200)
time.sleep(1)
robot.rotate(90)
time.sleep(1)

robot.move(200)
time.sleep(1)
robot.rotate(90)
time.sleep(1)

robot.move(200)
time.sleep(1)
robot.rotate(90)
time.sleep(1)

robot.move(200)
time.sleep(1)
robot.rotate(90)
time.sleep(1)


    #robot.move(-500)   # Por ejemplo, mueve 500 mm hacia adelante
    # robot.rotate(90)  # Por ejemplo, rota 90 grados a la izquierda
# robot.update_distance_sensors([8, 12, 9])  # Supongamos que los sensores detectan estas distancias

# robot.palante()
# robot.patras()

# robot.move_backwards()

# Cálculo de la odometría después de cada movimiento
# odometry_calculator.calculate_odometry()
