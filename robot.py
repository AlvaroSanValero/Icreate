from pycreate2 import Create2
import time
import math

class Robot:
    # wheel_radius=??, reduction=??, encoder_res=??, between_wheels_dist=??
    def __init__(self, port="/dev/tty.usbserial-DN04GLBY", speed=500, wheel_radius=36, reduction=508.8, encoder_res=1, between_wheels_dist=235):
        Robot.initPos = (0, 0)
        Robot.initOrientation = 0
        Robot.errorThres = 0.0001

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

    def __update_all_sensors(self):
        sensors = self.robot.get_sensors()
        self.encoder_values = (sensors.encoder_counts_left, sensors.encoder_counts_right)

    def move(self, distance):
        current_distance = 0
        self.robot.drive_direct(self.speed, self.speed)
        while current_distance < distance:
            prev_encoder = self.encoder_values
            self.__update_all_sensors()
            encoder_dif = (self.encoder_values[0] - prev_encoder[0], self.encoder_values[1] - prev_encoder[1])
            current_distance += self.conversion_factor * encoder_dif[0]
        self.robot.drive_stop()	


        # self.robot.drive_direct(self.speed, self.speed)
        # time.sleep(distance / self.speed)
        # self.robot.drive_stop()

        # self.x += distance * math.cos(math.radians(self.theta))
        # self.y += distance * math.sin(math.radians(self.theta))

    def log(self, message):
        with open("robot_log7.txt", 'a') as file:
            file.write(message + '\n')

    def rotate(self, angle):
        counter = 0
        # Multiplicador del counter para reducir velocidad
        counterLim = 100
        current_rotation = 0
        angle = angle * math.pi / 180.
        self.log("[INFO] Target angle: " + str(angle))
        # se presupone que inicialmente el error es positivo
        flag = True
        error = angle
        # girar al sentido agujas del reloj
        self.robot.drive_direct(self.speed, -self.speed)
        # comparamos el error hasta que sea menor que nuestro threshold
        
        while abs(error) >= self.errorThres:
                #
                # comparamos hacia que lado tiene que girar en funcion de su error
                #
                # tiene que girar al contrario agujas del reloj
                if error > 0:
                    # si anteriormente su error era negativo
                    if not flag:
                        counter += 1
                        flag = True
                        self.robot.drive_stop()
                        self.robot.drive_direct(self.speed, -self.speed)

                        if self.speed > 20 + counterLim:
                            self.speed = self.speed - (counterLim)
                        else: 
                            self.speed = 20
                        self.log("[INFO] Speed adjusted (Clockwise): " + str(self.speed))
                # tiene que girar al sentido de agujas del reloj
                elif error < 0:
                    # si anteriormente su error era positivo
                    if flag: 
                        counter += 1                   
                        flag = False
                        self.robot.drive_stop()
                        self.robot.drive_direct(-self.speed, self.speed)
                        if self.speed > 20 + counterLim:
                            self.speed = self.speed - (counterLim)
                        else: 
                            self.speed = 20
                        self.log("[INFO] Speed adjusted (Counter-clockwise): " + str(self.speed))
                
                prev_encoder = self.encoder_values
                self.__update_all_sensors()
                encoder_dif = (self.encoder_values[0] - prev_encoder[0], self.encoder_values[1] - prev_encoder[1])
                current_rotation -= 2 * encoder_dif[0] * self.conversion_factor / self.between_wheels_dist
                #self.log("[INFO] Encoder values: " + str(self.encoder_values))
                #self.log("[INFO] Current rotation: " + str(current_rotation))
                
                error = angle-current_rotation
                self.log("[INFO] Error: " + str(error))
        self.robot.drive_stop()
        self.log("[INFO] Rotation complete. Stopping robot.")
                
        # Rotación del robot en base a un ángulo dado y actualización de la orientación
        # self.robot.drive_direct(-self.speed, self.speed)
        # time.sleep(abs(angle) /((2*self.speed)/self.between_wheels_dist) ) # Calcula el tiempo de espera dinámicamente
        # self.robot.drive_stop()
        # self.theta += angle

        #__update_all_sensors()

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

# Movimiento del robot y actualización de las distancias de los sensores
#robot.move(500)  # Por ejemplo, mueve 500 mm hacia adelante
log_file = "robot_log.txt"
robot.rotate(90)  # Por ejemplo, rota 90 grados a la izquierda
# robot.update_distance_sensors([8, 12, 9])  # Supongamos que los sensores detectan estas distancias

# Cálculo de la odometría después de cada movimiento
# odometry_calculator.calculate_odometry()

