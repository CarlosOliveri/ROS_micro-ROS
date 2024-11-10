#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32  # Cambiamos de Float32 a Int32
from std_msgs.msg import Int32MultiArray
from std_srvs.srv import Trigger

class MotorNode(Node):
    def __init__(self):
        super().__init__('motor_node')

        self.client = self.create_client(Trigger, 'analize_frame')

        # Crear suscripciones
        self.create_subscription(
            Int32,
            '/motor_current_speed_1',  # Cambiado para que coincida con micro-ROS
            self.motor_speed_callback_1,
            10
        )

        self.create_subscription(
            Int32,
            '/motor_current_speed_2',  # Cambiado para que coincida con micro-ROS
            self.motor_speed_callback_2,
            10
        )

        self.create_subscription(
            Int32MultiArray,
            '/ultrasonic_sensor',  # Cambiado para que coincida con micro-ROS
            self.ultrasonic_callback,
            10
        )

        self.create_subscription(
            Int32MultiArray,
            '/PID_debugger',  # Cambiado para que coincida con micro-ROS
            self.PID_callback,
            10
        )

        # Nueva suscripción para recibir la velocidad deseada
        self.create_subscription(
            Int32,
            '/set_motor_speed_1',  # Cambiado de Float32 a Int32
            self.set_speed_callback_1,
            10
        )

        self.create_subscription(
            Int32,
            '/set_motor_speed_2',  # Cambiado de Float32 a Int32
            self.set_speed_callback_2,
            10
        )

        # Nueva suscripción para recibir el tiempo de muestreo de los sensores ultrasonicos
        self.create_subscription(
            Int32,
            '/set_sample_time',  # Cambiado de Float32 a Int32
            self.set_sample_time_callback,
            10
        )

        # Crear publicador para la velocidad deseada
        self.publisher_speed_1 = self.create_publisher(Int32, 'motor_1_speed', 10)  # Cambiado a Int32
        # Variable para almacenar la velocidad deseada
        self.desired_speed_1 = 0

        # Crear publicador para la velocidad deseada
        self.publisher_speed_2 = self.create_publisher(Int32, 'motor_2_speed', 10)  # Cambiado a Int32
        # Variable para almacenar la velocidad deseada
        self.desired_speed_2 = 0

        self.publisher_sample_time = self.create_publisher(Int32, 'sample_time',10)
        self.desired_sample_time = 1

    def motor_speed_callback_1(self, msg):
        # Ahora recibimos un Int32
        self.get_logger().info(f'Current motor speed 1: {msg.data} rad/min')

    def motor_speed_callback_2(self, msg):
        # Ahora recibimos un Int32
        self.get_logger().info(f'Current motor speed 2: {msg.data} rad/min')
        self.get_logger().info("---------------------")

    # Callback para recibir la velocidad deseada
    def set_speed_callback_1(self, msg):
        # Ahora recibimos un Int32
        self.desired_speed_1 = msg.data
        self.get_logger().info(f'Received new desired speed 1: {self.desired_speed_1} rad/min')
        self.publish_desired_speed_1()
        
    # Callback para recibir la velocidad deseada
    def set_speed_callback_2(self, msg):
        # Ahora recibimos un Int32
        self.desired_speed_2 = msg.data
        self.get_logger().info(f'Received new desired speed 2: {self.desired_speed_2} rad/min')
        self.publish_desired_speed_2()

    # Callback para recibir el tiempo de muestreo deseado
    def set_sample_time_callback(self, msg):
        # Ahora recibimos un Int32
        self.desired_sample_time = msg.data
        self.get_logger().info(f'Received new desired sample time: {self.desired_sample_time} seg.')
        self.publish_desired_sample_time()

    # Callback para recibir la distancia
    def ultrasonic_callback(self, msg):
        # Ahora recibimos un Int32
        distances = msg.data
        self.get_logger().info("-------------------------------")
        self.get_logger().info(f'Sensor 1: {distances[0]} cm, Sensor 2: {distances[1]} cm, Sensor 3: {distances[2]} cm')
        self.get_logger().info("-------------------------------")

    def PID_callback(self,msg):
        datos = msg.data
        self.get_logger().info(f'speed izquierda: {datos[0]},speed derecha: {datos[1]}')
        self.get_logger().info(f'Error: {datos[2]} °/s')
        self.get_logger().info(f'Conteo encoder izquierdo: {datos[3]} pulsos, Conteo encoder derecho: {datos[4]} pulsos')
        self.get_logger().info("-------------------------------")

    # Método para publicar la velocidad deseada
    def publish_desired_speed_1(self):
        msg = Int32()
        msg.data = self.desired_speed_1  # Publicar la velocidad deseada como Int32
        self.publisher_speed_1.publish(msg)
        self.get_logger().info(f'Publishing desired motor speed 1: {msg.data} RPM')

    # Método para publicar la velocidad deseada
    def publish_desired_speed_2(self):
        msg = Int32()
        msg.data = self.desired_speed_2  # Publicar la velocidad deseada como Int32
        self.publisher_speed_2.publish(msg)
        self.get_logger().info(f'Publishing desired motor speed 2: {msg.data} RPM')

    # Método para publicar el tiempo de muestreo deseado
    def publish_desired_sample_time(self):
        msg = Int32()
        msg.data = self.desired_sample_time # Publicar la velocidad deseada como Int32
        self.publisher_sample_time.publish(msg)
        self.get_logger().info(f'Publishing desired sample time: {msg.data} m.')

def main(args=None):
    rclpy.init(args=args)
    node = MotorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()