import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from std_srvs.srv import Empty
import time

# Classe para controlar o robô TurtleBot

class TurtleBot(Node):
    def __init__(self):
        super().__init__('turtlebot')

        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        # Criar serviço de parada de emergência
        self.create_service(Empty, 'emergency_stop', self.emergency_stop)

        self.state = "stopped"

    # Função para parar o robô em caso de emergência
    def emergency_stop(self, request, response):
    
        self.stop()
        rclpy.shutdown()
        return response

    # Função para enviar comandos de movimento para o robô
    def move(self, state):
        msg = Twist()
        msg.linear = linear
        msg.angular = angular
        self.publisher_.publish(msg)

        # Movimentação no robÔ baseado no estado

        

    #
    def stop(self):
        stop_msg = Twist()
        self.publisher_.publish(stop_msg)

    def move_forward(self, speed: float, duration: float):
        # Criar um Vector3 com a velocidade linear
        self.move(Vector3(x=speed, y=0.0, z=0.0), Vector3(), duration)

    def move_backward(self, speed: float, duration: float):
        self.move(Vector3(x=-speed, y=0.0, z=0.0), Vector3(), duration)

    def rotate_left(self, speed: float, duration: float):
        # Criar um Vector3 com a velocidade angular
        self.move(Vector3(), Vector3(z=speed), duration)

    def rotate_right(self, speed: float, duration: float):
        self.move(Vector3(), Vector3(z=-speed), duration)

# Inicialização do ROS 2 e do nó do robô
def init_robot():
    rclpy.init()
    robot_turtle = TurtleBot()
    return robot_turtle

# Função para manter o nó ROS ativo
def spin_robot(robot):
    rclpy.spin(robot)
    robot.destroy_node()
    rclpy.shutdown()