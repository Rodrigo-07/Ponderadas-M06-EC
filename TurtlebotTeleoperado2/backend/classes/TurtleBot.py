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

        self.moviment_timer = self.create_timer(0.1, self.move)

    # Função para parar o robô em caso de emergência
    def emergency_stop(self, request, response):
    
        self.stop()
        rclpy.shutdown()
        return response

    # Função para enviar comandos de movimento para o robô
    def move(self):
        msg = Twist()

        # Movimentação no robÔ baseado no estado dele
        if self.state == "forward":
            msg.linear.x = 0.5
        elif self.state == "backward":
            msg.linear.x = -0.5
        elif self.state == "left":
            msg.angular.z = 0.5
        elif self.state == "right":
            msg.angular.z = -0.5
        elif self.state == "stopped":
            self.stop()
        elif self.state == "emergency_stop":
            self.emergency_stop()
        else:
            print("Comando não reconhecido")

        self.publisher_.publish(msg)

    #
    def stop(self):
        stop_msg = Twist()
        self.publisher_.publish(stop_msg)

    def move_forward(self, speed: float):
        # Criar um Vector3 com a velocidade linear
        self.move(Vector3(x=speed, y=0.0, z=0.0), Vector3())

    def move_backward(self, speed: float):
        self.move(Vector3(x=-speed, y=0.0, z=0.0), Vector3())

    def rotate_left(self, speed: float):
        # Criar um Vector3 com a velocidade angular
        self.move(Vector3(), Vector3(z=speed))

    def rotate_right(self, speed: float):
        self.move(Vector3(), Vector3(z=-speed))

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