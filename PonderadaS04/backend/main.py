from fastapi import FastAPI, WebSocket
from classes.TurtleBot import init_robot, spin_robot
import threading
import time
from rclpy.node import Node
import rclpy
from std_srvs.srv import Empty

# Inicializar o ROS 2 e criar o robô TurtleBot
robot = init_robot()

app = FastAPI()

# Thread separada para manter o nó ROS ativo já que vou estar controlando o robo o tempo todo
# Isso é necessário para manter o nó ROS ativo e receber os comandos de movimento
# **** IDEA DO GPT PQ FOI O ÚNICO JEITO QUE RODOU ****
def ros_spin():
    spin_robot(robot)

# Iniciar a thread para rodar o spin do ROS
spin_thread = threading.Thread(target=ros_spin)
spin_thread.start()

# Rota para receber comandos de movimento do robô
@app.websocket("/ws_control")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    try:
        while True:

            # Receber o comando de movimento do robô
            data = await websocket.receive_text()

            # Calcular a latência da mensagem
            current_time = time.time()
            message_data = data.split('|') # Quebra a string recebida em comando e tempo de envio
            command = message_data[0] # Comando de movimento
            sent_time = float(message_data[1])
            latency = current_time - sent_time # Calcula a latência da mensagem

            print(f"Msg recebida: {command} com {latency} s de latência")
            await websocket.send_text(f"{latency} segundos")

            # Interpretar o comando de movimento e enviar para o robô
            if command == "forward":
                robot.move_forward(0.5, 1.0)
            elif command == "backward":
                robot.move_backward(0.5, 1.0)
            elif command == "left":
                robot.rotate_left(0.5, 1.0)
            elif command == "right":
                robot.rotate_right(0.5, 1.0)
            elif command == "stop":
                robot.stop()
            elif command == "emergency_stop":
                robot.emergency_stop()
            else:
                print("Comando não reconhecido")

    except Exception as e:
        print(f"Erro: {e}")
        await websocket.close()


# Rota para parada de emergência do robô
@app.get("/emergency_stop")
async def emergency_stop():

    # Verificar se o ROS 2 já foi iniciado
    if rclpy.ok() == False:
        rclpy.init()
    else:
        print("ROS/Rclpy já iniciado")

    # Criar um nó para chamar o serviço de parada de emergência
    node =  Node('emergency_stop')

    # Criar um cliente para chamar o serviço de parada de emergência que foi criado no nó do robô
    client = node.create_client(Empty, 'emergency_stop')

    while not client.wait_for_service(timeout_sec=1.0):
        print('Serviço não disponível, esperando...')

    request = Empty.Request()
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        # Desligar esse nó do ROS 2
        node.destroy_node()
        rclpy.shutdown()
        return {"status": "success"}
    
    else:
        return {"status": "failed"}