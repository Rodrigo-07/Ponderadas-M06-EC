# Turtlebot teleoperado pt. 2

Nessa ponderada deveria ser feito uma interface para teleoperar um Turtlebot3, com controle da velocidade em certas direções e visualização da "imagem" de uma câmera no robô. Para fazer isso, fiz uma interface gráfica em React (Vite) e um servidor em Python (FastAPI) que se comunicam via WebSocket.

Essa ponderada segue a mesma estrutura da ponderada passada, porém nessa tem o a conexão com a câmera do seu dispositivo, já que o simulador de TurtleBot3 não tem câmera. Além disso, foi mudado a lógica de movimetaçãod do robô para que ele se mova enquanto a tecla é precisionada.

## Como rodar

Para rodar, execute o passo a passo do backend, frontend e do robô:

### Backend

Primeiro criei um venv e instale as dependências do projeto

```bash

cd backend

python3 -m venv venv

source venv/bin/activate

pip install -r requirements.txt

```

Agora pode rodar o servidor com o seguite comando:

```bash

uvicorn main:app --reload

```

### Frontend

```bash

cd frontend/Turtlebot-Websocket

npm install

npm run dev

```

### Simulador de Turtlebot3 (Webots)

Caso não tenha o Webots instalado, siga as instruções em: [Webots](https://cyberbotics.com/doc/guide/installation-procedure)

```bash

ros2 launch webots_ros2_turtlebot robot_launch.py

```

Se for usar o robô real basta rodar o bringup do Turtlebot3.

```bash

ros2 launch turtlebot3_bringup robot_launch.py

```

## Demonstração

https://youtu.be/12QIdJa-hig
