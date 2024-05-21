# PonderadS04

Nessa ponderada deveria ser feito uma interface para teleoperar um Turtlebot3, com controle da velocidade em certas direções. Para fazer isso, fiz uma interface gráfica em React (Vite) e um servidor em Python (FastAPI) que se comunicam via WebSocket.

## Como rodar

### Backend

```bash

cd backend

pip install -r requirements.txt

uvicorn main:app --reload

```

### Frontend

```bash

cd frontend/Turtlebot-Websocket

npm install

npm run dev

```

### Simulador de Turtlebot3 (Webots)

```bash

ros2 launch webots_ros2_turtlebot robot_launch.py

```

## Demonstração

![Demonstração](https://youtu.be/kUAR_7Hk6uQ)
