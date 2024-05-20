from fastapi import FastAPI, WebSocket

app = FastAPI()

# Endpoint para o websocket (TESTE)
@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    # Aceita a conexão do cliente
    await websocket.accept()

    # Dentro de um loop infinito, o servidor estára "sempre" esperando por mensagens
    while True:

        # Recebe a mensagem do cliente
        data = await websocket.receive_text()
        print(data)
        await websocket.send_text(f"Message text was: {data}")




