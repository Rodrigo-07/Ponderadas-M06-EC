import React from "react";

const Video_ws = () => {

    const websocketConnection = new WebSocket('ws://127.0.0.1:8000/ws_camera')

    websocketConnection.onopen = () => {
        console.log('Conexão com o WebSocket estabelecida')
    }

    websocketConnection.onmessage = (event) => {
        console.log(event.data)
    }

    websocketConnection.onclose = () => {
        console.log('WebSocket desconectado')
    }

    // Receber a base64 recibido do servidor e exibir no vídeo
    websocketConnection.onmessage = (event) => {
        // const video = document.getElementById('video')

        // Salvar Json em uma variável
        const mensage = JSON.parse(event.data)

        console.log(mensage.time)

        // Montar msg base64
        video.src = `data:image/jpg;base64,${mensage.image}`

        const latencia = document.getElementById('latencia')
        latencia.innerHTML = `Latência: ${Date.now() - mensage.time} ms`
    }

    // Calcular latencia

    return (
        <div>
            <img id="video" width="640" height="480" src=""></img>

            <h3 id="latencia"></h3>

        </div>
    )

}

export default Video_ws