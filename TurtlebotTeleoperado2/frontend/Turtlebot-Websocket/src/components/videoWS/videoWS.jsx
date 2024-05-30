import React, { useEffect } from 'react';

const Video_ws = () => {

    // useEffect para facilitar a conexão com o WebSocket da câmera
    useEffect(() => {

        // Criação do WebSocket da câmera
        const websocketCamera = new WebSocket('ws://127.0.0.1:8000/ws_camera');

        websocketCamera.onopen = () => {
            console.log('Conexão com o WebSocket da câmera estabelecida');
        };

        websocketCamera.onmessage = (event) => {
            const message = JSON.parse(event.data);
            const video = document.getElementById('video');
            video.src = `data:image/jpg;base64,${message.image}`;
            const latencia = document.getElementById('latencia');
            latencia.innerHTML = `Latência da câmera: ${Date.now() - message.time} ms`;
        };

        websocketCamera.onclose = () => {
            console.log('WebSocket da câmera desconectado');
        };

        return () => {
            websocketCamera.close();
        };
    }, []);

    return (
        <div>
            <img id="video" width="640" height="480" src="" alt="Webcam Feed" />
            <h3 id="latencia"></h3>
        </div>
    );
};

export default Video_ws;
