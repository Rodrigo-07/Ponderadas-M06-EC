import React, { useState, useEffect, useCallback, useRef } from 'react';

const TurtleControl = () => {
    // State para armazenar o WebSocket
    const [ws, setWs] = useState(null);
    const [latency, setLatency] = useState(null); // State para armazenar a latência
    const lastCommandRef = useRef(null); // Ref para armazenar o último comando enviado
    const isKeyDownRef = useRef({}); // Ref para armazenar quais teclas estão pressionadas

    useEffect(() => {

        // Criação do WebSocket
        const websocket = new WebSocket('ws://127.0.0.1:8000/ws_control');
        setWs(websocket);
        
        // Eventos do WebSocket
        websocket.onmessage = (event) => {
            setLatency(event.data);
            console.log(event.data);
        };

        // Evento de desconexão
        websocket.onclose = () => {
            console.log('WebSocket disconecatado');
        };

        return () => {
            websocket.close();
        };
    }, []);

    // Função para enviar comandos ao servidor
    // COloquei em uma função pra poder trabalhar organizar os comandos enviados (evitando comandos duplos) e pegar o timestamp
    const sendCommand = (command) => {
        // Verifica se o WebSocket está aberto e se o comando é diferente do último comando enviado
        // *** IDEA DE ÚLTIMO COMANDO FOI DO GPT ***
        if (ws && command !== lastCommandRef.current) {
            const timestamp = Date.now() / 1000; // s
            ws.send(`${command}|${timestamp}`);
            console.log(`Comando enviado: ${command} às ${timestamp}`);
            lastCommandRef.current = command;
        }
    };

    // Função para receber os eventos de teclado
    const handleKeyDown = useCallback((event) => {
        // Verifica se a tecla está pressionada
        if (!isKeyDownRef.current[event.key]) {
            isKeyDownRef.current[event.key] = true;
            switch (event.key) {
                case 'ArrowUp':
                case 'w':
                case 'W':
                    sendCommand('forward');
                    break;
                case 'ArrowDown':
                case 's':
                case 'S':
                    sendCommand('backward');
                    break;
                case 'ArrowLeft':
                case 'a':
                case 'A':
                    sendCommand('left');
                    break;
                case 'ArrowRight':
                case 'd':
                case 'D':
                    sendCommand('right');
                    break;
                default:
                    break;
            }
        }
    }, [ws]);

    // Função para quando a tecla é solta, entçao eu envio o comando de parar
    const handleKeyUp = useCallback((event) => {
        isKeyDownRef.current[event.key] = false;
        switch (event.key) {
            case 'ArrowUp':
            case 'w':
            case 'W':
            case 'ArrowDown':
            case 's':
            case 'S':
            case 'ArrowLeft':
            case 'a':
            case 'A':
            case 'ArrowRight':
            case 'd':
            case 'D':
                sendCommand('stop');
                break;
            default:
                break;
        }
    }, [ws]);

    // Adiciona os eventos de teclado na janela do navegador e remove quando o componente é desmontado
    useEffect(() => {
        window.addEventListener('keydown', handleKeyDown);
        window.addEventListener('keyup', handleKeyUp);
        return () => {
            window.removeEventListener('keydown', handleKeyDown);
            window.removeEventListener('keyup', handleKeyUp);
        };
    }, [handleKeyDown, handleKeyUp]);

    const handleMouseDown = (direction) => {
        sendCommand(direction);
    };

    const handleMouseUp = () => {
        sendCommand('stop');
    };

    return (
        <div>
            <h1>Turtlebot3 Control 🐢</h1>
            <div>
                <h2>Use as setas do teclado ou botões para controlar</h2>
                <div style={{ display: 'flex', flexDirection: 'column', alignItems: 'center', marginBottom: '35px'}}>
                    <button onMouseDown={() => handleMouseDown('forward')} onMouseUp={handleMouseUp}>Cima</button>
                    <div style={{ display: 'flex', justifyContent: 'center', margin: '10px 0' }}>
                        <button onMouseDown={() => handleMouseDown('left')} onMouseUp={handleMouseUp} style={{ marginRight: '10px' }}>Esquerda</button>
                        <button onMouseDown={() => handleMouseDown('right')} onMouseUp={handleMouseUp}>Direito</button>
                    </div>
                    <button onMouseDown={() => handleMouseDown('backward')} onMouseUp={handleMouseUp}>Trás</button>
                </div>
                {latency && <div>Latencia do envio do comando: {latency}</div>}
            </div>
        </div>
    );
};

export default TurtleControl;
