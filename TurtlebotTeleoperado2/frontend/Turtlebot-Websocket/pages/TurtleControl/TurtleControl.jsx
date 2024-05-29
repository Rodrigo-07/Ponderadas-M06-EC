import React, { useState, useEffect, useCallback, useRef } from 'react';
import Video_ws from '../../src/components/videoWS/videoWS';

const TurtleControl = () => {
    const [wsControl, setWsControl] = useState(null);
    const [latency, setLatency] = useState(null);
    const lastCommandRef = useRef(null);
    const isKeyDownRef = useRef({});
    const [lastCommand, setLastCommand] = useState('');

    useEffect(() => {
        const websocketControl = new WebSocket('ws://127.0.0.1:8000/ws_control');
        setWsControl(websocketControl);

        websocketControl.onopen = () => {
            console.log('Conexão com o WebSocket de movimentação estabelecida');
        };
        
        websocketControl.onmessage = (event) => {
            setLatency(event.data);
            console.log(event.data);
        };

        websocketControl.onclose = () => {
            console.log('WebSocket de movimentação desconectado');
        };

        return () => {
            websocketControl.close();
        };
    }, []);

    const sendCommand = (command) => {
        if (wsControl && command !== lastCommandRef.current) {
            const timestamp = Date.now() / 1000;
            wsControl.send(`${command}|${timestamp}`);
            console.log(`${command}|${timestamp}`);
            console.log(`Comando enviado: ${command} às ${timestamp}`);
            lastCommandRef.current = command;
            
            if (command === 'forward') {
                setLastCommand('frente');
            } else if (command === 'backward') {
                setLastCommand('trás');
            } else if (command === 'left') {
                setLastCommand('esquerda');
            } else if (command === 'right') {
                setLastCommand('direita');
            }
        }
    };

    const callEmergencyStop = async () => {
        try {
            const response = await fetch('http://127.0.0.1:8000/emergency_stop', {
                method: 'GET',
            });
            if (response.ok) {
                console.log('Emergency stop called successfully');
            } else {
                console.error('Failed to call emergency stop');
            }
        } catch (error) {
            console.error('Error calling emergency stop:', error);
        }
    };

    const handleKeyDown = useCallback((event) => {
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
                case ' ':
                    callEmergencyStop();
                    break;
                default:
                    break;
            }
        }
    }, [wsControl]);

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
                sendCommand('stopped');
                break;
            default:
                break;
        }
    }, [wsControl]);

    useEffect(() => {
        window.addEventListener('keydown', handleKeyDown);
        window.addEventListener('keyup', handleKeyUp);
        return () => {
            window.removeEventListener('keydown', handleKeyDown);
            window.removeEventListener('keyup', handleKeyUp);
        };
    }, [handleKeyDown, handleKeyUp]);

    const handleMouseDown = (direction) => {
        console.log('Mouse down:', direction);
        sendCommand(direction);
    };

    const handleMouseUp = () => {
        sendCommand('stopped');
    };

    return (
        <div>
            <h1>Turtlebot3 Control 🐢</h1>
            <div>
                <h2>Webcam</h2>
                <Video_ws />
            </div>
            <div>
                <h2>Use as setas do teclado ou botões para controlar</h2>
                <div style={{ display: 'flex', flexDirection: 'column', alignItems: 'center', marginBottom: '35px'}}>
                    <button onMouseDown={() => handleMouseDown('forward')} onMouseUp={handleMouseUp}>Cima</button>
                    <div style={{ display: 'flex', justifyContent: 'center', margin: '10px 0' }}>
                        <button onMouseDown={() => handleMouseDown('left')} onMouseUp={handleMouseUp} style={{ marginRight: '10px' }}>Esquerda</button>
                        <button onMouseDown={() => handleMouseDown('right')} onMouseUp={handleMouseUp}>Direita</button>
                    </div>
                    <button onMouseDown={() => handleMouseDown('backward')} onMouseUp={handleMouseUp}>Trás</button>
                </div>
                {latency && <div>Latência do envio do comando: {latency}</div>}
            </div>
            <div>
                <h2>Log robô</h2>
                <div>Robô indo para {lastCommand}</div>
            </div>
            <div>
                <h2>Parada de emergência</h2>
                <button onClick={callEmergencyStop}>Parada de emergência</button>
            </div>
            <div>
                <h2>Comandos</h2>
                <ul>
                    <li>Setas do teclado ou W, A, S, D: Movimentar</li>
                    <li>Barra de espaço: Parada de emergência</li>
                </ul>
            </div>
        </div>
    );
};

export default TurtleControl;
