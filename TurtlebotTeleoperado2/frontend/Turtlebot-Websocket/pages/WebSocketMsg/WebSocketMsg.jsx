import React, { useState, useEffect } from 'react';

const WebSocketMsg = () => {
  const [message, setMessage] = useState('');
  const [receivedMessages, setReceivedMessages] = useState([]);
  const [ws, setWs] = useState(null);

  useEffect(() => {
    // Cria uma nova conexão WebSocket
    const websocket = new WebSocket('ws://127.0.0.1:8000/ws');
    setWs(websocket);

    // Lida com mensagens recebidas
    websocket.onmessage = (event) => {
      setReceivedMessages((prev) => [...prev, event.data]);
    };

    // Lida com a conexão WebSocket fechada
    websocket.onclose = () => {
      console.log('WebSocket disconnected');
    };

    // Limpa a conexão WebSocket ao desmontar o componente
    return () => {
      websocket.close();
    };
  }, []);

  const sendMessage = () => {
    if (ws) {
      ws.send(message);
      setMessage('');
    }
  };

  return (
    <div>
      <h1>WebSocket Test</h1>
      <input
        type="text"
        value={message}
        onChange={(e) => setMessage(e.target.value)}
      />
      <button onClick={sendMessage}>Send Message</button>
      <div>
        <h2>Received Messages:</h2>
        <ul>
          {receivedMessages.map((msg, index) => (
            <li key={index}>{msg}</li>
          ))}
        </ul>
      </div>
    </div>
  );
};

export default WebSocketMsg;
