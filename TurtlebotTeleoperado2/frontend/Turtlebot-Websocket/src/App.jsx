import { useState } from 'react'
import reactLogo from './assets/react.svg'
import viteLogo from '/vite.svg'
import './App.css'

import WebSocketMsg from '../pages/WebSocketMsg/WebSocketMsg';
import TurtleControl from '../pages/TurtleControl/TurtleControl';

function App() {
  const [count, setCount] = useState(0)

  return (
    <>
     <TurtleControl />
    </>
  )
}

export default App
