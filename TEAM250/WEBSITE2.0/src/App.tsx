import { useState } from 'react'
import reactLogo from './assets/react.svg'
import viteLogo from '/vite.svg'
import './App.css'
import { Header } from './components/Header/Header.tsx'
import { Route, Routes } from 'react-router-dom'
import { Home } from './pages/Home.tsx'
import { About } from './pages/About.tsx'

function App() {
  
  return(
    <Routes>
      <Route path='/' element={<Home />}></Route>
      <Route path='/pages/About' element={<About/>}></Route>
    </Routes>
  )

}

export default App
