import { Link } from "react-router-dom"
import { Header } from "../components/Header/Header"

export const Home = () => {

    return(
        <>
            <Header/>
            <div> Home </div>

            <Link to = {'/pages/About'}>
                <button className="bg-blue-500 border-spacing-0 rounded-xl text-yellow-300"> About </button>
            </Link>
            

        </>

    )
}