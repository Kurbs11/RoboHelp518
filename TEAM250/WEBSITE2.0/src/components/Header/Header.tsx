import { Link } from "react-router-dom";


export const Header = () => {

    return(
        <>
            <header>
                <div className="justify-around">
                    <ul className="list-none flex justify-center overflow-hidden bg-[rgb(106,106,250)] max-h-[100px] m-0 p-0">
                        <li className="float-left">
                            <Link to = {'/'}>
                                <div className="flex justify-around items-center">
                                    <img src="./src/images/logo.jpg" alt="Team 250 Robotics Homepage"/>
                                </div>
                            </Link>
                        </li>
                        <div className="flex items-center">
                            <li><a href="home.html">Home</a></li>
                            <li><a href="about.html">About Us</a></li>
                            <li><a href="competition.html">Competitions</a></li>
                            <li><a href="alumni.html">Alumni</a></li>
                            <li><a href="links.html">Links</a></li>
                        </div>
                    </ul>
                </div>
            </header>
        </>
    )
};