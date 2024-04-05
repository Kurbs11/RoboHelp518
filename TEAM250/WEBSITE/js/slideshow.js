let slideIndex = 1;
showSlides(slideIndex);

function plusSlides(n) {
    slideIndex += n
    showSlides(slideIndex);
}

function currentSlide(n) {
    slideIndex = n
    showSlides(slideIndex);
}

function showSlides(n) {
    let i;
    let slides = document.getElementsByClassName("HVRCSlides");
    let dots = document.getElementsByClassName("dot");
    if (n > slides.length) {slideIndex = 1}    
    if (n < 1) {slideIndex = slides.length}
    for (i = 0; i < slides.length; i++) {
    slides[i].style.display = "none";  
    }
    for (i = 0; i < dots.length; i++) {
    dots[i].className = dots[i].className.replace(" active", "");
    }
    console.log(slides[slideIndex-1]);
    slides[slideIndex-1].style.display = "block";  

    dots[slideIndex-1].className += " active";
}
