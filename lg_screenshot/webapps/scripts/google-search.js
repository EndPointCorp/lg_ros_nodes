module.exports = {
    customize: customize
};

function customize(screenshooter, log) {
    screenshooter.evaluator = function() {
        var logo = document.querySelector('#logo');
        var img = document.querySelector('#logo > img');
	var logocont = document.querySelector('#logocont');
        if (logo && logo.style.backgroundImage) {
            logo.style.backgroundImage = logo.style.backgroundImage.replace(/(nav_logo[0-9]+)/g,'$1_hr');
            return logo.style.backgroundImage;
        }
        else if (img) {
            img.src = img.src.replace(/(nav_logo[0-9]+)/g,'$1_hr');
            return img.src;
        }
	else if (logocont) {
	    return 'doodle';
	}
        return false;
    };
}
