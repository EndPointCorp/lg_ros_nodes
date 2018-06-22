// Load some scripts in the right order

// <script src="http://maps.googleapis.com/maps/api/js?signed_in=true"></script>
// <script src="js/main.js"></script>

(function() {
    var map_key = getParameterByName('map_api_key', String, '');
    var src = 'http://maps.googleapis.com/maps/api/js?key=' + map_key;

    load(src, function() {
        load('js/main.js');
    });

    function load(src, onload) {
        var script = document.createElement('script');
        if (onload) {
            script.onload = onload;
        }
        script.src = src;
        document.head.appendChild(script);
    }

})();