var cache = [];

// create WebSocket connection
const socket = new WebSocket('ws://localhost:8080');

// canvas variables
var canvas;
var context;
var width;
var height;

// sliders
var zoomSlider;
var cacheSlider;

// field and point size
// var actualWidth = 24993.6; // 82 ft --> mm
var actualWidth = 22511.764; // directly scaled from image size
var actualHeight = 9144; // 30 ft --> mm
var pointSize = 6; // px

document.addEventListener('DOMContentLoaded', function(event) {
    // initialize canvas variables
    canvas = document.getElementById('field');
    context = canvas.getContext('2d');
    width = parseInt(canvas.width);
    height = parseInt(canvas.height);

    // initiate slider variables
    zoomSlider  = document.getElementById('zoom-slider');
    cacheSlider = document.getElementById('cache-slider');

    // connection opened
    socket.addEventListener('open', function(event) {
        console.log('connected');
        socket.send('update');
    });

    function parseTimestamp(timestamp) {
        var date = new Date(timestamp);

        var hours = date.getHours();
        dateString = (date.getMonth() + 1) + "/" 
            + date.getDate() + "/" 
            + date.getFullYear() + ", " 
            + (hours % 12) + ":" 
            + ("00" + date.getMinutes()).slice(-2) + ":" 
            + ("00" + (date.getSeconds() + (date.getMilliseconds() / 1000).toFixed(3).slice(1))).slice(-6) + " "
            + ((hours > 12) ? "PM" : "AM");

        return dateString;
    }

    var lastData;
    var count = 0;

    // received message from server
    socket.addEventListener('message', function(event) {
        var data = JSON.parse(event.data);
        cache.push(data);

        document.getElementById("title").innerHTML = 'Time: ' + parseTimestamp(data.timestamp);
        cacheSlider.max = cache.length - 1;
        cacheSlider.value = cache.length - 1;
        cacheSlider.style = "display: block";

        console.log('message ' + ++count + ' received');
        plotData(data);

        lastData = data;
    });

    // connection closed
    socket.addEventListener('close', function(event) {
        console.log('connection lost');
    });

    // connection error
    socket.addEventListener('error', function(event) {
        document.getElementById("title").innerHTML = "Status: Error";
        console.log('error');
    });

    function updatePoints(data) {
        clearPoints();
        drawPoints(data);
    }

    function clearPoints() {
        context.clearRect(0, 0, width, height);
    }

    function drawPoints(data) {
        for (var i = 0; i < data.length; i++) {
	    var actualX = (data[i].x / actualWidth) * width;
	    var actualY = (data[i].y / actualHeight) * height;
            context.fillRect(actualX - (pointSize / 2), actualY - (pointSize / 2), pointSize, pointSize);
        }
    }

    function parseData(data) {
        var points = [];

        for (var i = 0; i < data.scan.length; i++) {
            points[i] = {
                'x': data.scan[i].x,
                'y': data.scan[i].y
            }
        }
        
        console.log(points);
        return points;
    }

    var zoom = 0;

    function getZoom() {
        return (1 + (0.005 * zoom));
    }

    var backgroundSize = 'background-size: 100%; ';
    var backgroundPositionX = 0;
    var backgroundPositionY = 0;

    function updateCanvasStyle() {
        canvas.style = backgroundSize + 'background-position: ' + backgroundPositionX + 'px ' + backgroundPositionY + 'px';
    }

    function plotData(data) {
        var points = parseData(data);

        for (var i = 0; i < points.length; i++) {
            points[i].x *= getZoom();
            points[i].x += (backgroundPositionX / width) * actualWidth;
            points[i].y *= getZoom();
            points[i].y += (backgroundPositionY / width) * actualWidth;
        }

        updatePoints(points);
    }

    var previousZoom = zoom;

    // do stuff when zoom slider is changed
    zoomSlider.addEventListener('input', function (evt) {
        previousZoom = zoom;
        zoom = parseInt(this.value);

        backgroundSize = 'background-size: ' + (100 * getZoom()) + '%;';

        var multiplier = zoom / previousZoom;
        multiplier = zoom < previousZoom ? multiplier : 1 / multiplier;
        backgroundPositionX *= multiplier;
        backgroundPositionY *= multiplier;

        // make sure backgroundPositionX and backgroundPositionY don't go above the limit
        backgroundPositionX = backgroundPositionX > getMaxMove().x ? getMaxMove().x : backgroundPositionX;
        backgroundPositionY = backgroundPositionY > getMaxMove().y ? getMaxMove().y : backgroundPositionY;

        updateCanvasStyle();

        plotData(lastData);

        document.getElementById('zoom').innerHTML = "Zoom: " + Math.round(100 * getZoom()) + "%";
    });

    cacheSlider.addEventListener('input', function (evt) {
        var data = cache[parseInt(this.value)];
        document.getElementById('title').innerHTML = 'Time: ' + parseTimestamp(data.timestamp);
        plotData(data)
        lastData = data;
    });

    function getMaxMove() {
        var maxWidth = (width * (getZoom() - 1));
        var maxHeight = (height * (getZoom() - 1));
        return {'x': maxWidth, 'y': maxHeight};
    }

    var clickX;
    var clickY;
    var mouseX;
    var mouseY;
    var dragInterval;

    canvas.onmousedown = function(event) {
        clickX = event.x - backgroundPositionX;
        clickY = event.y - backgroundPositionY;

        dragInterval = setInterval(function() {
            var dragX = mouseX - clickX;
            var dragY = mouseY - clickY;
            var maxX = -getMaxMove().x;
            var maxY = -getMaxMove().y;

            // make sure dragX and dragY don't go above 0 or go below the limit
            dragX = dragX < maxX ? maxX : dragX;
            dragX = dragX > 0 ? 0 : dragX;
            dragY = dragY < maxY ? maxY : dragY;
            dragY = dragY > 0 ? 0 : dragY;

            backgroundPositionX = dragX;
            backgroundPositionY = dragY;
            updateCanvasStyle();

            plotData(lastData)
        }, 10);
    };

    canvas.onmousemove = function(event) {
        mouseX = event.x;
        mouseY = event.y;
    }

    document.onmouseup = function(event) {
        clearInterval(dragInterval);
    };
});