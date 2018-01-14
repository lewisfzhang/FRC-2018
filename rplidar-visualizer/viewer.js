// things in the queue will be drawn then removed
var coordinateQueue = [];

// create WebSocket connection
const socket = new WebSocket('ws://localhost:8080');

// canvas variables
var canvas;
var context;
var status;
var width;
var height;

// sliders
var zoomSlider;

document.addEventListener('DOMContentLoaded', function(event) {
    // initialize canvas variables
    canvas = document.getElementById('field');
    context = canvas.getContext('2d');
    status = document.getElementById('status');
    width = parseInt(canvas.width);
    height = parseInt(canvas.height);

    // initiate slider variables
    zoomSlider = document.getElementById('zoom-slider');

    // connection opened
    socket.addEventListener('open', function(event) {
        console.log('connected');
        status.innerHTML = 'Connected to the WebSocket';
        socket.send('update');
    });

    var lastData;

    // received message from server
    socket.addEventListener('message', function(event) {
        parseData(JSON.parse(event.data));
        console.log('Message from server ', event.data);
        updatePoints();

        lastData = JSON.parse(event.data);
    });

    // connection closed
    socket.addEventListener('close', function(event) {
        console.log('closed');
        status.innerHTML = 'Status: Connection Lost';
    });

    // connection error
    socket.addEventListener('error', function(event) {
        console.log('error');
        status.innerHTML = 'Status: Error';
    });

    function updatePoints() {
        clearPoints();
        drawPoints();
        coordinateQueue = [];
    }

    function clearPoints() {
        context.clearRect(0, 0, width, height);
    }

    function drawPoints() {
        for (var i = 0; i < coordinateQueue.length; i++) {
            context.fillRect(coordinateQueue[i].x - 3, coordinateQueue[i].y - 3, 6, 6);
        }
    }

    function parseData(data) {
        for (var i = 0; i < data.scan.length; i++) {
            coordinateQueue[i] = Object.assign({}, data.scan[i]);
        }
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

    // do stuff when zoom slider is changed
    zoomSlider.addEventListener('input', function (evt) {
        zoom = parseInt(this.value);

        backgroundSize = 'background-size: ' + (100 * getZoom()) + '%;';
        backgroundPositionX *= zoom;
        backgroundPositionY *= zoom;
        updateCanvasStyle();

        parseData(lastData);
        console.log(coordinateQueue);
        for (var i = 0; i < coordinateQueue.length; i++) {
            coordinateQueue[i].x *= getZoom();
            coordinateQueue[i].y *= getZoom();
        }
        updatePoints();
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

            parseData(lastData);
            for (var i = 0; i < coordinateQueue.length; i++) {
                coordinateQueue[i].x *= getZoom();
                coordinateQueue[i].x += dragX;
                coordinateQueue[i].y *= getZoom();
                coordinateQueue[i].y += dragY;
            }
            updatePoints();
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