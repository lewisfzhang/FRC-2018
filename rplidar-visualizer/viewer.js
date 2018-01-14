// things in the queue will be drawn then removed
var coordinateQueue = [];
var cache = [];
var cacheIndex = 0;

// create WebSocket connection
var socket = new WebSocket('ws://localhost:8080');

// canvas variables
var canvas;
var ctx;
var status;

// document.onload = function() {
document.addEventListener("DOMContentLoaded", function(event) {
    // socket = new WebSocket('ws://localhost:8080');

    // initialize variables
    canvas = document.getElementById("field");
    ctx = canvas.getContext("2d");
    status = document.getElementById("status");

    // connection opened
    socket.addEventListener('open', function(event) {
        console.log("connected");
        status.innerHTML = 'Connected to the WebSocket';
        socket.send("update");
    });

    // received message from server
    socket.addEventListener('message', function(event) {
        parseData(JSON.parse(event.data));
        updatePoints();
        console.log('Message from server ', event.data);
        document.getElementById("slidecontainer").innerHTML = '<h1 class="timestamp">' + cache[cacheIndex].timestamp + '</h1>' +
            '<input type="range" min="0" max="' + (cache.length - 1) + '" value="' + cacheIndex + '" class="slider">';
    });

    // connection closed
    socket.addEventListener('close', function(event) {
        console.log("closed");
        status.innerHTML = 'Status: Connection Lost';
    });

    // connection error
    socket.addEventListener('error', function(event) {
        console.log("error");
        status.innerHTML = 'Status: Error';
    });

    function updatePoints() {
        cache.push(coordinateQueue);
        drawPoints();
        coordinateQueue = [];
    }

    function drawPoints() {
        for (var i = 0; i < cache[cacheIndex].length; i++) {
            ctx.fillRect(cache[cacheIndex][i].x, cache[cacheIndex][i].y, 5, 5);
        }
    }

    function parseData(data) {
        for (var i = 0; i < data.scan.length; i++) {
            coordinateQueue[i] = data.scan[i];
        }
    }
});