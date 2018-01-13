document.addEventListener("DOMContentLoaded", function(event) {
    // robot position
    var robotX, robotY;

    // things in the queue will be drawn then removed
    var coordinateQueue = [];

    // create WebSocket connection
    const socket = new WebSocket('ws://localhost:8080');

    // canvas variables
    var canvas = document.getElementById("field");
    var ctx = canvas.getContext("2d");
    var status = document.getElementById("status");
    var width = canvas.width;
    var height = canvas.height;

    // connection opened
    socket.addEventListener('open', function (event) {
        status.innerHTML = "Status: Connected to WebSocket";
        socket.send("update");
    });

    // received messed from server
    socket.addEventListener('message', function (event) {
        console.log('Message from server ', event.data);
        parseData(JSON.parse(event.data));
        updatePoints();
    });

    // connection closed
    socket.addEventListener('close', function(event) {
        status.innerHTML = "Status: Connection Lost";
    });

    // connection error
    socket.addEventListener('error', function(event) {
        status.innerHTML = "Status: Error";
    });

    function updatePoints() {
        console.log("Updating Points");
        console.log(coordinateQueue);
        // draw Points
        for (var i = 0; i < coordinateQueue.length; i++) {
            ctx.fillRect(coordinateQueue[i].x, coordinateQueue[i].y, 5, 5);
        }

        coordinateQueue = [];
        console.log(coordinateQueue.length);
    }

    class Point {
        constructor(x, y) {
            this.x = x;
            this.y = y;
        }
    }

    function parseData(data) {
        for (var i = 0; i < data.scan.length; i++) {
            coordinateQueue[i] = data.scan[i];
        }
    }
});