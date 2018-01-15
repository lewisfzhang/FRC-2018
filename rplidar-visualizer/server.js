const WebSocketServer = require('websocket').server;
const http = require('http');
const port = 8080;

// const ntClient = require('wpilib-nt-client');
// const client = new ntClient.Client();

// client.start((isConnected, err) => {
//     console.log({ isConnected, err });
// }, 'roborio-254.local');
//
// client.addListener((key, val, type, id) => {
//     console.log({ key, val, type, id });
// })

var server = http.createServer(function(request, response) {});
server.listen(port, function() {});
console.log('connected on port ' + port);

const socket = new WebSocketServer({
    httpServer: server
});

var points = 100;

function randomizePoints() {
    var json = {'scan':[], 'timestamp': 0};

    // randomize numbers (for testing purposes)
    for (var i = 0; i < points; i++) {
        json.scan[i] = {
            'x': Math.round(Math.random() * 500),
            'y': Math.round(Math.random() * 500)
        };
    }

    json.timestamp = (new Date()).getTime();

    return json;
}

var count = 0;

socket.on('request', function(request) {
    var connection = request.accept(null, request.origin);

    console.log('connected to ' + request.origin);

    connection.on('message', function(event) {
        console.log(event.utf8Data);

        var interval = setInterval(function() {
            connection.send(JSON.stringify(randomizePoints()));
            console.log('message ' + ++count + ' sent');
        }, 100);

        setTimeout(function() {
            clearInterval(interval);
        }, 10000); 
    });

    connection.on('close', function() {
        console.log('connection closed');
        connection = null;
    });
});

