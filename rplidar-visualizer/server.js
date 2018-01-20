const WebSocketServer = require('websocket').server;
const http = require('http');
const port = 8080;

const ntClient = require('wpilib-nt-client');
const client = new ntClient.Client();

client.start((connected, error) => {
    console.log('connected to robot: ' + connected);
}, 'roborio-254-frc.local');

client.addListener((key, value, type, id) => {
    console.log({key, value, type, id});
})

var server = http.createServer(function(request, response) {});
server.listen(port, function() {});
console.log('websocket connected on port ' + port);

const socket = new WebSocketServer({
    httpServer: server
});

var points = 100;

function randomizePoints() {
    var json = {'scan':[], 'timestamp': 0};

    // randomize numbers (for testing purposes)
    for (var i = 0; i < points; i++) {
        json.scan[i] = {
            'x': Math.round(Math.random() * 24993.6),
            'y': Math.round(Math.random() * 9144)
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

