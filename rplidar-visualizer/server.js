const WebSocketServer = require('websocket').server;
const http = require('http');
const port = 8080;
const ntClient = require('wpilib-nt-client');
const client = new ntClient.Client();

client.start((isConnected, err) => {
    console.log({ isConnected, err });
}, 'roborio-254.local');

client.addListener((key, val, type, id) => {
    console.log({ key, val, type, id });
})


var server = http.createServer(function(request, response) {});
server.listen(port, function() {});
console.log("connected on port " + port);

socket = new WebSocketServer({
    httpServer: server
});

var json = {"scan":
        [
            {"x":0, "y":0},
            {"x":0, "y":0},
            {"x":0, "y":0},
            {"x":0, "y":0},
            {"x":0, "y":0}
        ]
};

// randomize numbers (for testing purposes)
for (var i in json.scan) {
    point = json.scan[i];
    point.x = Math.round(Math.random() * 50);
    point.y = Math.round(Math.random() * 50);
}

socket.on('request', function(request) {
    var connection = request.accept(null, request.origin);

    connection.on('message', function incoming(data) {
        console.log("Received Message: %s", data);
        connection.send(JSON.stringify(json));
    });

    connection.on('close', function(connection) {
        console.log("connection closed");
    });
});