var http = require("http");

const ntClient = require('wpilib-nt-client');

const client = new ntClient.Client();

client.start((isConnected, err) => {
    console.log({ isConnected, err });
}, 'roborio-3571.local');

http.createServer(function(request, response){
    response.writeHead(200, {"Content-Type":"text/plain"});
    response.write('kys');
    response.end();
}).listen(1337);