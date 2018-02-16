import sys
import signal
import json

sys.path.append("assets/python/monotonic-1.3-py2.7.egg")
sys.path.append("assets/python/pynetworktables-2017.0.8-py2.7.egg")
sys.path.append("assets/python/SimpleWebSocketServer-0.1.0-py2.7.egg")

from networktables import NetworkTable
from SimpleWebSocketServer import WebSocket, SimpleWebSocketServer
 
connections = set()
class DashboardWebSocket(WebSocket):
    def handleMessage(self):
        try:
            jsonPayload = json.loads(self.data)
            if jsonPayload["type"] == "string":
                table.putString(jsonPayload["key"], jsonPayload["value"])
            elif jsonPayload["type"] == "boolean":
                table.putBoolean(jsonPayload["key"], jsonPayload["value"])
            elif jsonPayload["type"] == "number":
                table.putNumber(jsonPayload["key"], jsonPayload["value"])
        except:
            traceback.print_exc()

    def handleConnected(self):
        print self.address, 'connected'
        connections.add(self)
              
    def handleClose(self):
        print self.address, 'closed'
        connections.remove(self)

if len(sys.argv) != 2:
    print("Error: specify a Network Table IP to connect to!")
    exit(-1)

ip = sys.argv[1]

NetworkTable.setClientMode()
NetworkTable.setIPAddress(ip)
NetworkTable.initialize()

table = NetworkTable.getTable("SmartDashboard")
server = SimpleWebSocketServer('', 8000, DashboardWebSocket)

def valueChanged(table, key, value, isNew):
    print("valueChanged: key: '%s'; value: %s; isNew: %s" % (key, value, isNew))
    jsonObj = {
        'table': str(table),
        'key': str(key),
        'value': str(value)
    }
    jsonString = u"" + json.dumps(jsonObj)
    print(jsonString)
    for connection in connections:
        connection.sendMessage(jsonString)

def close_sig_handler(signal, frame):
    server.close()
    sys.exit()

table.addTableListener(valueChanged)
signal.signal(signal.SIGINT, close_sig_handler)
server.serveforever()