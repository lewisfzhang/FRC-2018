import signal
import sys
import time
import json
from networktables import NetworkTables
from SimpleWebSocketServer import SimpleWebSocketServer, WebSocket

# To see messages from networktables, you must setup logging
import logging
logging.basicConfig(level=logging.DEBUG)

if len(sys.argv) != 2:
    print("Error: specify an IP to connect to!")
    exit(0)

ip = sys.argv[1]

NetworkTables.initialize(server = ip)

def connectionListener(connected, info):
    print(info, '; Connected=%s' % connected)

NetworkTables.addConnectionListener(connectionListener, immediateNotify=True)

activeBridges = set()
class DashboardWebSocket(WebSocket):
    def handleConnected(self):
        print("connected", self.address, self.request.path)
        activeBridges.add(self)

    def handleMessage(self):
        print("got message:", self.data)

    def handleClose(self):
        print("closed")

    def sendBridgeValue(self, tableName, key, value):
        jsonPayload = {}
        jsonPayload["table"] = tableName
        jsonPayload["key"] = key
        jsonPayload["value"] = value
        jsonPayload["timestamp"] = round(1000 * time.time())
        jsonString = u"" + json.dumps(jsonPayload)
        self.sendMessage(jsonString)

def valueChanged(table, key, value, isNew):
    print("valueChanged: key: '%s'; value: %s; isNew: %s" % (key, value, isNew))
    for bridge in activeBridges:
        bridge.sendBridgeValue(table.path, key, value)

sd = NetworkTables.getTable("SmartDashboard")
sd.addEntryListener(valueChanged)

server = SimpleWebSocketServer("", 8080, DashboardWebSocket)

def close_sig_handler(signal, frame):
    server.close()
    sys.exit()

signal.signal(signal.SIGINT, close_sig_handler)
server.serveforever()