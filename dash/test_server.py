import sys
import time
import logging
import json

sys.path.append("assets/python/monotonic-1.3-py2.7.egg")
sys.path.append("assets/python/pynetworktables-2017.0.8-py2.7.egg")
from networktables import NetworkTable

logging.basicConfig(level=logging.DEBUG)
NetworkTable.initialize()

table = NetworkTable.getTable("SmartDashboard")

def valueChanged(table, key, value, isNew):
    print("Value Changed", table.path, key, value)

table.addTableListener(valueChanged)

i = 0
while True:
    table.putString("Match Cycle", "TELEOP" if i % 5 < 2 else "DISABLED")
    #TODO send more test values
    
    i += 1
    time.sleep(5)