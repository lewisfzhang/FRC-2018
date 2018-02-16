# Cheesy Dash 3.0
Cheesy Dash, a Python webapp, is used as a Dashboard for our drive team at competitions. The app allows a user to select autonomous paths and view essential robot data.

## Getting Started
These instructions will get you a copy of the project up and running on your local machine for development, testing, and use.

### Dependencies
- monotonic 1.3
- pynetworktables 2017.0.8
- SimpleWebSocketServer 0.1.0

### Installing
- Clone this Repository (`git clone https://github.com/Team254/FRC-2018.git`)
- If you don't have Python, [download](https://www.python.org/downloads/) at least Python 2.7 or newer
- Run App (See [Running](https://github.com/Team254/FRC-2018#running))

### Running
- To run with the robot, start the server (`python dash.py 10.2.54.2`) and open `index.html` in a browser
- To run locally (without a robot), start the server (`python dash.py localhost`), open `index.html` in a browser, and start the fake robot server (`python test_server.py`)