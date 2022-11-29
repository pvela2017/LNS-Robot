"""
Sockets ip's and port configuration file

Socket 1 : Driving motors
Socket 2 : Steering motors
Socket 3 : Steering angle feedback

"""
socket1 = {
  "HOST": '192.168.0.7',
  "PORT": 20001,
  "TIMEOUT": 0.1
}

socket2 = {
  "HOST": '192.168.0.7',
  "PORT": 20005,
  "TIMEOUT": 0.1
}

socket3 = {
  "HOST": '192.168.0.8',
  "PORT": 3131,
  "TIMEOUT": 0.001
}