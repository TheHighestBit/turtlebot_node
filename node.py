#!/usr/bin/env python

# Adapted from https://github.com/ros-teleop/teleop_twist_keyboard

from __future__ import print_function

import threading
import socket
import json

import rospy

from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped

TwistMsg = Twist

msg = """
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

moveBindings = {
        'i':(1,0,0,0),
        'o':(1,0,0,-1),
        'j':(0,0,0,1),
        'l':(0,0,0,-1),
        'u':(1,0,0,1),
        ',':(-1,0,0,0),
        '.':(-1,0,0,1),
        'm':(-1,0,0,-1),
        'O':(1,-1,0,0),
        'I':(1,0,0,0),
        'J':(0,1,0,0),
        'L':(0,-1,0,0),
        'U':(1,1,0,0),
        '<':(-1,0,0,0),
        '>':(-1,-1,0,0),
        'M':(-1,1,0,0),
        't':(0,0,1,0),
        'b':(0,0,-1,0),
    }

speedBindings={
        'q':(1.1,1.1),
        'z':(.9,.9),
        'w':(1.1,1),
        'x':(.9,1),
        'e':(1,1.1),
        'c':(1,.9),
    }

class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        self.publisher = rospy.Publisher('cmd_vel', TwistMsg, queue_size = 1)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.speed = 0.0
        self.turn = 0.0
        self.condition = threading.Condition()
        self.done = False

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.publisher.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def update(self, x, y, z, th, speed, turn):
        self.condition.acquire()
        self.x = x
        self.y = y
        self.z = z
        self.th = th
        self.speed = speed
        self.turn = turn
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(0, 0, 0, 0, 0, 0)
        self.join()

    def run(self):
        twist_msg = TwistMsg()

        if stamped:
            twist = twist_msg.twist
            twist_msg.header.stamp = rospy.Time.now()
            twist_msg.header.frame_id = twist_frame
        else:
            twist = twist_msg
        while not self.done:
            if stamped:
                twist_msg.header.stamp = rospy.Time.now()
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)

            # Copy state into twist message.
            twist.linear.x = self.x * self.speed
            twist.linear.y = self.y * self.speed
            twist.linear.z = self.z * self.speed
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = self.th * self.turn

            self.condition.release()

            # Publish.
            self.publisher.publish(twist_msg)

        # Publish stop message when thread exits.
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.publisher.publish(twist_msg)

class SocketServer(threading.Thread):
    def __init__(self, host='0.0.0.0', port=9090, buffer_size=1024):
        threading.Thread.__init__(self)
        self.host = host
        self.port = port
        self.buffer_size = buffer_size
        self.sock = None
        self.client_connected = False
        self.running = True
        self.last_command = ''
        self.daemon = True  # Thread will exit when main program exits

    def setup_server(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.host, self.port))
        self.sock.listen(1)
        print(f"Socket server started on {self.host}:{self.port}")

    def run(self):
        self.setup_server()
        
        while self.running:
            print("Waiting for connection...")
            try:
                client, addr = self.sock.accept()
                self.client_connected = True
                print(f"Connected to {addr}")
                
                while self.client_connected and self.running:
                    try:
                        data = client.recv(self.buffer_size)
                        if not data:
                            print("Client disconnected")
                            self.client_connected = False
                            break
                            
                        # Process received data
                        command = data.decode('utf-8').strip()
                        self.last_command = command
                        print(f"Received command: {command}")
                        
                    except Exception as e:
                        print(f"Error receiving data: {e}")
                        self.client_connected = False
                        break
                
                client.close()
                
            except Exception as e:
                print(f"Connection error: {e}")
                if self.sock:
                    self.sock.close()
                break
                
    def stop(self):
        self.running = False
        if self.sock:
            self.sock.close()
            
    def get_last_command(self):
        cmd = self.last_command
        self.last_command = ''
        return cmd

def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed, turn)

if __name__=="__main__":
    rospy.init_node('teleop_twist_socket')

    speed = rospy.get_param("~speed", 0.5)
    turn = rospy.get_param("~turn", 1.0)
    speed_limit = rospy.get_param("~speed_limit", 1000)
    turn_limit = rospy.get_param("~turn_limit", 1000)
    repeat = rospy.get_param("~repeat_rate", 0.0)
    stamped = rospy.get_param("~stamped", False)
    twist_frame = rospy.get_param("~frame_id", '')
    socket_port = rospy.get_param("~socket_port", 9090)
    
    if stamped:
        TwistMsg = TwistStamped

    # Start the publisher thread
    pub_thread = PublishThread(repeat)
    
    # Start the socket server
    socket_server = SocketServer(port=socket_port)
    socket_server.start()

    x = 0
    y = 0
    z = 0
    th = 0
    status = 0

    try:
        pub_thread.wait_for_subscribers()
        pub_thread.update(x, y, z, th, speed, turn)

        print("Socket server running. Awaiting commands...")
        print(vels(speed, turn))

        print(msg)
        
        rate = rospy.Rate(10)  # 10Hz refresh rate
        
        while not rospy.is_shutdown():
            command = socket_server.get_last_command()
            
            if command:
                key = command[0] if command else ''
                
                if key in moveBindings.keys():
                    x = moveBindings[key][0]
                    y = moveBindings[key][1]
                    z = moveBindings[key][2]
                    th = moveBindings[key][3]
                    print(f"Movement command: {key}")
                elif key in speedBindings.keys():
                    speed = min(speed_limit, speed * speedBindings[key][0])
                    turn = min(turn_limit, turn * speedBindings[key][1])
                    if speed == speed_limit:
                        print("Linear speed limit reached!")
                    if turn == turn_limit:
                        print("Angular speed limit reached!")
                    print(vels(speed, turn)) 
                elif key == 's':
                    x = 0
                    y = 0
                    z = 0
                    th = 0
                    print("Stopping")
                elif key == 'quit' or key == '\x03':
                    break
                    
                pub_thread.update(x, y, z, th, speed, turn)
                
            rate.sleep()

    except Exception as e:
        print(e)

    finally:
        # Stop the threads
        socket_server.stop()
        pub_thread.stop()