#!/usr/bin/env python

# Adapted from https://github.com/ros-teleop/teleop_twist_keyboard

from __future__ import print_function

import threading
import socket
# import json # json was imported but not used
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped


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

CTRL-C to quit (in terminal) or send 'quit' command via socket
"""

moveBindings = {
        'i':(1.0,0.0,0.0,0.0),
        'o':(1.0,0.0,0.0,-1.0),
        'j':(0.0,0.0,0.0,1.0),
        'l':(0.0,0.0,0.0,-1.0),
        'u':(1.0,0.0,0.0,1.0),
        ',':(-1.0,0.0,0.0,0.0),
        '.':(-1.0,0.0,0.0,1.0),
        'm':(-1.0,0.0,0.0,-1.0),
        'O':(1.0,-1.0,0.0,0.0),
        'I':(1.0,0.0,0.0,0.0),
        'J':(0.0,1.0,0.0,0.0),
        'L':(0.0,-1.0,0.0,0.0),
        'U':(1.0,1.0,0.0,0.0),
        '<':(-1.0,0.0,0.0,0.0), # Note: '<' might be hard to send, consider changing
        '>':(-1.0,-1.0,0.0,0.0), # Note: '>' might be hard to send, consider changing
        'M':(-1.0,1.0,0.0,0.0),
        't':(0.0,0.0,1.0,0.0),
        'b':(0.0,0.0,-1.0,0.0),
    }

speedBindings={
        'q':(1.1,1.1),
        'z':(.9,.9),
        'w':(1.1,1.0),
        'x':(.9,1.0),
        'e':(1.0,1.1),
        'c':(1.0,.9),
    }

class PublishThread(threading.Thread):
    def __init__(self, node: Node, rate: float, use_stamped: bool, frame_id: str):
        super(PublishThread, self).__init__()
        self.node = node
        self.use_stamped = use_stamped
        self.frame_id = frame_id

        if self.use_stamped:
            self.twist_msg_type = TwistStamped
        else:
            self.twist_msg_type = Twist

        self.publisher = self.node.create_publisher(self.twist_msg_type, 'cmd_vel', 10)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.speed = 0.0
        self.turn = 0.0
        self.condition = threading.Condition()
        self.done = False

        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None # Wait indefinitely for new message

        self.start()

    def wait_for_subscribers(self):
        i = 0
        while rclpy.ok() and self.publisher.get_subscription_count() == 0:
            if i == 4:
                self.node.get_logger().info(
                    "Waiting for subscriber to connect to {}".format(self.publisher.topic_name))
            time.sleep(0.5)
            i = (i + 1) % 5
        if not rclpy.ok():
            self.node.get_logger().error("Shutdown request received before subscribers connected")
            raise Exception("Got shutdown request before subscribers connected")

    def update(self, x: float, y: float, z: float, th: float, speed: float, turn: float):
        with self.condition:
            self.x = x
            self.y = y
            self.z = z
            self.th = th
            self.speed = speed
            self.turn = turn
            self.condition.notify()

    def stop(self):
        self.done = True
        self.update(0.0, 0.0, 0.0, 0.0, 0.0, 0.0) # Send a final zero command
        self.join(timeout=2.0) # Wait for thread to finish

    def run(self):
        twist_msg_instance = self.twist_msg_type()

        if self.use_stamped:
            if not isinstance(twist_msg_instance, TwistStamped):
                 self.node.get_logger().fatal("Logic error: use_stamped is True but message is not TwistStamped!")
                 return
            twist_content = twist_msg_instance.twist
            twist_msg_instance.header.frame_id = self.frame_id
        else:
            if not isinstance(twist_msg_instance, Twist):
                 self.node.get_logger().fatal("Logic error: use_stamped is False but message is not Twist!")
                 return
            twist_content = twist_msg_instance

        while not self.done and rclpy.ok():
            if self.use_stamped:
                twist_msg_instance.header.stamp = self.node.get_clock().now().to_msg()
            
            with self.condition:
                if self.timeout is not None:
                    self.condition.wait(self.timeout)
                else:
                    self.condition.wait() # Wait indefinitely if no timeout (rate is 0)
                
                if self.done: # Check done flag again after wait
                    break

                twist_content.linear.x = self.x * self.speed
                twist_content.linear.y = self.y * self.speed
                twist_content.linear.z = self.z * self.speed
                twist_content.angular.x = 0.0
                twist_content.angular.y = 0.0
                twist_content.angular.z = self.th * self.turn

            self.publisher.publish(twist_msg_instance)

        # Publish stop message when thread exits
        twist_content.linear.x = 0.0
        twist_content.linear.y = 0.0
        twist_content.linear.z = 0.0
        twist_content.angular.x = 0.0
        twist_content.angular.y = 0.0
        twist_content.angular.z = 0.0
        if rclpy.ok(): # Only publish if context is still valid
            self.publisher.publish(twist_msg_instance)
        self.node.get_logger().info("PublishThread finished.")


class SocketServer(threading.Thread):
    def __init__(self, host='0.0.0.0', port=9090, buffer_size=1024, node_logger=None):
        threading.Thread.__init__(self)
        self.host = host
        self.port = port
        self.buffer_size = buffer_size
        self.sock = None
        self.client_connected = False
        self.running = True
        self.last_command = ''
        # self.daemon = True # Explicit join is preferred
        self.logger = node_logger if node_logger else print # Use node logger or print

    def setup_server(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.settimeout(1.0) # Timeout for accept()
        self.sock.bind((self.host, self.port))
        self.sock.listen(1)
        self.logger(f"Socket server started on {self.host}:{self.port}")

    def run(self):
        try:
            self.setup_server()
        except Exception as e:
            self.logger(f"Failed to setup socket server: {e}")
            self.running = False
            return

        while self.running:
            try:
                client, addr = self.sock.accept()
                self.logger(f"Connected to {addr}")
                client.settimeout(1.0)
                self.client_connected = True

                while self.client_connected and self.running:
                    data = client.recv(self.buffer_size)
                    if not data:
                        self.client_connected = False
                        break
                    cmd = data.decode('utf-8').strip()
                    if cmd:
                        self.last_command = cmd

            except socket.timeout:
                continue
            except Exception as e:
                if self.running:
                    self.logger(f"Socket server error: {e}")
                self.running = False

        if self.sock:
            self.sock.close()
        self.logger("SocketServer thread finished.")
            
    def stop(self):
        self.running = False
        # The run loop will exit due to self.running and socket timeouts.
        self.join(timeout=2.0) # Wait for the thread to finish
            
    def get_last_command(self):
        # This is a basic way; for more complex scenarios, a thread-safe queue might be better.
        cmd = self.last_command
        self.last_command = '' # Clear after reading
        return cmd

def vels(speed: float, turn: float) -> str:
    return f"currently:\tspeed {speed:.2f}\tturn {turn:.2f}"

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('teleop_twist_socket_node') # Changed node name slightly for clarity

    # Declare and get parameters
    node.declare_parameter('speed', 0.5)
    node.declare_parameter('turn', 1.0)
    node.declare_parameter('speed_limit', 1.0) # Max linear speed
    node.declare_parameter('turn_limit', 1.0)  # Max angular speed
    node.declare_parameter('repeat_rate', 10.0) # PublishThread internal publish rate if no new command
    node.declare_parameter('stamped', False)
    node.declare_parameter('frame_id', 'base_link') # Default frame_id if stamped
    node.declare_parameter('socket_port', 9090)

    speed_param = node.get_parameter('speed').get_parameter_value().double_value
    turn_param = node.get_parameter('turn').get_parameter_value().double_value
    speed_limit = node.get_parameter('speed_limit').get_parameter_value().double_value
    turn_limit = node.get_parameter('turn_limit').get_parameter_value().double_value
    repeat = node.get_parameter('repeat_rate').get_parameter_value().double_value
    use_stamped = node.get_parameter('stamped').get_parameter_value().bool_value
    twist_frame = node.get_parameter('frame_id').get_parameter_value().string_value
    socket_port = node.get_parameter('socket_port').get_parameter_value().integer_value

    pub_thread = None
    socket_server = None

    try:
        pub_thread = PublishThread(node, repeat, True, twist_frame)
        socket_server = SocketServer(port=socket_port, node_logger=node.get_logger().info)
        socket_server.start()

        x, y, z, th = 0.0, 0.0, 0.0, 0.0
        current_speed = speed_param
        current_turn = turn_param

        pub_thread.wait_for_subscribers()
        pub_thread.update(x, y, z, th, current_speed, current_turn)

        node.get_logger().info("Teleop node started. Listening for socket commands...")
        node.get_logger().info(vels(current_speed, current_turn))
        print(msg) # Show controls to the user
        
        # Main loop rate for processing socket commands
        loop_rate = node.create_rate(20) # Process commands at 20 Hz 
        
        while rclpy.ok():
            command = socket_server.get_last_command()

            logger = node.get_logger()
            logger.info(f"Socket command: {command}") # For debugging, can be removed later
            
            if command:
                key = command[0] if command else ''
                
                if key in moveBindings:
                    x = moveBindings[key][0]
                    y = moveBindings[key][1]
                    z = moveBindings[key][2]
                    th = moveBindings[key][3]
                    # node.get_logger().debug(f"Movement: {key}")
                elif key in speedBindings:
                    current_speed = min(speed_limit, current_speed * speedBindings[key][0])
                    current_turn = min(turn_limit, current_turn * speedBindings[key][1])
                    if current_speed == speed_limit:
                        node.get_logger().info("Linear speed limit reached!")
                    if current_turn == turn_limit:
                        node.get_logger().info("Angular speed limit reached!")
                    node.get_logger().info(vels(current_speed, current_turn))
                elif key == 's': # Custom command to stop
                    x, y, z, th = 0.0, 0.0, 0.0, 0.0
                    node.get_logger().info("Stopping robot.")
                elif command == 'quit': # Check for 'quit' command
                    node.get_logger().info("Quit command received via socket.")
                    break 
                # else: # Optionally, stop on unknown key
                    # x, y, z, th = 0.0, 0.0, 0.0, 0.0
                    # node.get_logger().info(f"Unknown key '{key}', stopping.")
                    
                pub_thread.update(x, y, z, th, current_speed, current_turn)
            
            loop_rate.sleep()

    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt, shutting down.')
    except Exception as e:
        node.get_logger().error(f"An unhandled error occurred in main: {e}")
        import traceback
        node.get_logger().error(traceback.format_exc())
    finally:
        node.get_logger().info('Initiating shutdown of threads and node...')
        if socket_server and socket_server.is_alive():
            socket_server.stop()
            node.get_logger().info('SocketServer stopped.')
        if pub_thread and pub_thread.is_alive():
            pub_thread.stop()
            node.get_logger().info('PublishThread stopped.')
        
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
        node.get_logger().info("ROS 2 node shutdown complete.")
        print("Application terminated.")

if __name__=="__main__":
    main()