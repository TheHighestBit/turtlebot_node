#!/usr/bin/env python

from __future__ import print_function

import socket
import time
import select # For non-blocking socket checks

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

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
    'i': (1.0, 0.0, 0.0, 0.0), 'o': (1.0, 0.0, 0.0, -1.0), 'j': (0.0, 0.0, 0.0, 1.0),
    'l': (0.0, 0.0, 0.0, -1.0), 'u': (1.0, 0.0, 0.0, 1.0), ',': (-1.0, 0.0, 0.0, 0.0),
    '.': (-1.0, 0.0, 0.0, 1.0), 'm': (-1.0, 0.0, 0.0, -1.0), 'O': (1.0, -1.0, 0.0, 0.0),
    'I': (1.0, 0.0, 0.0, 0.0), 'J': (0.0, 1.0, 0.0, 0.0), 'L': (0.0, -1.0, 0.0, 0.0),
    'U': (1.0, 1.0, 0.0, 0.0), '<': (-1.0, 0.0, 0.0, 0.0), '>': (-1.0, -1.0, 0.0, 0.0),
    'M': (-1.0, 1.0, 0.0, 0.0), 't': (0.0, 0.0, 1.0, 0.0), 'b': (0.0, 0.0, -1.0, 0.0),
}

speedBindings = {
    'q': (1.1, 1.1), 'z': (0.9, 0.9), 'w': (1.1, 1.0),
    'x': (0.9, 1.0), 'e': (1.0, 1.1), 'c': (1.0, 0.9),
}

def vels(speed: float, turn: float) -> str:
    return f"currently:\tspeed {speed:.2f}\tturn {turn:.2f}"

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('teleop_twist_socket_node_simple')
    logger = node.get_logger()

    # Parameters
    node.declare_parameter('speed', 0.5)
    node.declare_parameter('turn', 1.0)
    node.declare_parameter('speed_limit', 1.0)
    node.declare_parameter('turn_limit', 1.0)
    node.declare_parameter('loop_rate', 20.0) # Rate for main loop and publishing
    node.declare_parameter('stamped', False)
    node.declare_parameter('frame_id', 'base_link')
    node.declare_parameter('socket_port', 9090)
    node.declare_parameter('socket_host', '0.0.0.0')

    current_speed_setting = node.get_parameter('speed').get_parameter_value().double_value
    current_turn_setting = node.get_parameter('turn').get_parameter_value().double_value
    speed_limit = node.get_parameter('speed_limit').get_parameter_value().double_value
    turn_limit = node.get_parameter('turn_limit').get_parameter_value().double_value
    loop_rate_hz = node.get_parameter('loop_rate').get_parameter_value().double_value
    use_stamped = node.get_parameter('stamped').get_parameter_value().bool_value
    twist_frame = node.get_parameter('frame_id').get_parameter_value().string_value
    socket_port = node.get_parameter('socket_port').get_parameter_value().integer_value
    socket_host = node.get_parameter('socket_host').get_parameter_value().string_value

    # Publisher
    twist_msg_type = TwistStamped if use_stamped else Twist
    publisher = node.create_publisher(twist_msg_type, 'cmd_vel', 10)

    # Socket setup
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((socket_host, socket_port))
    server_socket.listen(1)
    server_socket.setblocking(False) # Non-blocking
    logger.info(f"Socket server listening on {socket_host}:{socket_port}")

    client_socket = None
    client_address = None

    # Initial values
    x, y, z, th = 0.0, 0.0, 0.0, 0.0
    
    # Wait for subscribers
    '''
    sub_wait_count = 0
    while rclpy.ok() and publisher.get_subscription_count() == 0:
        if sub_wait_count % (int(loop_rate_hz) * 2) == 0: # Log every 2 seconds approx
             logger.info(f"Waiting for subscriber to connect to {publisher.topic_name}")
        time.sleep(1.0 / loop_rate_hz)
        sub_wait_count +=1
    if not rclpy.ok():
        logger.info("Shutdown requested before subscriber connected.")
        node.destroy_node()
        rclpy.shutdown()
        return
        '''

    logger.info("Teleop node started. Ready for client connection.")
    logger.info(vels(current_speed_setting, current_turn_setting))
    print(msg)

    loop_rate = node.create_rate(loop_rate_hz)
    
    try:
        while True:
            # Handle connections
            if client_socket is None:
                try:
                    # Check for new connection without blocking
                    readable, _, _ = select.select([server_socket], [], [], 0) # Timeout 0 for non-blocking check
                    if server_socket in readable:
                        client_socket, client_address = server_socket.accept()
                        client_socket.setblocking(False) # Non-blocking for client
                        logger.info(f"Accepted connection from {client_address}")
                except Exception as e:
                    logger.error(f"Error accepting connection: {e}")
                    if client_socket:
                        client_socket.close()
                    client_socket = None # Ensure it's reset

            # Handle commands from connected client
            if client_socket:
                try:
                    readable, _, _ = select.select([client_socket], [], [], 0) # Timeout 0
                    if client_socket in readable:
                        data = client_socket.recv(1024)
                        if data:
                            command = data.decode('utf-8').strip()
                            print(f"Received command: '{command}'")
                            if command == 'quit':
                                logger.info("Quit command received. Shutting down.")
                                break
                            
                            key = command[0] if command else ''
                            if key in moveBindings:
                                x, y, z, th = moveBindings[key]
                            elif key in speedBindings:
                                current_speed_setting = min(speed_limit, current_speed_setting * speedBindings[key][0])
                                current_turn_setting = min(turn_limit, current_turn_setting * speedBindings[key][1])
                                if current_speed_setting == speed_limit:
                                    logger.info("Linear speed limit reached!")
                                if current_turn_setting == turn_limit:
                                    logger.info("Angular speed limit reached!")
                                logger.info(vels(current_speed_setting, current_turn_setting))
                            elif key == 's': # Explicit stop
                                x, y, z, th = 0.0, 0.0, 0.0, 0.0
                                logger.info("Stopping robot via 's' command.")
                            # else: # Stop on any other key if desired
                                # x, y, z, th = 0.0, 0.0, 0.0, 0.0

                        else: # Empty data means client disconnected
                            logger.info(f"Client {client_address} disconnected.")
                            client_socket.close()
                            client_socket = None
                            x, y, z, th = 0.0, 0.0, 0.0, 0.0 # Stop the robot
                
                except (BlockingIOError, socket.timeout):
                    pass # No data received, totally normal for non-blocking
                except ConnectionResetError:
                    logger.info(f"Client {client_address} connection reset.")
                    if client_socket: client_socket.close()
                    client_socket = None
                    x, y, z, th = 0.0, 0.0, 0.0, 0.0 # Stop the robot
                except Exception as e:
                    logger.error(f"Error receiving data: {e}")
                    if client_socket: client_socket.close()
                    client_socket = None
                    x, y, z, th = 0.0, 0.0, 0.0, 0.0 # Stop the robot

            # Publish Twist message
            twist_msg_instance = twist_msg_type()
            
            if use_stamped:
                if not isinstance(twist_msg_instance, TwistStamped):
                    logger.fatal("Logic error: use_stamped is True but message is not TwistStamped!")
                    break 
                twist_content = twist_msg_instance.twist
                twist_msg_instance.header.stamp = node.get_clock().now().to_msg()
                twist_msg_instance.header.frame_id = twist_frame
            else:
                if not isinstance(twist_msg_instance, Twist):
                    logger.fatal("Logic error: use_stamped is False but message is not Twist!")
                    break
                twist_content = twist_msg_instance

            twist_content.linear.x = x * current_speed_setting
            twist_content.linear.y = y * current_speed_setting
            twist_content.linear.z = z * current_speed_setting # For up/down if t/b are used
            twist_content.angular.x = 0.0
            twist_content.angular.y = 0.0
            twist_content.angular.z = th * current_turn_setting
            
            publisher.publish(twist_msg_instance)

            loop_rate.sleep()

    except KeyboardInterrupt:
        logger.info('KeyboardInterrupt, shutting down.')
    except Exception as e:
        logger.error(f"An unhandled error occurred in main: {e}")
        import traceback
        logger.error(traceback.format_exc())
    finally:
        logger.info('Initiating shutdown...')
        # Send a final stop command
        if publisher and rclpy.ok():
            twist_msg_instance = twist_msg_type()
            if use_stamped:
                twist_content = twist_msg_instance.twist
                twist_msg_instance.header.stamp = node.get_clock().now().to_msg()
                twist_msg_instance.header.frame_id = twist_frame
            else:
                twist_content = twist_msg_instance
            
            twist_content.linear.x = 0.0
            twist_content.linear.y = 0.0
            twist_content.linear.z = 0.0
            twist_content.angular.x = 0.0
            twist_content.angular.y = 0.0
            twist_content.angular.z = 0.0
            try:
                publisher.publish(twist_msg_instance)
                logger.info('Final stop command published.')
            except Exception as e_pub:
                logger.error(f"Error publishing final stop command: {e_pub}")


        if client_socket:
            try:
                client_socket.close()
                logger.info("Client socket closed.")
            except Exception as e_sock:
                logger.error(f"Error closing client socket: {e_sock}")
        if server_socket:
            try:
                server_socket.close()
                logger.info("Server socket closed.")
            except Exception as e_sock:
                logger.error(f"Error closing server socket: {e_sock}")
        
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
        logger.info("ROS 2 node shutdown complete.")
        print("Application terminated.")

if __name__ == "__main__":
    main()