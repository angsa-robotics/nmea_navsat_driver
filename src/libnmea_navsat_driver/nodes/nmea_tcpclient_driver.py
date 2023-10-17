# Copyright 2022 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import socket
import sys
import time
from time import sleep

import rclpy

from libnmea_navsat_driver.driver import Ros2NMEADriver

def main(args=None):
    rclpy.init(args=args)
    driver = Ros2NMEADriver()

    try:
        gnss_ip = driver.declare_parameter('ip', '192.168.0.70').value
        gnss_port = driver.declare_parameter('port', 9111).value
        buffer_size = driver.declare_parameter('buffer_size', 4096).value
        socket_timeout = driver.declare_parameter('socket_timeout', 5.0).value
        no_data_reconnection_timeout = driver.declare_parameter('no_data_reconnection_timeout', 20).value
    except KeyError as e:
        driver.get_logger().err("Parameter %s not found" % e)
        sys.exit(1)

    frame_id = driver.get_frame_id()

    driver.get_logger().info("Using gnss sensor with ip {} and port {}".format(gnss_ip, gnss_port))

    # Connection-loop: connect and keep receiving. If receiving fails, reconnect
    # Connect to the gnss sensor using tcp
    while rclpy.ok():
        try:
            # Create a socket
            gnss_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

            # Connect to the gnss sensor
            gnss_socket.settimeout(socket_timeout)
            gnss_socket.connect((gnss_ip, gnss_port))
            last_data_packet_time = time.time()
        except socket.error as exc:
            driver.get_logger().error("Caught exception socket.error when setting up socket: %s" % exc)
            sleep(5.0)
            driver.get_logger().info("Trying again now.")
            continue

        # recv-loop: When we're connected, keep receiving stuff until that fails
        partial = ""
        try:
            while rclpy.ok():
                try:
                    gnss_socket.settimeout(socket_timeout)
                    partial += gnss_socket.recv(buffer_size).decode("ascii")

                    if time.time() > last_data_packet_time + no_data_reconnection_timeout:
                        driver.get_logger().error(f"No data since {time.time() - last_data_packet_time:.1f}s, recreating socket.")
                        raise TimeoutError

                    # strip the data
                    lines = partial.splitlines()
                    if partial.endswith('\n'):
                        full_lines = lines
                        partial = ""
                    else:
                        full_lines = lines[:-1]
                        partial = lines[-1]

                    if not full_lines:
                        driver.get_logger().warn("No data from device...", throttle_duration_sec=5.0)
                        sleep(0.05)
                        continue
                    last_data_packet_time = time.time()

                    for data in full_lines:
                        try:
                            if driver.add_sentence(data, frame_id):
                                driver.get_logger().info("Received sentence: %s" % data)
                            else:
                                driver.get_logger().warn("Error with sentence: %s" % data)
                        except ValueError as e:
                            driver.get_logger().warn(
                                "Value error, likely due to missing fields in the NMEA message. "
                                "Error was: %s. Please report this issue to me. " % e)

                except socket.error as exc:
                    driver.get_logger().error("Caught exception socket.error when receiving: %s" % exc)
                    break
                except TimeoutError:
                    # Create new socket in outer loop
                    break
        finally:
            try:
                driver.get_logger().info(f"Closing old socket")
                gnss_socket.close()
            except Exception as e:
                driver.get_logger().error(f"Failed to shut down socket: {e}")
