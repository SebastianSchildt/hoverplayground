import sys
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import BatteryState # Enable use of the sensor_msgs/BatteryState message type
import json
import serial

from hovercar_parser.parse_hoverdata import init_state, init_write_file, parse_from_serial, write_into_file

class HoverInfoPublisher(Node):

  def __init__(self):
    super().__init__('hover_info_pub')
    self.declare_parameter('log_path', None)
    self.declare_parameter('battery_capacity')
    self.declare_parameter('hover_port')


    self.log_path = self.get_parameter('log_path').value
    print("write into ", self.log_path)
    self.battery_capacity = self.get_parameter('battery_capacity').value
    self.hover_port = self.get_parameter('hover_port').value

    self.ser = serial.Serial(str(self.hover_port), 115200, timeout=None)  # open serial port

    self.dataPtr, self.state, self.SerialFrame = init_state()
    if self.log_path:
      self.log_file = init_write_file(self.log_path)
    self.parsed_data = None

    self.publisher_battery_state = self.create_publisher(BatteryState, '/battery_status', 10)

    timer_period = 0.5
    self.timer = self.create_timer(timer_period, self.get_battery_state)

    self.battery_voltage = 0.0 # Initialize the battery voltage level
    self.percent_charge_level = 0  # Initialize the percentage charge level
    self.frame_id = 0

  def serial_to_json(self, file = None):
        b = self.ser.read(size=1)
        self.state, self.dataPtr, self.parsed_data = parse_from_serial(b, self.state, self.dataPtr, self.SerialFrame)

        if file:
            write_into_file(file, self.parsed_data)

  def get_battery_state(self):
    self.serial_to_json(self.log_file)
    msg = BatteryState() # Create a message of this type
    if self.parsed_data:
      self.battery_voltage = float(self.parsed_data['batVoltage'])
    msg.voltage = self.battery_voltage
    msg.percentage = msg.voltage *100.0 / self.battery_capacity

    msg.header.stamp = self.get_clock().now().to_msg()
    self.frame_id = self.frame_id +1
    msg.header.frame_id = str(self.frame_id)
    self.publisher_battery_state.publish(msg) # Publish BatteryState message

def main(args=None):
  rclpy.init(args=args)
  hover_info_pub = HoverInfoPublisher()
  rclpy.spin(hover_info_pub)
  hover_info_pub.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()
