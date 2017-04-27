#!/usr/bin/env python

# ROS imports
import rospy
from std_msgs.msg import Float64

# Serial read
import serial

class SerialPublisher:
	def __init__(self):
		# Create publishers
		self.temp_pub     = rospy.Publisher('/temp', Float64, queue_size=1)
		self.humidity_pub = rospy.Publisher('/humidity', Float64, queue_size=1)
		self.light_pub    = rospy.Publisher('/light', Float64, queue_size=1)
		self.voltage_pub  = rospy.Publisher('/voltage', Float64, queue_size=1)
		
		# Specify serial port parameters
		port = '/dev/ARDUINO'
		baudrate = 9600
		parity=serial.PARITY_NONE
		stopbits=serial.STOPBITS_ONE
		bytesize=serial.EIGHTBITS

		# Try opening serial port with specified parameters
		#try:
		self.ser = serial.Serial(port=port,baudrate=baudrate,parity=parity,stopbits=stopbits,bytesize=bytesize)
		#except:
			#rospy.signal_shutdown('Could not open serial port: {}'.format(port))

		# Begin to read from port
		self.readPort()

	def readPort(self):
		# Initialize serial read response and ros messages
		read_rslt    = ''
		temp_msg     = Float64()
		humidity_msg = Float64()
		light_msg    = Float64()
		voltage_msg  = Float64()

		try: 
			self.ser.open()
		except Exception, e:
			print 'error open serial port: {}'.format(str(e))

		while self.ser.is_open:
			read_rslt = self.ser.readline()



			rospy.loginfo(read_rslt[:-2].split(','))

			temp_msg.data, humidity_msg.data, light_msg.data, voltage_msg.data = [float(i) for i in read_rslt[:-2].split(',')]



			#temp_msg.data, humidity_msg.data, light_msg.data, voltage_msg.data = read_rslt.split(',')

			

			
			self.temp_pub.publish(temp_msg)
			self.humidity_pub.publish(humidity_msg)
			self.light_pub.publish(light_msg)
			self.voltage_pub.publish(voltage_msg)

if __name__ == '__main__':
    rospy.init_node('SerialPublisher')
    sp = SerialPublisher()
    rospy.spin()













