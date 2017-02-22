#!/usr/bin/env python

# ROS imports
import rospy

# Messages
from std_msgs.msg import Float64, String
from sensor_nodes.srv import Turn, TurnRequest, TurnResponse
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse

# Python Utilites
import sys
import time

# Servo Library
import RPi.GPIO as GPIO

class State:
	# Duty Rate (%)
	LEFT  = 7.9
	STOP  = 0
	RIGHT = 6.1
	@staticmethod
	def get_state_string(s):
		if s == State.LEFT: return 'LEFT'
		elif s == State.RIGHT: return 'RIGHT'
		else: return 'STOP'

class ServoController:
	def __init__(self):
		self.lock = False
		self.status = State.STOP
		self.position = 0.0 # Limits are between -2 and 2 (revolutions)

		self.pub_status = rospy.Publisher('/servo/status', String, queue_size=1)
		self.pub_position = rospy.Publisher('/servo/position', Float64, queue_size=1)
		self.srv_turn = rospy.Service('turn', Turn, self._handle_turn)
		self.srv_unlatch = rospy.Service('unlatch', Trigger, self._handle_unlatch)

		# Initialize Servo
		GPIO.setmode(GPIO.BCM)
		GPIO.setup(25, GPIO.OUT)
		self.servo = GPIO.PWM(25, 50)
		self.servo.start(0)
	
	def _handle_turn(self, srv):
		if not self.lock:
			# prevent all other process by locking
			self.lock = True

			# Store commands locally
			is_left  = bool(srv.is_left)
			duration = float(srv.duration) # int in seconds
		
			# Set Status
			self.status = (State.LEFT if is_left else State.RIGHT)
		
			# Publish Turn Status
			status_msg = String()
			status_msg.data = State.get_state_string(self.status)
			
			# Setup Position Message
			pos_msg = Float64()
			pos_msg.data = self.position

			start_time = time.time()
			self.servo.ChangeDutyCycle(self.status)
			while (time.time() - start_time) < duration:
				self.pub_status.publish(status_msg)
				self.pub_position.publish(pos_msg)
				if (pos_msg.data > -2 and pos_msg.data < 2) or (pos_msg.data < -2 and is_left) or (pos_msg.data > 2 and not is_left):
					pos_msg.data = (pos_msg.data+0.000145 if is_left else pos_msg.data-0.000175)
				else:
					self.servo.ChangeDutyCycle(0)
					break

			# Set back to stop status
			self.servo.ChangeDutyCycle(0)
			self.status = State.STOP

			# Publish stop status
			status_msg.data = State.get_state_string(self.status)
			self.pub_status.publish(status_msg)

			# Store position
			self.position = pos_msg.data

			# Unlatch locking
			self.lock = False

			# Return response
			return TurnResponse(True)
		else:
			# Catch locking issue
			return TurnResponse(False)

	def _handle_unlatch(self, srv):
		rospy.wait_for_service('turn')
		unlatch_resp = TriggerResponse()

		if not self.lock:
			# Call turn service
			try:
				unlatch = rospy.ServiceProxy('turn', Turn)
				req = TurnRequest(True, 0.5)
				resp1 = unlatch(req)

				unlatch_resp.success = True
				unlatch_resp.message = "The arm was unlatched"
			except rospy.ServiceException, e:
				rospy.loginfo("Service call failed: {}".format(e))
				unlatch_resp.success = False
				unlatch_resp.message = "The arm was NOT unlatched due to a rosservice error"
		else:
			unlatch_resp.success = False
			unlatch_resp.message = "The arm was NOT unlatched due to the servo being locked"
			
		return unlatch_resp

	def cleanup(self):
		self.servo.stop()
		GPIO.cleanup()
		rospy.loginfo("Successfully cleaned up node")
		
if __name__ == "__main__":
	rospy.init_node("ServoController")
	sc = ServoController()
	rospy.on_shutdown(sc.cleanup)
	rospy.spin()
