import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.qos  import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import TimesyncStatus
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import VehicleLocalPosition

class OffboardControl(Node):

	def __init__(self):
		super().__init__('my_OffboardControl')
		qos_profile = QoSProfile(
			reliability = QoSReliabilityPolicy.BEST_EFFORT,                  # RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
			durability  = QoSDurabilityPolicy.TRANSIENT_LOCAL,               # RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL, 
			history     = QoSHistoryPolicy.KEEP_LAST,                        # RMW_QOS_POLICY_HISTORY_KEEP_LAST,
			depth       = 1 )

		# internal variables
		self.nav_state       = VehicleStatus.NAVIGATION_STATE_MAX
		self.arming_state    = VehicleStatus.ARMING_STATE_MAX
		self.local_timestamp = 0
		
		self.flag_arm = 1

		# SUBSCRIBER NODE(S)
		self.vehicle_status_Subscriber = self.create_subscription ( VehicleStatus,        '/fmu/vehicle_status/out',         self.vehicle_status_callback, qos_profile ) 
		self.timesync_Subscriber       = self.create_subscription ( TimesyncStatus,       '/fmu/timesync/out',               self.timesync_callback,       qos_profile )
		# self.vehicle_locpos_Subscriber = self.create_subscription ( VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_locpos_callback, qos_profile )

		# PUBLISHER NODE(S)
			# for general commands: ARM, DISARM, LAND etc.
		self.vehicle_command_Publisher = self.create_publisher ( VehicleCommand,      "/fmu/vehicle_command/in",       qos_profile )	
		self.offboard_mode_Publisher   = self.create_publisher ( OffboardControlMode, '/fmu/offboard_control_mode/in', qos_profile )
		self.trajectory_Publisher      = self.create_publisher ( TrajectorySetpoint,  '/fmu/trajectory_setpoint/in',   qos_profile )

		self.mem_x = 0
		self.mem_y = 0

	       	# timer create
		timer_period = 0.1  # seconds
		self.timer = self.create_timer(timer_period, self.cmd_offboard_timer)
		self.i = 0
		self.j = 0
		
		#initial fly mode!!!!
		self.mode_Possition()
		
	def vehicle_status_callback(self, msg):
        	# TODO: handle NED->ENU transformation
        	# print("NAV_STATUS: ", msg.nav_state)
        	# print("  - offboard status: ", VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        	self.nav_state    = msg.nav_state
        	self.arming_state = msg.arming_state
        
    
	def cmd_offboard_timer(self):
		
		if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
			self.i += 1
			
			if self.i > 0 and self.i <= 80:
				self.mem_x += 0.5
				self.mem_y += 0.0  
				self.newTrajectorySet(self.mem_x, self.mem_y, -5.0, 0.0, True)
			if self.i > 100 and self.i <= 160:
				self.mem_x += 0.0
				self.mem_y -= 0.5
				self.newTrajectorySet(self.mem_x, self.mem_y, -5.0, -1.5707, True)
			if self.i > 200 and self.i <= 240:
				self.mem_x -= 0.5
				self.mem_y += 0.0  
				self.newTrajectorySet(self.mem_x, self.mem_y, -5.0, -3.1415, True)	
			if self.i > 300 and self.i <= 320:
				self.mem_x += 0.0
				self.mem_y += 0.5  
				self.newTrajectorySet(self.mem_x, self.mem_y, -5.0, 1.5707, True)			
				
			print("[INFO] : counter = ", self.i)
			if self.i > 321:
				self.i = 321
				
				self.land()
				print("[INFO] : Land command was set")
		else:
                    self.newTrajectorySet(0.0, 0.0, 0.0, 0.0, False)
			
	#=========================================================================================================================
	def newTrajectorySet (self, x_SN, y_VE, z_Down, heading_angle, position = True):
		# Publish offboard control modes
		offboard_msg = OffboardControlMode()
		offboard_msg.timestamp    = self.local_timestamp
		
		if position:
			offboard_msg.position     = True
			offboard_msg.velocity     = False
		else:
			offboard_msg.position     = False
			offboard_msg.velocity     = True
			
		offboard_msg.acceleration = False
		offboard_msg.attitude     = False		# add by myself
		offboard_body_rate        = False		# add by myself
		
		self.offboard_mode_Publisher.publish(offboard_msg)	
		#===============================================================
	
	
		# NED local world frame
		# Publish the trajectory setpoints 
		trajectory_msg = TrajectorySetpoint()
		trajectory_msg.timestamp   = self.local_timestamp
		
		if position:
			trajectory_msg.x = x_SN		        # X Position in meters (positive is forward or North)
			trajectory_msg.y = y_VE		        # Y Position in meters (positive is right or East)
			trajectory_msg.z = z_Down		# Z Position in meters (positive is down)	
			trajectory_msg.vx = float("nan")	# X velocity in m/s (positive is forward or North)
			trajectory_msg.vy = float("nan")	# Y velocity in m/s (positive is right or East)
			trajectory_msg.vz = float("nan")	# Z velocity in m/s (positive is down)
		else:
			trajectory_msg.vx = x_SN        	# X velocity in m/s (positive is forward or North)
			trajectory_msg.vy = y_VE		# Y velocity in m/s (positive is right or East) - velocity[1]
			trajectory_msg.vz = z_Down		# Z velocity in m/s (positive is down)
			trajectory_msg.x = float("nan")	        # X Position in meters (positive is forward or North)
			trajectory_msg.y = float("nan")	        # Y Position in meters (positive is right or East)
			trajectory_msg.z = float("nan")	        # Z Position in meters (positive is down)	
		
		trajectory_msg.yaw = heading_angle		# yaw or heading in radians (0 is forward or North)
		
		trajectory_msg.jerk[0] = float("nan")
		trajectory_msg.jerk[1] = float("nan")
		trajectory_msg.jerk[2] = float("nan")	
		trajectory_msg.acceleration[0] = float("nan")	# X acceleration in m/s/s (positive is forward or North)
		trajectory_msg.acceleration[1] = float("nan")	# Y acceleration in m/s/s (positive is right or East)
		trajectory_msg.acceleration[2] = float("nan")	# Z acceleration in m/s/s (positive is down)
		trajectory_msg.yawspeed = 0.0			# yaw rate in rad/s
			
		self.trajectory_Publisher.publish(trajectory_msg)
	
	#=========================================================================================================================	
	def arm(self):
		self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0)
		self.get_logger().info("Arm command send")

	def disarm(self):
		self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, 0.0)
		self.get_logger().info("Disarm command send")

	def takeoff(self):
		self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF)
		self.get_logger().info("Takeoff command send")

	# daca este deja in aer merge foarte bine
	def land(self):
		self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
		self.get_logger().info("Land command send")

	def mode_Possition(self):
		self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 3.0)   #  1.0 => Manual,   2.0 => Altitude,  3.0 => Possition
		self.get_logger().info("Possition mode command send")                            #  4.0 => Mission,  5.0 => Acro,      7.0 => Stabilized,  

			
	def timesync_callback(self, msg):
	        self.local_timestamp = msg.timestamp
	
	"""
	def vehicle_locpos_callback (self, msg):
		self.j += 1
		if self.j == 5:
			print("[INFO] : from HoverGames - x = {}    y = {}    z = {}".format(msg.x, msg.y, msg.z))	
			self.j = 0	
	"""

	# parameter 1 and 2, as defined by MAVLink uint16 VEHICLE_CMD enum.
	def publish_vehicle_command(self, command, param1=0.0, param2=0.0):  
		msg = VehicleCommand()
		msg.timestamp = self.local_timestamp
		msg.param1 = param1
		msg.param2 = param2
		msg.command = command         # command ID
		msg.target_system     = 1     # system which should execute the command
		msg.target_component  = 1     # component which should execute the command, 0 for all components
		msg.source_system     = 1     # system sending the command
		msg.source_component  = 1     # component sending the command
		msg.from_external     = True
		self.get_logger().info('Next Vehicle Command Set To: "%s"' % msg)
		self.vehicle_command_Publisher.publish(msg)

def main(args=None):
	rclpy.init(args=args)

	offboard_control = OffboardControl()

	rclpy.spin(offboard_control)

	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	offboard_control.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main() 
