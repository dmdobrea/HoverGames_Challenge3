import rclpy
import numpy as np
from   rclpy.node          import Node
from   rclpy.qos           import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy, QoSLivelinessPolicy
from   rcl_interfaces.msg  import ParameterDescriptor
from   sensor_msgs.msg     import Image 	        # Imports the built-in Image message type
from   std_msgs.msg        import Int8
import cv2 				                # OpenCV library - import before cv_bridge!
from   cv_bridge           import CvBridge 	        # Package to convert between ROS and OpenCV Images

from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import TimesyncStatus
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import OffboardControlMode

PI        = 3.1415926
PIpe2     = 1.5707963
doiPI     = 6.2831852
treiPIpe2 = 4.7123889

class OffboardControl(Node):

    def __init__(self):

        super().__init__('flight_control_node')
        
        qos_profile = QoSProfile(
	    reliability = QoSReliabilityPolicy.RELIABLE,		        # Gazebo => QoSReliabilityPolicy.BEST_EFFORT
	    durability  = QoSDurabilityPolicy.TRANSIENT_LOCAL,      # Gazebo => QoSDurabilityPolicy.TRANSIENT_LOCAL 
	    liveliness  = QoSLivelinessPolicy.AUTOMATIC,
	    history     = QoSHistoryPolicy.KEEP_LAST,               # Gazebo => QoSHistoryPolicy.KEEP_LAST
	    depth       = 1 )
      
	# creates a parameter with the name and default value
        verbose_descriptor = ParameterDescriptor(description='Add verbosity to the running node.')
        self.declare_parameter('verbose', 1, verbose_descriptor)
    
        depthCam_descriptor = ParameterDescriptor(description='agriHoverGames fly based on Depth Camera D435i.')
        self.declare_parameter('depth_cam', 1, depthCam_descriptor)

        # internal variables
        self.nav_state       = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state    = VehicleStatus.ARMING_STATE_MAX
        self.local_timestamp = 0
   
	#createa fake black image
        self.blank_image = np.zeros((480,640,3), np.uint8)
        self.contor = 0;		

	# SUBSCRIBER NODE(S)
        self.vehicle_status_Subscriber = self.create_subscription ( VehicleStatus,  '/fmu/vehicle_status/out',  self.vehicle_status_callback, qos_profile ) 
        self.timesync_Subscriber       = self.create_subscription ( TimesyncStatus, '/fmu/timesync/out',        self.timesync_callback,       qos_profile )

        # PUBLISHER NODE(S)
		# for general commands: ARM, DISARM, LAND etc.
        self.vehicle_command_Publisher = self.create_publisher ( VehicleCommand,      "/fmu/vehicle_command/in",       qos_profile )	
        self.offboard_mode_Publisher   = self.create_publisher ( OffboardControlMode, '/fmu/offboard_control_mode/in', qos_profile )
        self.trajectory_Publisher      = self.create_publisher ( TrajectorySetpoint,  '/fmu/trajectory_setpoint/in',   qos_profile )
   
        # Create the publisher. This publisher will publish images of type Image
        # to the video_frames topic.
        self.publisher_videoFlight     = self.create_publisher(Image, 'video_flight',   qos_profile)

        # Create a publishe for the offboard state
        # if agriHoverGame is in the Offboard mode => publish 1
        # if agriHoverGame is not in Offboard mode => publish 0
        self.publisher_offboardMode    = self.create_publisher(Int8, 'flight_offboard', qos_profile)

        # timer create
        timer_period = 0.3  # seconds
        self.timer = self.create_timer(timer_period, self.cmd_offboard_timer)
		
        self.dt     = timer_period
        self.R      = 10
        self.theta  = 0.0
        self.omega  = 0.5
        self.V      = self.R * self.omega

	# Used to convert image format between ROS 2 and OpenCV
        self.br = CvBridge()
		
	#initial fly mode!!!!
        self.mode_Possition()
		
    def vehicle_status_callback(self, msg):
        # gets the parameter from the node
        verbose = self.get_parameter('verbose').get_parameter_value().integer_value

        if verbose == 1:
            print("[INFO_FC] : NAV_STATUS: ", msg.nav_state)
            print("[INFO_FC] : ARM status: ", msg.arming_state)
        
        self.nav_state    = msg.nav_state
        self.arming_state = msg.arming_state    

        msg_state = Int8()
        # in real world
        # if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
        # only for tests - is more easy to arm than put in offboard 
        # mode when the drone is on your table    
        if self.arming_state == VehicleStatus.ARMING_STATE_ARMED: 
            msg_state.data = 1
            self.publisher_offboardMode.publish(msg_state)
        else:
            msg_state.data = 0
            self.publisher_offboardMode.publish(msg_state)

    
    def cmd_offboard_timer(self):
        # gets the depth_cam, verbositi parameters from the node
        depth_cam = self.get_parameter('depth_cam').get_parameter_value().integer_value
        verbose   = self.get_parameter('verbose').get_parameter_value().integer_value
    	
        if VehicleStatus.NAVIGATION_STATE_OFFBOARD == VehicleStatus.NAVIGATION_STATE_OFFBOARD:   # if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:

            # do the magic things and drive the agriHoverGames
            if depth_cam == 1:
                # 1. get depth data from D435i
                # 2. process data from D435i
                # 3. Execute flight commands
        
                # if verbose then display image data to help debug
                if verbose == 1:
                    # for debug publish the processing result
          
                    # Publish the image.
                    # The 'cv2_to_imgmsg' method converts an OpenCV
                    # image to a ROS 2 image message
                    label = "Frame no.: {}".format(self.contor)
                    tmp_frame = self.blank_image 
                    cv2.putText(tmp_frame, label, (30, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
          
                    self.publisher_videoFlight.publish(self.br.cv2_to_imgmsg(tmp_frame))
                    self.contor = self.contor + 1;
                    
                    # Display the message on the console
                    self.get_logger().info('Publishing video frame') 
      
            else:
                # fly in a circular way
                self.newTrajectorySet(self.V * np.sin(self.theta), self.V * np.cos(self.theta), 0.0, self.angleNED(self.theta), False) 
                self.theta = self.theta + self.omega * self.dt
                if self.theta > doiPI:
                    self.theta = 0;
    
        else:
            self.newTrajectorySet(0.0, 0.0, 0.0, 0.0, False)
			
    def angleNED (self, unghi):
        if unghi >= 0 and unghi < PIpe2:
            return -(unghi - PIpe2)
        if unghi >= PIpe2 and unghi < PI:
            return -(unghi - PIpe2)
        if unghi >= PI and unghi < treiPIpe2:
            return -(unghi - PIpe2)
        if unghi >= treiPIpe2 and unghi < doiPI:
            return (-(unghi - PIpe2) + doiPI)
		
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
        offboard_msg.attitude     = False		# add
        offboard_body_rate        = False		# add
		
        self.offboard_mode_Publisher.publish(offboard_msg)	
        #===============================================================
		
        # NED local world frame
	# Publish the trajectory setpoints 
        trajectory_msg             = TrajectorySetpoint()
        trajectory_msg.timestamp   = self.local_timestamp
		
        if position:
            trajectory_msg.x = x_SN			# X Position in meters (positive is forward or North)
            trajectory_msg.y = y_VE			# Y Position in meters (positive is right or East)
            trajectory_msg.z = z_Down		        # Z Position in meters (positive is down)	
            trajectory_msg.vx = float("nan")	        # X velocity in m/s (positive is forward or North)
            trajectory_msg.vy = float("nan")	        # Y velocity in m/s (positive is right or East)
            trajectory_msg.vz = float("nan")	        # Z velocity in m/s (positive is down)
        else:
            trajectory_msg.vx = x_SN		        # X velocity in m/s (positive is forward or North)
            trajectory_msg.vy = y_VE		        # Y velocity in m/s (positive is right or East)
            trajectory_msg.vz = z_Down		        # Z velocity in m/s (positive is down)
            trajectory_msg.x = float("nan")		# X Position in meters (positive is forward or North)
            trajectory_msg.y = float("nan")		# Y Position in meters (positive is right or East)  
            trajectory_msg.z = float("nan")		# Z Position in meters (positive is down)	

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
	        
		
    # parameter 1 and 2, as defined by MAVLink uint16 VEHICLE_CMD enum.
    def publish_vehicle_command(self, command, param1=0.0, param2=0.0): 
        verbose   = self.get_parameter('verbose').get_parameter_value().integer_value

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

        if verbose == 1:
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
