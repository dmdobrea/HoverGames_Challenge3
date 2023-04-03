# Import the necessary libraries
import os
import cv2                                                # OpenCV library - import before cv_bridge

import rclpy                                              # Python Client Library for ROS 2
from   rclpy.qos                  import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from   rcl_interfaces.msg         import ParameterDescriptor
from   rclpy.node                 import Node             # Handles the creation of nodes
from   sensor_msgs.msg            import Image            # Imports the built-in Image message type
from   cv_bridge                  import CvBridge         # Package to convert between ROS and OpenCV Images
from   std_msgs.msg               import Int8
from   std_msgs.msg               import Float64
from   .submodules.VideoDevices   import checkVideoDevices
import tflite_runtime.interpreter as tflite
import numpy                      as np



class VideoHealthDetect(Node):
    """
    Here I create an VideoDataHealthDetect class, which is a subclass of the Node class.
    """
    def __init__(self):
        """
        Class constructor to set up the node
        """

        # Initiate the Node class's constructor and give it a name
        super().__init__('health_plant_node')

        qos_profile = QoSProfile(
            reliability = QoSReliabilityPolicy.BEST_EFFORT,                  # RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability  = QoSDurabilityPolicy.TRANSIENT_LOCAL,               # RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history     = QoSHistoryPolicy.KEEP_LAST,                        # RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth       = 1 )

        # creates a parameter(s) with the name and default value(s)
        verbose_descriptor = ParameterDescriptor(description='Add verbosity to the running node.')
        self.declare_parameter('verbose', 1, verbose_descriptor)

        NavQplus_descriptor = ParameterDescriptor(description='0 => RPi    1 => NavQ+.')
        self.declare_parameter('NavQplus', 1, NavQplus_descriptor)

        width_descriptor = ParameterDescriptor(description='Image width [640]')
        self.declare_parameter('width', 640, width_descriptor)

        height_descriptor = ParameterDescriptor(description='Image height [480]')
        self.declare_parameter('height', 480, height_descriptor)

        filePath_descriptor = ParameterDescriptor(description='File path to the mp4 video')
        self.declare_parameter('file_path', '/home/user/FilmTest.mp4', filePath_descriptor)  # path to the video, like '/home/user/FilmTest.mp4' or 'None'

        target_descriptor = ParameterDescriptor(description='Set where detection will be done')
        self.declare_parameter('target', 'npu', target_descriptor)    # target = {'cpu', 'gpu', 'npu'} 
     
        noThreads_descriptor = ParameterDescriptor(description='Set the number of threads')
        self.declare_parameter('no_threads', 1, noThreads_descriptor)

        model_descriptor = ParameterDescriptor(description='Path of the object detection model.')
        self.declare_parameter('model_path', '/home/user/detection-precision-npu-in-uint8_out-uint8_channel_ptq.tflite', model_descriptor)

        delegate_descriptor = ParameterDescriptor(description='External delegate library path.')
        self.declare_parameter('delegate_path', '/lib/libvx_delegate.so', delegate_descriptor)
    
        mean_descriptor = ParameterDescriptor(description='Input_mean')
        self.declare_parameter('mean', 127.5, mean_descriptor)

        stdv_descriptor = ParameterDescriptor(description='Input standard deviation')
        self.declare_parameter('stdv', 127.5, stdv_descriptor)

        probThreshold_descriptor = ParameterDescriptor(description='Confidence threshold after that the detection is considered')
        self.declare_parameter('prob_thr', 0.9, probThreshold_descriptor)

    
        #==============================================================================================================================
        # Create the publisher. This publisher will publish images with detected abnormalities 
        # to the video_detect topic - this is used only for debug!
        self.publisher_         = self.create_publisher(Image,   'video_detect',    qos_profile)

        # used in the publication of the areas of the affected leaves
        self.publisher_vitis    = self.create_publisher(Float64, 'detect_vitis',    qos_profile)
        self.publisher_leafroll = self.create_publisher(Float64, 'detect_leafroll', qos_profile)

        # create a subscription for the flight_offboard topic published by flight_control_node
        # When agriHoverGames drone is in offboard mode it start the detection process
        self.subscription_offboard = self.create_subscription( Int8, 'flight_offboard', self.listener_callback_offboard, qos_profile)

        self.agriHG_is_offboard    = 0      # actual state
        self.agriHG_previous_state = 0      # previously state

        self.areaVitisLeaves    = 0.0       # total area of leaves having eriophyes vitis desease
        self.areaLeafrollLeaves = 0.0       # total area of leaves having grapevine leafroll desease

        self.tmp_areaVitisLeaves    = 0.0   # temporar (at each second) total area of leaves having eriophyes vitis desease
        self.tmp_areaLeafrollLeaves = 0.0   # temporar (at each second) total area of leaves having grapevine leafroll desease

        self.contor_offboard_send = 0
        self.contor_1s  = 0                 # for posting at each second the area of lives for each class detected in the previous second
        self.contor_10s = 0                 # for posting at every 10 seconds (as a negative number) the total area of diseased leaves 
                                            # from the moment when the system was set to Offboard mode.  
        #==============================================================================================================================
        # I will make a detection wvery 0.33 seconds
        timer_period = 0.33  # seconds

        # Create the timer
        self.timer = self.create_timer(timer_period, self.videoDetection_callback)

        # Get the config. parameters from Node
        verbose    = self.get_parameter('verbose').get_parameter_value().integer_value
        width      = self.get_parameter('width').get_parameter_value().integer_value
        height     = self.get_parameter('height').get_parameter_value().integer_value
        NavQplus   = self.get_parameter('NavQplus').get_parameter_value().integer_value
        filePath   = self.get_parameter('file_path').get_parameter_value().string_value
        target     = self.get_parameter('target').get_parameter_value().string_value
        no_threads = self.get_parameter('no_threads').get_parameter_value().integer_value
        model      = self.get_parameter('model_path').get_parameter_value().string_value
        ext_dlg    = self.get_parameter('delegate_path').get_parameter_value().string_value
        ext_dlg_o  = None
        
        # Create a VideoCapture object
        # The argument '0' gets the default webcam.
        self.vs = checkVideoDevices (filePath, 0, NavQplus, width, height, verbose)

        # Used to convert image format between ROS 2 and OpenCV
        self.br = CvBridge()

        #==============================================================================
        # to be used with NPU ang GPU
        ext_delegate = None
        ext_delegate_options = {}

        #==============================================================================
        # initialize the frame dimensions
        self.W = None
        self.H = None

        #=============================================================================
        if verbose == 1: print("[INFO_HP] : Configuring plant detection algorithm")

        if no_threads < 1:
            numberThreads = None
        else:
            numberThreads = no_threads

        # select place were to run recognition algorithm
        if target == "cpu":
            if verbose == 1: print("[INFO_HP] : Running human recognition on CPU")

        elif target == "gpu":
            if verbose == 1: print("[INFO_HP] : Running human recognition on GPU")
            os.environ["USE_GPU_INFERENCE"]="1"

        else:
            if target == "npu":
                if verbose == 1: print("[INFO] : Running human recognition on NPU")
                os.environ["USE_GPU_INFERENCE"]="0"
            else:
                sys.exit("[ERR.] : The target system is not known!!!! The program will quit!")

        if target == "npu" or target == "gpu":
            # parse extenal delegate options
            if ext_dlg_o is not None:
                options = ext_dlg_o.split(';')
                for o in options:
                    kv = o.split(':')
                    if(len(kv) == 2):
                        ext_delegate_options[kv[0].strip()] = kv[1].strip()

            # load external delegate
            if ext_dlg is not None:
                if verbose == 1: print("[INFO] : Loading external delegate from {} with args: {}".format(ext_dlg, ext_delegate_options))
                ext_delegate = [ tflite.load_delegate(ext_dlg, ext_delegate_options) ]

        # Load TFLite model and allocate tensors
        if verbose == 1: print("[INFO] : Loading external model {}".format(model))
        self.interpreter = tflite.Interpreter( model_path=model, experimental_delegates=ext_delegate, num_threads=numberThreads)
        self.interpreter.allocate_tensors()

        # Get input and output tensors.
        self.input_details  = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()

        # info regarding the output
        # print ('Output: ', output_details)

        # check the type of the input tensor
        self.floating_model = self.input_details[0]['dtype'] == np.float32
        if verbose == 1:
            if self.floating_model:
                print("[INFO_HP] : The input tensor is float32")
            else:
                print("[INFO_HP] : The input tensor is integer")

        self.modelH = self.input_details[0]['shape'][1]
        self.modelW = self.input_details[0]['shape'][2]
        if verbose == 1: 
            print( "[INFO_HP] : Model require the following resolution: H.{} x W.{}".format(self.modelH, self.modelW) )

        # sys.exit("Only to test the neuronal model!!!!")

        #=========================================================================================
        # at the first run the algorithm requires a very long time to init
        # here I do it
        fake_image = np.zeros((self.modelH, self.modelW,3), np.uint8)
        input_data = np.expand_dims(fake_image, axis=0)

        self.interpreter.set_tensor(self.input_details[0]['index'], input_data)
        self.interpreter.invoke()

    #===========================================================================================================================================
    def listener_callback_offboard ( self, msg ):
        verbose = self.get_parameter('verbose').get_parameter_value().integer_value

        self.agriHG_previous_state = self.agriHG_is_offboard
        self.agriHG_is_offboard = msg.data

        if self.agriHG_previous_state == 0 and self.agriHG_is_offboard == 1:
            # agriHoverGames entered in Offboard mode after a while set to zero all area of deseased leaves
            self.tmp_areaVitisLeaves    = 0.0     # temporar (at each second) total area of leaves having eriophyes vitis desease
            self.tmp_areaLeafrollLeaves = 0.0     # temporar (at each second) total area of leaves having grapevine leafroll desease

            self.areaVitisLeaves        = 0.0     # total area of leaves having eriophyes vitis desease
            self.areaLeafrollLeaves     = 0.0     # total area of leaves having grapevine leafroll desease

        if verbose == 1:
            print ("[INFO_HP] : agriHoverGames Offboard mode state {}".format(msg.data))

    #===========================================================================================================================================
    def videoDetection_callback ( self ):
        """
        Timer Callback function for each new detection
        """

        # get the parameters from the node
        verbose  = self.get_parameter('verbose').get_parameter_value().integer_value
        mean     = self.get_parameter('mean').get_parameter_value().double_value
        stdv     = self.get_parameter('stdv').get_parameter_value().double_value
        prob_thr = self.get_parameter('prob_thr').get_parameter_value().double_value

        # only in offboard mode detect
        if self.agriHG_is_offboard == 1:

            #=====================================================================
            # Capture frame-by-frame
            # This method returns True/False as well as the video frame.
            ret, frame = self.vs.read()

            if ret == False:
                if verbose == 1:
                    printf("[ERR._HP] : No frame !!!!")
                return

            # check to see if the frame dimensions are not set
            if self.W is None or self.H is None:
                # set the frame dimensions
                (self.H, self.W) = frame.shape[:2]
                if verbose == 1:
                    print( "[INFO_HP] : The input image has the resolution: H.{} x W.{}".format(self.H, self.W) )

            #===============================================================
            # create a compatible input data to the deep model

            # frame => (W, H, 3)
            # img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            if (self.H != self.modelH) or (self.W != self.modelW):
                img = cv2.resize(frame, (self.modelW, self.modelH), cv2.INTER_AREA)
                # now: img => (modelW, modelH, 3)

            # add N dim
            input_data = np.expand_dims(img, axis=0)
            # now input_data => (1, modelW, modelH, 3)

            # for floating point model convert input data to [0, 1]
            if self.floating_model:
                input_data = (np.float32(input_data) - mean) / stdv

            # send data to input
            self.interpreter.set_tensor(self.input_details[0]['index'], input_data)
           
            #===============================================================
            # start leavs detection
            self.interpreter.invoke()

            output_data = self.interpreter.get_tensor(self.output_details[0]['index'])[0]


            # real output data (classes and boxes)
            #output_data = (np.float32(output_data) - 135) * 0.027911635115742683

            #print('output', output_data)
            #print('output dim.:', output_data.shape)

            #print('first  : ', output_data[0, 0:7])
            #print('second : ', output_data[1, 0:7])
            # sys.exit("Only to test the neuronal output!!!!")

            # print (self.interpreter.get_tensor(self.output_details[0]['index'])[0][0])

            #=====================================================================
            # simulating
            self.tmp_areaVitisLeaves    += 0.1
            self.tmp_areaLeafrollLeaves += 0.2

            if self.floating_model:
                if verbose == 1 : print ("[INFO_HP] : floating point model => nothing to do ...")
            else:
                if verbose == 1 : print ("[INFO_HP] : Integer model!")

                for i in range(output_data.shape[0]):

                    # get area  arond setected leaf with eriophyes vitis
                    confidence = output_data[i, 5]/255.0

                    # check against probability threshold, background and grapevine leafroll
                    if confidence > prob_thr and output_data[i, 5] > output_data[i, 4] and output_data[i, 5] > output_data[i, 6]:
                        left   = int ( self.W * output_data[i, 0]/255.0)   # detection[3]*W
                        top    = int ( self.H * output_data[i, 1]/255.0)   # detection[4]*H
                        right  = int ( self.W * output_data[i, 2]/255.0)   # detection[5]*W
                        bottom = int ( self.H * output_data[i, 3]/255.0)   # detection[6]*H

                        self.tmp_areaVitisLeaves += (right - left)*(bottom - top)


                    # get area arond setected leaf with grapevine leafroll
                    confidence = output_data[i, 6]/255.0

                    # check against probability threshold, background and eriophyes vitis
                    if confidence > prob_thr and output_data[i, 6] > output_data[i, 4] and output_data[i, 6] > output_data[i, 5]:
                        left   = int ( self.W * output_data[i, 0]/255.0)   # detection[3]*W
                        top    = int ( self.H * output_data[i, 1]/255.0)   # detection[4]*H
                        right  = int ( self.W * output_data[i, 2]/255.0)   # detection[5]*W
                        bottom = int ( self.H * output_data[i, 3]/255.0)   # detection[6]*H

                        self.tmp_areaLeafrollLeaves += (right - left)*(bottom - top)

            #=====================================================================
            # sendin the image with the detection boxes
            if verbose == 1:
                # draw boxes around detected objects
                if self.floating_model:
                    print ("[INFO_HP] : floating point model => nothing to do ...")
                else:
                    print ("[INFO_HP] : Integer model!")
                    # is working with: detection-ssd_mobilenet_v3-train-CV_807.tflite
                    #                  detection-ssd_mobilenet_v3-train-CV_817.tflite
                    #                  detection-ssd_mobilenet_v3-test-CV_827.tflite
                    for i in range(output_data.shape[0]):

                        # draw a rectangle arond retected leaf with eriophyes vitis
                        confidence = output_data[i, 5]/255.0

                        # check against probability threshold, background and grapevine leafroll disease
                        if confidence > prob_thr and output_data[i, 5] > output_data[i, 4] and output_data[i, 5] > output_data[i, 6]:
                            left   = int ( self.W * output_data[i, 0]/255.0)   # detection[3]*W
                            top    = int ( self.H * output_data[i, 1]/255.0)   # detection[4]*H
                            right  = int ( self.W * output_data[i, 2]/255.0)   # detection[5]*W
                            bottom = int ( self.H * output_data[i, 3]/255.0)   # detection[6]*H

                            #draw a red rectangle around detected object
                            cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), thickness=2)

                            label = "Vitis: {:.2f}%".format(confidence*100)
                            y = top - 15 if top - 15 > 15 else top + 15
                            cv2.putText(frame, label, (left, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

                        # draw a rectangle arond retected leaf with grapevine leafroll
                        confidence = output_data[i, 6]/255.0

                        # check against probability threshold, background and eriophyes vitis
                        if confidence > prob_thr and output_data[i, 6] > output_data[i, 4] and output_data[i, 6] > output_data[i, 5]:
                            left   = int ( self.W * output_data[i, 0]/255.0)   # detection[3]*W
                            top    = int ( self.H * output_data[i, 1]/255.0)   # detection[4]*H
                            right  = int ( self.W * output_data[i, 2]/255.0)   # detection[5]*W
                            bottom = int ( self.H * output_data[i, 3]/255.0)   # detection[6]*H

                            #draw a blue rectangle around detected object
                            cv2.rectangle(frame, (left, top), (right, bottom), (255, 0, 0), thickness=2)

                            label = "Leafroll: {:.2f}%".format(confidence*100)
                            y = top - 15 if top - 15 > 15 else top + 15
                            cv2.putText(frame, label, (left, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

                # Publish the image. The 'cv2_to_imgmsg' method converts an 
                # OpenCV image to a ROS 2 image message
                self.publisher_.publish(self.br.cv2_to_imgmsg(frame))
    
                # Display the message on the console
                self.get_logger().info('Publishing video frame')

            #=====================================================================
            # posting the the area of diseased leaves from the 2 classes
            self.contor_1s  += 1 
            self.contor_10s += 1 

            msg1 = Float64 ()
            msg2 = Float64 ()


            if self.contor_1s == 3:
                # 1 s pass => add temp results to final results
                self.areaVitisLeaves    += self.tmp_areaVitisLeaves
                self.areaLeafrollLeaves += self.tmp_areaLeafrollLeaves

                # send temp results
                msg1.data = self.tmp_areaVitisLeaves
                msg2.data = self.tmp_areaLeafrollLeaves

                self.publisher_vitis.publish(msg1)
                self.publisher_leafroll.publish(msg2)

                # set on zero in order to get the new areas
                self.tmp_areaVitisLeaves    = 0.0
                self.tmp_areaLeafrollLeaves = 0.0

                self.contor_1s = 0

            if self.contor_10s == 30:
                # 10 seconds pass sent also the areas descovered up to now
                msg1.data = - self.areaVitisLeaves
                msg2.data = - self.areaLeafrollLeaves

                self.publisher_vitis.publish(msg1)
                self.publisher_leafroll.publish(msg2)

                self.contor_10s = 0

        else:
            # agriHoverGames is not more in Offboard mode => send 
            # the area of diseased leaves continuously
            self.contor_offboard_send += 1

            # send the area of diseased leaves every 5 seconds
            if self.contor_offboard_send == 15:
                msg1 = Float64 ()
                msg2 = Float64 ()

                msg1.data = self.areaVitisLeaves
                msg2.data = self.areaLeafrollLeaves

                self.publisher_vitis.publish(msg1)
                self.publisher_leafroll.publish(msg2)

                self.contor_offboard_send = 0

#=======================================================================================================================================
def main(args=None):
    # Initialize the ROS 2 library for Python
    rclpy.init(args=args)

    # Create the node
    videoHealth_publisher = VideoHealthDetect()

    # Spin the node so the callback function is called.
    rclpy.spin(videoHealth_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    videoHealth_publisher.destroy_node()

    # Shutdown the ROS 2 client library for Python
    rclpy.shutdown()

if __name__ == '__main__':
    main()
                      
