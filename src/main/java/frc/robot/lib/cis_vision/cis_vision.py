#!/usr/bin/env python3

from pathlib import Path
import sys
import cv2
import depthai as dai
import numpy as np
import time
import threading
from networktables import NetworkTables
from AStarPlanner import AStarPlanner

MM_TO_INCH = 0.0393700787
MM_TO_FEET = 0.0032808399

MOVING_AVERAGE_WINDOW_SIZE = 10
'''The purpose of the moving average filter is to reduce the variability of the
spatial YOLO detections.'''

YOLO_IOU_THRESHOLD = 0.1
'''A lower IoU will group more bounding-boxes together versus a higher IoU.
For this implementation, it is assumed that there is only single target, so
tune this value to provide a single bounding box.'''

LABEL_MAP = [
    "person",         "bicycle",    "car",           "motorbike",     "aeroplane",   "bus",           "train",
    "truck",          "boat",       "traffic light", "fire hydrant",  "stop sign",   "parking meter", "bench",
    "bird",           "cat",        "dog",           "horse",         "sheep",       "cow",           "elephant",
    "bear",           "zebra",      "giraffe",       "backpack",      "umbrella",    "handbag",       "tie",
    "suitcase",       "frisbee",    "skis",          "snowboard",     "sports ball", "kite",          "baseball bat",
    "baseball glove", "skateboard", "surfboard",     "tennis racket", "bottle",      "wine glass",    "cup",
    "fork",           "knife",      "spoon",         "bowl",          "banana",      "apple",         "sandwich",
    "orange",         "broccoli",   "carrot",        "hot dog",       "pizza",       "donut",         "cake",
    "chair",          "sofa",       "pottedplant",   "bed",           "diningtable", "toilet",        "tvmonitor",
    "laptop",         "mouse",      "remote",        "keyboard",      "cell phone",  "microwave",     "oven",
    "toaster",        "sink",       "refrigerator",  "book",          "clock",       "vase",          "scissors",
    "teddy bear",     "hair drier", "toothbrush"
]
TARGET_LABEL = 49
'''For this implementation, YOLO identifies the game piece as an orange'''

AStarPlanner = AStarPlanner()
'''Create the A* path planning object. This will initialize the field map and 
be used to plan paths from the robot's position to detected game pieces or 
other points of interest.'''


def ConnectToNetworkTables (timeout_s=600.0, server='10.46.07.2'):
    ''' This function will establish a connection to the network tables served
    by the robot RoboRIO.

    If the connection is successfully established, the function will simply
    return. If the connection cannot be established within the timeout period,
    the program will exit. The IP address for FRC is explained in detail here:
    https://docs.wpilib.org/en/stable/docs/networking/networking-introduction/ip-configurations.html

    Args:
        timeout_s (:obj:`float`): The length of time to wait for a connetion to
            be established.
        server (:obj:`string`): The IP address of the server.

    Returns:
        None
    '''
    cond = threading.Condition()
    notified = [False]

    def connectionListener( connected, info ):
        with cond:
            notified[0] = True
            cond.notify()

    NetworkTables.initialize( server=server )
    NetworkTables.addConnectionListener( connectionListener, immediateNotify=True )
    
    nt_connected = False
    with cond:
        if not notified[0]:
            nt_connected = cond.wait( timeout_s )
    if not nt_connected:
        sys.exit()


def ConnectToOakD (pipeline, num_retries=10):
    ''' This function will establish a connection to the OAK-D device.

    This function will attempt to to connect to the OAK-D device from the
    Raspberry PI over ethernet. If a connection cannot be made, then the
    program will exit. Otherwise, the connected device will be returned.

    Args:
        num_retries (:obj:`int`): The number of retries allowed while trying to
            connect to the device.

    Returns:
        device (:obj:`dai.Device`): The connected OAK-D device.
    '''
    retry_count = 0
    try: device = dai.Device( pipeline )
    except: device = None
    while device is None:
        if retry_count == 10:
            sys.exit()
        else:
            retry_count += 1
            print("Connecting to OAK-D, retry %i" % ( num_retries ) )
            try: device = dai.Device( pipeline )
            except: device = None

    return device


def CreatePipeline (nn_yolo_blob_path):
    ''' This function will create a pipeline to run on the OAK-D device.

    The pipeline created in this function is specific to using the tiny
    YOLO pretrained model from the depthAI repo. The color camera is used
    for object detection and the B&W cameras are used for stereo depth
    detections. Both of object inference and stereo depth calculations are
    fused together and sent to the Raspberry PI from the OAK-D.
    
    TODO: Currently, the only tuned parameter is the YOLO IoU threshold.
    Explore other parameters in an attempt to achieve reliable and robust
    target detection.

    Args:
        num_retries (:obj:`int`): The number of retries allowed while trying to
            connect to the device.

    Returns:
        pipeline (:obj:`dai.Pipeline`): The configured pipeline.
    '''
    # Start with an empty pipeline
    pipeline = dai.Pipeline()

    # Define the nodes used in the pipeline. The neural network will run on the
    # color camera and the the left and right B&W cameras are used for depth
    # perception.
    cam_rgb = pipeline.create( dai.node.ColorCamera )
    mono_left = pipeline.create( dai.node.MonoCamera )
    mono_right = pipeline.create( dai.node.MonoCamera )
    stereo = pipeline.create( dai.node.StereoDepth )
    spatial_detection_network = pipeline.create( dai.node.YoloSpatialDetectionNetwork )

    # Send this data to the host
    xout_rgb = pipeline.create( dai.node.XLinkOut )
    xout_nn = pipeline.create( dai.node.XLinkOut )
    xout_rgb.setStreamName( "rgb" )
    xout_nn.setStreamName( "detections" )

    # Configure the nodes
    cam_rgb.setPreviewSize( 416, 416 )
    cam_rgb.setResolution( dai.ColorCameraProperties.SensorResolution.THE_1080_P )
    cam_rgb.setInterleaved( False )
    cam_rgb.setColorOrder( dai.ColorCameraProperties.ColorOrder.BGR )
    mono_left.setResolution( dai.MonoCameraProperties.SensorResolution.THE_400_P )
    mono_left.setBoardSocket( dai.CameraBoardSocket.LEFT )
    mono_right.setResolution( dai.MonoCameraProperties.SensorResolution.THE_400_P )
    mono_right.setBoardSocket( dai.CameraBoardSocket.RIGHT )
    stereo.initialConfig.setConfidenceThreshold( 255 )
    spatial_detection_network.setBlobPath( nn_yolo_blob_path )
    spatial_detection_network.setConfidenceThreshold( 0.5 )
    spatial_detection_network.input.setBlocking( False )
    spatial_detection_network.setBoundingBoxScaleFactor( 0.5 )
    spatial_detection_network.setDepthLowerThreshold( 100 )
    spatial_detection_network.setDepthUpperThreshold( 5000 )
    spatial_detection_network.setNumClasses( len( LABEL_MAP ) )
    spatial_detection_network.setCoordinateSize( 4 )
    spatial_detection_network.setAnchors( np.array([10,14, 23,27, 37,58, 81,82, 135,169, 344,319]) )
    spatial_detection_network.setAnchorMasks({ "side26": np.array([1,2,3]), "side13": np.array([3,4,5]) })
    spatial_detection_network.setIouThreshold( YOLO_IOU_THRESHOLD )

    # Connect the node inputs/outputs
    mono_left.out.link( stereo.left )
    mono_right.out.link( stereo.right )
    cam_rgb.preview.link( spatial_detection_network.input )
    spatial_detection_network.passthrough.link( xout_rgb.input )
    spatial_detection_network.out.link( xout_nn.input )
    stereo.depth.link(spatial_detection_network.inputDepth )

    return pipeline


def CreateQueues (device):
    ''' This function will create the host queue's needed to capture the OAK-D
    data.

    This function will create the queue's for the outputs defined in the 
    'CreatePipeline' function. For an embedded implemention, the only data the
    OAK-D really needs to send to the host as the detection output from the 
    neural network. The color camera frames are also sent back to the host for
    debug and future work.

    TODO: It would be nice to be able to view the RGB frames and detection
    information at the drivers station. This would help debug and system
    bringup.
    https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/shuffleboard/getting-started/shuffleboard-displaying-camera.html

    Args:
        device (:obj:`dai.Device`): The connected OAK-D device.

    Returns:
        detection_nn_queue (:obj:`Queue`): The connected OAK-D device.
        rgb_queue (:obj:`Queue`): The connected OAK-D device.
    '''
    detection_nn_queue = device.getOutputQueue( name="detections", maxSize=4, blocking=False )
    rgb_queue = device.getOutputQueue( name="rgb", maxSize=4, blocking=False )
    return detection_nn_queue, rgb_queue


def PlanPath ( robot_x_in, robot_y_in, robot_heading_rad, depthai_x_in, depthai_z_in ):
    ''' This function will use the A* algorithm to plan a path from the robot
    to the detected target.

    The algorithm searches for an optimal path relative to the field coordinate
    system. This means the target location needs to first be transformed from
    the camera frame of reference to the field frame of reference. For this
    implementation, the camera frame of reference is the same as the robot's
    frame of reference (meaning a single transform is needed).

    When the path has been found, this function will return a list of waypoints
    which are relative to the field coordinate system. The list of waypoints
    doesn't contain the starting position of the robot as this will always be
    the first waypoint in the list.

    Args:
        robot_x_in (:obj:`float`): The robot's x-coordinate position on the
            field frame.
        robot_y_in (:obj:`float`): The robot's y-coordinate position on the
            field frame.
        robot_heading_rad (:obj:`float`): The robot's current heading.
        depthai_x_in (:obj:`float`): The target's x-coordinate position
            relative to the OAK-D camera frame.
        depthai_z_in (:obj:`float`): The target's x-coordinate position
            relative to the OAK-D camera frame.

    Returns:
        waypoints_x_robot_frame (:obj:`list of float`): The x-coordinates of
            the path relative to the field frame.
        waypoints_y_robot_frame (:obj:`list of float`): The y-coordinates of
            the path relative to the field frame.
    '''
    # The DepthAI coordinates are Z is positive away, X is positive to the
    # right, and Y is positive up...not quite right-hand rule...we can
    # invert the Y value to make it so. Assign the depthai spatial coordinates
    # and assign them to the camera frame (which is the same as our robot frame).
    target_x_in, target_y_in = depthai_z_in, depthai_x_in

    # Transform the target to the field frame. TODO: Fix the field reference
    # so that the mapper in the A* planner matches the FRC field convention.
    camera_field_frame = np.array(([robot_x_in, robot_y_in]))
    target_camera_frame = np.array(([target_x_in, target_y_in]))
    rotation_matrix = np.array( ( [ np.cos(robot_heading_rad), -np.sin(robot_heading_rad)],
                                  [ np.sin(robot_heading_rad),  np.cos(robot_heading_rad)], ) ) 
    _target_field_frame = rotation_matrix.dot( target_camera_frame ) + camera_field_frame
    target_field_frame = [_target_field_frame[1], _target_field_frame[0]]

    # Plan a path to the target in the field frame
    # print("Robot Location Field (x_in,y_in): (%.2f,%.2f)" % (camera_field_frame[0], camera_field_frame[1]))
    # print("Target Location Field (x_in,y_in): (%.2f,%.2f)" % (target_field_frame[0], target_field_frame[1]))
    waypoints_x_field_frame, waypoints_y_field_frame = AStarPlanner.plan_path( camera_field_frame, target_field_frame )
            
    # Transform the path to the target to the camera/robot frame
    waypoints_x_robot_frame = []
    waypoints_y_robot_frame = []
    for x, y in zip(waypoints_x_field_frame,waypoints_y_field_frame):
        wp_field_frame = np.array(([x, y]))
        wp_robot_frame = wp_field_frame - camera_field_frame
        waypoints_x_robot_frame.append(wp_robot_frame[0])
        waypoints_y_robot_frame.append(-wp_robot_frame[1])

    return waypoints_x_robot_frame, waypoints_y_robot_frame


if __name__ == '__main__':

    # Connect the networktables as a client. The robot will start and serve the
    # network tables. If the connection cannot be made within 10 minutes, this
    # script will exit. Otherwise, if the function returns, that means a
    # connection has been made and is ready to go. In that case, initialize
    # the date entries of this coprocessor.
    ConnectToNetworkTables()
    print("Connected to NetworkTables")
    cp_networktable = NetworkTables.getTable("Coprocessor")
    cp_networktable.putBoolean( "NT_Connected", True )
    cp_networktable.putBoolean( "DepthAI_Connected", False )
    cp_networktable.putBoolean( "Path_Valid", False )
    cp_networktable.putNumberArray( "Waypoints_x_in" , [0.0])
    cp_networktable.putNumberArray( "Waypoints_y_in" , [0.0])

    # TODO: Currently using a model provided with the OAK-D documentation.
    # This will very likely need to be updated for detecting objects specific
    # to the years game.
    nn_yolo_blob = str((Path(__file__).parent / Path('./models/yolo-v3-tiny-tf_openvino_2021.4_6shave.blob')).resolve().absolute())
    if not Path( nn_yolo_blob ).exists():
        sys.exit()

    # Connect to device with the configured pipeline. Once connected, create
    # the queues used to get the data from the ouputs of the pipeline.
    pipeline = CreatePipeline( nn_yolo_blob )
    device = ConnectToOakD ( pipeline )
    detection_nn_queue, _ = CreateQueues( device )
    print("Connected to OAK-D")
    cp_networktable.putBoolean( "DepthAI_Connected", True )

    # Initialize the moving average filters. These are necessary to reduce the
    # variation in the detections.
    moving_average_detection = MOVING_AVERAGE_WINDOW_SIZE*[0]
    moving_average_x = MOVING_AVERAGE_WINDOW_SIZE*[0]
    moving_average_y = MOVING_AVERAGE_WINDOW_SIZE*[0]
    moving_average_z = MOVING_AVERAGE_WINDOW_SIZE*[0]
    # # ----- USE FOR DEBUG ONLY -----
    # loop_times = []
    # loop_cnt = 0
    # while loop_cnt != 900: # ~30 seconds of data collection
    #     loop_cnt += 1
    # # ----- USE FOR DEBUG ONLY -----
    while True:
        detections = detection_nn_queue.get().detections
        found_target = False
        if len( detections ) != 0:
            for detection in detections:
                if detection.label == TARGET_LABEL:
                    # # ----- USE FOR DEBUG ONLY -----
                    # if np.mean( moving_average_detection ) == 0:
                    #     start_time = time.time()
                    # # ----- USE FOR DEBUG ONLY -----
                    found_target = True
                    moving_average_detection.pop()
                    moving_average_detection.insert( 0, 1 )
                    moving_average_x.pop()
                    moving_average_x.insert( 0, detection.spatialCoordinates.x * MM_TO_INCH )
                    moving_average_y.pop()
                    moving_average_y.insert( 0, detection.spatialCoordinates.y * MM_TO_INCH )
                    moving_average_z.pop()
                    moving_average_z.insert( 0, detection.spatialCoordinates.z * MM_TO_INCH )
                    # # ----- USE FOR DEBUG ONLY -----
                    # print("=====================================")
                    # print("Object: %s, Confidence: %0.2f, Location (x,y,z): (%0.2f,%0.2f,%0.2f)" % ( 
                    #     LABEL_MAP[detection.label],
                    #     detection.confidence*100,
                    #     detection.spatialCoordinates.x * MM_TO_FEET,
                    #     detection.spatialCoordinates.y * MM_TO_FEET,
                    #     detection.spatialCoordinates.z * MM_TO_FEET ) )
                    # # ----- USE FOR DEBUG ONLY -----
                    break

        if not found_target:
            moving_average_detection.pop()
            moving_average_detection.insert( 0, 0 )
            moving_average_x.pop()
            moving_average_x.insert( 0, 0 )
            moving_average_y.pop()
            moving_average_y.insert( 0, 0 )
            moving_average_z.pop()
            moving_average_z.insert( 0, 0 )

        elif np.mean( moving_average_detection ) == 1:

            # Get the starting pose from the network tables. The code running on the
            # robot will maintain this information.
            robot_x_in = cp_networktable.getNumber( "Robot_x_in" , 0.0 )
            robot_y_in = cp_networktable.getNumber( "Robot_y_in" , 0.0 )
            robot_heading_rad = cp_networktable.getNumber( "Robot_heading_rad" , 0.0 )
            depthai_x_in = np.mean( moving_average_x )
            depthai_z_in = np.mean( moving_average_z )

            # Plan a path to the target and send the wayponts to the main
            # processor via the network tables.    
            waypoints_x_in, waypoints_y_in = PlanPath( robot_x_in, robot_y_in, robot_heading_rad, depthai_x_in, depthai_z_in )
            cp_networktable.putNumberArray( "Waypoints_x_in" , waypoints_x_in )
            cp_networktable.putNumberArray( "Waypoints_y_in" , waypoints_y_in )           
            cp_networktable.putBoolean( "Path_Valid", True )

            # # ----- USE FOR DEBUG ONLY -----
            # robot_x_in, robot_y_in = 141, 222
            # depthai_z_in, depthai_x_in = 587, 20
            # robot_heading_rad = np.arctan((20 - 222) / (587 - 141))
            # waypoints_x_in, waypoints_y_in = PlanPath( robot_x_in, robot_y_in, robot_heading_rad, depthai_x_in, depthai_z_in )
            # dt = time.time() - start_time
            # loop_times.append([dt])
            # print("Loop time: %.6f" % ( dt ) )
            # moving_average_detection = MOVING_AVERAGE_WINDOW_SIZE*[0]
            # moving_average_x = MOVING_AVERAGE_WINDOW_SIZE*[0]
            # moving_average_y = MOVING_AVERAGE_WINDOW_SIZE*[0]
            # moving_average_z = MOVING_AVERAGE_WINDOW_SIZE*[0]
            # # ----- USE FOR DEBUG ONLY -----                        


    # # ----- USE FOR DEBUG ONLY -----
    # import csv
    # with open('loop_time.csv', mode='w') as out_file:
    #     writer = csv.writer(out_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
    #     writer.writerow(['loop_time(s)'])
    #     for entry in loop_times:
    #         writer.writerow(entry)
    # # ----- USE FOR DEBUG ONLY -----
