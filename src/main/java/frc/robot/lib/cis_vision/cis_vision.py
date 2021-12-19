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


def ConnectToNetworkTables ( timeout_s = 600.0 ):
    '''
    '''
    cond = threading.Condition()
    notified = [False]

    def connectionListener( connected, info ):
        with cond:
            notified[0] = True
            cond.notify()

    NetworkTables.initialize( server='10.46.07.2' )
    NetworkTables.addConnectionListener( connectionListener, immediateNotify=True )
    
    nt_connected = False
    with cond:
        if not notified[0]:
            nt_connected = cond.wait( timeout_s )
    if not nt_connected:
        sys.exit()


def ConnectToOakD ( pipeline ):
    '''
    '''
    device = dai.Device( pipeline )
    return device


if __name__ == '__main__':

    # Connect the networktables as a client. The robot will start and serve the
    # network tables. If the connection cannot be made within 10 minutes, this
    # script will exit. Otherwise, if the function returns, that means a
    # connection has been made and is ready to go. In that case, initialize
    # the date entries of this coprocessor.
    ConnectToNetworkTables()
    cp_networktable = NetworkTables.getTable("Coprocessor")
    cp_networktable.putBoolean( "NT_Connected", True )
    cp_networktable.putString( "DepthAI_Status", "Connecting" )
    cp_networktable.putBoolean( "Path_Valid", False )
    cp_networktable.putNumberArray( "Waypoints_x_in" , [0.0])
    cp_networktable.putNumberArray( "Waypoints_y_in" , [0.0])

    # Create the A* path planning object. This will initialize the field map
    # and be used to plan paths from the robot's position to detected game
    # pieces or other points of interest.
    AStarPlanner = AStarPlanner()

    # TODO: Currently using a model provided with the OAK-D documentation.
    # This will very likely need to be updated for detecting objects specific
    # to the years game.
    nnBlobPath = str((Path(__file__).parent / Path('./models/yolo-v3-tiny-tf_openvino_2021.4_6shave.blob')).resolve().absolute())
    if not Path(nnBlobPath).exists():
        sys.exit()
    labelMap = [
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



    # Create the pipeline
    pipeline = dai.Pipeline()

    # Create the nodes
    mono_left = pipeline.create(dai.node.MonoCamera)
    mono_right = pipeline.create(dai.node.MonoCamera)
    stereo = pipeline.create( dai.node.StereoDepth )
    spatial_detection_network = pipeline.create( dai.node.YoloSpatialDetectionNetwork )

    # Define the data the OAK-D will send to the host
    xout_nn = pipeline.create( dai.node.XLinkOut )
    xout_nn.setStreamName( "detections" )

    # Properties
    mono_left.setResolution( dai.MonoCameraProperties.SensorResolution.THE_400_P )
    mono_left.setBoardSocket( dai.CameraBoardSocket.LEFT )
    mono_right.setResolution( dai.MonoCameraProperties.SensorResolution.THE_400_P )
    mono_right.setBoardSocket( dai.CameraBoardSocket.RIGHT )
    stereo.initialConfig.setConfidenceThreshold( 255 )
    spatial_detection_network.setBlobPath(nnBlobPath)
    spatial_detection_network.setConfidenceThreshold(0.5)
    spatial_detection_network.input.setBlocking(False)
    spatial_detection_network.setBoundingBoxScaleFactor(0.5)
    spatial_detection_network.setDepthLowerThreshold(100)
    spatial_detection_network.setDepthUpperThreshold(5000)
    spatial_detection_network.setNumClasses(80)
    spatial_detection_network.setCoordinateSize(4)
    spatial_detection_network.setAnchors(np.array([10,14, 23,27, 37,58, 81,82, 135,169, 344,319]))
    spatial_detection_network.setAnchorMasks({ "side26": np.array([1,2,3]), "side13": np.array([3,4,5]) })
    spatial_detection_network.setIouThreshold(0.5)

    # Linking
    mono_left.out.link( stereo.left )
    mono_right.out.link( stereo.right )
    stereo.depth.link( spatial_detection_network.inputDepth )
    spatial_detection_network.out.link( xout_nn.input )









    # # Create pipeline
    # pipeline = dai.Pipeline()

    # # Define sources and outputs
    # camRgb = pipeline.create(dai.node.ColorCamera)
    # spatialDetectionNetwork = pipeline.create(dai.node.YoloSpatialDetectionNetwork)
    # monoLeft = pipeline.create(dai.node.MonoCamera)
    # monoRight = pipeline.create(dai.node.MonoCamera)
    # stereo = pipeline.create(dai.node.StereoDepth)

    # xoutRgb = pipeline.create(dai.node.XLinkOut)
    # xoutNN = pipeline.create(dai.node.XLinkOut)
    # xoutBoundingBoxDepthMapping = pipeline.create(dai.node.XLinkOut)
    # xoutDepth = pipeline.create(dai.node.XLinkOut)

    # xoutRgb.setStreamName("rgb")
    # xoutNN.setStreamName("detections")
    # xoutBoundingBoxDepthMapping.setStreamName("boundingBoxDepthMapping")
    # xoutDepth.setStreamName("depth")

    # # Properties
    # camRgb.setPreviewSize(416, 416)
    # camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    # camRgb.setInterleaved(False)
    # camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

    # monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    # monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
    # monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    # monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)

    # # setting node configs
    # stereo.initialConfig.setConfidenceThreshold(255)

    # spatialDetectionNetwork.setBlobPath(nnBlobPath)
    # spatialDetectionNetwork.setConfidenceThreshold(0.5)
    # spatialDetectionNetwork.input.setBlocking(False)
    # spatialDetectionNetwork.setBoundingBoxScaleFactor(0.5)
    # spatialDetectionNetwork.setDepthLowerThreshold(100)
    # spatialDetectionNetwork.setDepthUpperThreshold(5000)

    # # Yolo specific parameters
    # spatialDetectionNetwork.setNumClasses(80)
    # spatialDetectionNetwork.setCoordinateSize(4)
    # spatialDetectionNetwork.setAnchors(np.array([10,14, 23,27, 37,58, 81,82, 135,169, 344,319]))
    # spatialDetectionNetwork.setAnchorMasks({ "side26": np.array([1,2,3]), "side13": np.array([3,4,5]) })
    # spatialDetectionNetwork.setIouThreshold(YOLO_IOU_THRESHOLD)

    # # Linking
    # monoLeft.out.link(stereo.left)
    # monoRight.out.link(stereo.right)

    # camRgb.preview.link(spatialDetectionNetwork.input)
    # spatialDetectionNetwork.passthrough.link(xoutRgb.input)

    # spatialDetectionNetwork.out.link(xoutNN.input)
    # spatialDetectionNetwork.boundingBoxMapping.link(xoutBoundingBoxDepthMapping.input)

    # stereo.depth.link(spatialDetectionNetwork.inputDepth)
    # spatialDetectionNetwork.passthroughDepth.link(xoutDepth.input)


    # Connect to device and start pipeline
    # device = ConnectToOakD ( pipeline )
    with dai.Device( pipeline ) as device:

        # Output queues will be used to get the rgb frames and nn data from the outputs defined above
        #previewQueue = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
        detectionNNQueue = device.getOutputQueue( name="detections", maxSize=4, blocking=False )
        #xoutBoundingBoxDepthMappingQueue = device.getOutputQueue(name="boundingBoxDepthMapping", maxSize=4, blocking=False)
        #depthQueue = device.getOutputQueue( name="depth", maxSize=4, blocking=False )

        cp_networktable.putBoolean( "DepthAI_Connected", True )
        detection_moving_average = 10*[0]
        while True:

            # Process the new detection(s)
            detections = detectionNNQueue.get().detections
            if len(detections) != 0:
                found_orange = False
                for detection in detections:
                    # try:
                    #     label = labelMap[detection.label]
                    # except:
                    #     label = detection.label
                    # print("Object: %s, Confidence: %0.2f, Location (x,y,z): (%0.2f,%0.2f,%0.2f)" % ( 
                    #     label,
                    #     detection.confidence*100,
                    #     detection.spatialCoordinates.x * 0.0032808399,
                    #     detection.spatialCoordinates.y * 0.0032808399,
                    #     detection.spatialCoordinates.z * 0.0032808399 ) )

                    if detection.label == 49:
                        found_orange = True
                        detection_moving_average.insert(0,1)
                        mm_to_ft = 0.0032808399
                        print("=====================================")
                        print("Object: %s, Confidence: %0.2f, Location (x,y,z): (%0.2f,%0.2f,%0.2f)" % ( 
                            labelMap[detection.label],
                            detection.confidence*100,
                            detection.spatialCoordinates.x * mm_to_ft,
                            detection.spatialCoordinates.y * mm_to_ft,
                            detection.spatialCoordinates.z * mm_to_ft ) )

                        break
                if not found_orange:
                    detection_moving_average.insert(0,1)

            else:
                detection_moving_average.insert(0,1)

            # # The DepthAI coordinates are Z is positive away, X is positive to the
            # # right, and Y is positive up...not quite right-hand rule...we can
            # # invert the Y value to make it so.
            # # If the moving average is > 50%, then plan a path
            # if np.mean( detection_moving_average ):

            #     # Use a default starting pose until the pose estimator with vision
            #     # is ready to go.
            #     current_x, current_y, current_theta_rad = 4*12, 3*12, np.radians( 45.0 ) # pose_estimator.get_pose()
                
            #     # Get the spatial coordinates and assign them to the camera frame
            #     # (which is the same as our robot frame).
            #     mm_to_inch = 0.0393700787
            #     depthai_x, depthai_z = detection.spatialCoordinates.x * mm_to_inch, detection.spatialCoordinates.z * mm_to_inch
            #     target_x, target_y = depthai_z, depthai_x

            #     # Transform the target to the field frame
            #     camera_field_frame = np.array(([current_x, current_y]))
            #     target_camera_frame = np.array(([target_x, target_y]))
            #     rotation_matrix = np.array( ( [ np.cos(current_theta_rad), -np.sin(current_theta_rad)],
            #                                 [ np.sin(current_theta_rad),  np.cos(current_theta_rad)], ) ) 
            #     _target_field_frame = rotation_matrix.dot( target_camera_frame ) + camera_field_frame
            #     target_field_frame = [_target_field_frame[1], _target_field_frame[0]]

            #     # Plan a path to the target in the field frame.
            #     # TODO: fix the planner coordinate system
            #     print("Robot Location Field (x_in,y_in): (%.2f,%.2f)" % (camera_field_frame[0], camera_field_frame[1]))
            #     print("Target Location Field (x_in,y_in): (%.2f,%.2f)" % (target_field_frame[0], target_field_frame[1]))
            #     waypoints_x_field_frame, waypoints_y_field_frame = Planner.plan_path( camera_field_frame, target_field_frame )
                
            #     # Transform the path to the target to the camera/robot frame
            #     waypoints_x_robot_frame = []
            #     waypoints_y_robot_frame = []
            #     for x, y in zip(waypoints_x_field_frame,waypoints_y_field_frame):
            #         wp_field_frame = np.array(([x, y]))
            #         wp_robot_frame = wp_field_frame - camera_field_frame
            #         waypoints_x_robot_frame.append(wp_robot_frame[0])
            #         waypoints_y_robot_frame.append(-wp_robot_frame[1])

            #     print(waypoints_x_robot_frame)
            #     print(waypoints_y_robot_frame)
            #     cp_networktable.putNumberArray( "Waypoints_x_in" , waypoints_x_robot_frame)
            #     cp_networktable.putNumberArray( "Waypoints_y_in" , waypoints_y_robot_frame)           
            #     cp_networktable.putBoolean( "Path_Ready", True )
            # else:
            #     cp_networktable.putBoolean( "Path_Ready", False )

    cp_networktable.putString( "DepthAI_Status", "Connecting" )