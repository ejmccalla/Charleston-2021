import cv2

'''
This file is used purely for visualizing the path planning and not needed for
the actual implementation running on the robot. The field_img_bgr variable
defines the path to an image of the actual field. This can be found in the
game documents. The field_img_offsets_in defines upper-left and lower-right
locations of the playing field locations. This defines the bounding rectangle
of the playing field.
'''

field_img_bgr = cv2.imread( './fields/field_image_shed.png' )
field_img_offsets_in = [ ( 0.0, 0.0 ), ( 960.0, 700.0 )]
