import math
import rospy

import numpy as np

from cv_bridge import CvBridge, CvBridgeError


def angular_distance(position : tuple, target_position : tuple, degrees : bool) -> float:
    """Calculate the angular distance between origin and target position

    Args:
        position (tuple): (x, y, z) current position.
        yaw (float): current yaw.
        target_position (tuple): (x, y, z) target position.
        degrees: Define if radian's or degree's at result type 

    Returns:
        float: distance
    """    
    x, y, _= position
    tx, ty, _= target_position

    to= np.arctan2(ty - y, tx - x)

    heading= to - 0
    if heading > math.pi:
        heading -= 2 * math.pi

    elif heading < -math.pi:
        heading += 2 * math.pi

    return heading if not degrees else np.degrees(heading)

def image_transport(img_msg):
        rospy.logwarn(img_msg.header)
        try:
            return CvBridge().imgmsg_to_cv2(img_msg, "passthrough")

        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
            

