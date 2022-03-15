#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations

import math

pub_ = None
LIDAR = ""

regions_ = {
    'right': 0,
    'fright': 0,
    'front': 0,
    'fleft': 0,
    'left': 0,
    'bright':0,
}
state_ = 0
state_dict_ = {
    0: 'find the wall',
    1: 'turn left',
    2: 'follow the wall',
    3: 'follow corner',
}

def clbk_laser(msg):
    global regions_, LIDAR

    newRange = []
    for val in msg.ranges:
        if (val == 0):
            newRange.append(3)
        else:
            newRange.append(val)

    if LIDAR == "RP":
	    regions_ = {
		'left':  min(min(newRange[539:541]), 3),
		'fleft': min(min(newRange[421:538]), 3),
		'front':  min(min(newRange[300:420]), 3),
		'fright':  min(min(newRange[182:300]), 3),
		'right':   min(min(newRange[179:181]), 3),
		'bright':   min(min(newRange[60:178]), 3),
        'bleft':   min(min(newRange[541:600]), 3),
	    }
	    print ("LIDAR: RP")
    else:
	    regions_ = {
		'left':  min(min(newRange[179:181]), 3),
		'fleft': min(min(newRange[60:178]), 3),
		'front':  min(min(newRange[661:719]),min(newRange[0:59]), 3),
		'fright':  min(min(newRange[542:660]), 3),
		'right':   min(min(newRange[539:541]), 3),
		'bright':   min(min(newRange[420:538]), 3),
        'bleft':   min(min(newRange[181:300]), 3),
	    }
	    print ("LIDAR: YD")
 
    print ("front distance: "+ str(regions_["front"]))
    print ("front-right distance: "+ str(regions_["fright"]))
    print ("right distance: "+ str(regions_["right"]))
    print ("back-right distance: "+ str(regions_["bright"]))

    take_action()


def change_state(state):
    global state_, state_dict_
    if state is not state_:
        print ('Wall follower - [%s] - %s' % (state, state_dict_[state]))
        state_ = state


def take_action():
    global regions_
    regions = regions_
    msg = Twist()
    linear_x = 0
    angular_z = 0

    state_description = ''

    d = 0.3

    if regions['front'] > d and regions['fleft'] > (d+0.2) and regions['bleft'] > d:
        state_description = 'case 1 - nothing'
        change_state(0)
    elif regions['front'] < d and regions['fleft'] > d:
        state_description = 'case 2 - front'
        change_state(1)
    elif regions['front'] < d and regions['fleft'] < d:
        state_description = 'case 2 - front & fright'
        change_state(1)
    elif regions['front'] > d and regions['fleft'] < d:
        state_description = 'case 3 - fright'
        change_state(2)
    elif regions['front'] > d and regions['left'] < d:
        state_description = 'case 4 - right'
        change_state(2)
    elif regions['front'] > d and regions['bleft'] < d:
        state_description = 'case 5 - bright'
        change_state(3)
    elif regions['front'] > d and regions['left'] > d:
        state_description = 'case 5 - right too far'
        change_state(3)    
    elif regions['front'] > d and regions['bleft'] > d and regions['fleft'] < (d+0.2):
        state_description = 'case 5 - close fright'
        change_state(3)
    else:
        state_description = 'unknown case'
        rospy.loginfo('unknown case')


def find_wall():
    msg = Twist()
    msg.linear.x = 0.05
    msg.angular.z = 0.3
    return msg


def turn_left():
    msg = Twist()
    msg.angular.z = -0.3
    return msg


def follow_the_wall():
    msg = Twist()
    msg.linear.x = 0.05
    msg.angular.z = 0.05
    return msg

def follow_corner():
    msg = Twist()
    msg.linear.x = 0.05
    msg.angular.z = 0.2
    return msg

def main():
    global pub_, LIDAR

    rospy.init_node('wall_follow')
    LIDAR = rospy.get_param("~LIDAR")
    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    sub = rospy.Subscriber('/scan', LaserScan, clbk_laser)
    
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        msg = Twist()
        if state_ == 0:
            msg = find_wall()
        elif state_ == 1:
            msg = turn_left()
        elif state_ == 2:
            msg = follow_the_wall()
        elif state_ == 3:
            msg = follow_corner()
            pass
        else:
            rospy.logerr('Unknown state!')

        pub_.publish(msg)

        rate.sleep()


if __name__ == '__main__':
    main()
