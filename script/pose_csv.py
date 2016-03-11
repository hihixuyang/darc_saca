import rospy
import sys
import csv

from geometry_msgs.msg import Pose

def quad_callback(data):
        
def base_callback(data):

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('quad/pose', Pose, quad_callback)
    rospy.Subscriber('base/pose', Pose, base_callback)
    rospy.spin()
    
if __name__ == '__main__':
    quad_file = open('quad.csv', 'w')
    base_file = open('base.csv', 'w')
    listener()
    file.close()
    
