import rospy
import sys
import csv
import math

from geometry_msgs.msg import Vector3
from darc_saca.msg import PointList
from sensor_msgs.msg import LaserScan

def scan_callback(data):
    n = len(data.ranges)
    scan_file.write(str(n)+'\n')
    for val in data.ranges:
        scan_file.write(str(val)+'\n')

    raw_file.write(str(n)+'\n')
    #theta_rel = data.angle_min
    theta_rel = -math.pi / 4.0
    for val in data.ranges:
        if val <= 6.0:
            raw_file.write(str(val*math.cos(theta_rel)) + ', ')
            raw_file.write(str(-val*math.sin(theta_rel)) + '\n')
        theta_rel += data.angle_increment;

def seg_callback(data):
    n = len(data.points)
    seg_file.write(str(n)+'\n')
    for point in data.points:
        seg_file.write(str(point.x) + ',' + str(point.y) + '\n')
        
def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('scan', LaserScan, scan_callback)
    rospy.Subscriber('segmented_points', PointList, seg_callback)
    rospy.spin()
    
if __name__ == '__main__':
    total_args = len(sys.argv)
    if (total_args <= 1) : 
      print 'Enter file name'
      name = ''
      exit
    elif (total_args > 3) : 
        print 'Too Many Args'
        name = ''
        exit
    else:
        name = str(sys.argv[1])
    scan_file = open('scan_' + name + '.csv', 'w')
    raw_file = open('raw_' + name + '.csv', 'w')
    seg_file = open('seg_' + name + '.csv', 'w')

    listener()

    scan_file.close()
    raw_file.close()
    seg_file.close()
    
