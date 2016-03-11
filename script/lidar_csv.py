import rospy
import sys
import csv

from geometry_msgs.msg import Vector3
from darc_saca.msg import PointList

def callback(data):
    n = len(data.points)
    file.write(str(n)+'\n')
    for point in data.points:
        #x = str(point.x)
        #y = str(point.y)
        file.write(str(point.x) + ',' + str(point.y) + '\n')
        
def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('segmented_points', PointList, callback)
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
    file = open('seg_'+name+'.csv', 'w')
    listener()
    file.close()
    
