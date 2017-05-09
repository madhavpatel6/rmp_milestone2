#!/usr/bin/env python
#from __future__ import division
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf
import math
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Int32

'''
def getXY(node_number):
    node_number-=1
    grid_size_x = rospy.get_param('/grid_size_x') +1
    grid_size_y = rospy.get_param('/grid_size_y') +1
    grid_step = rospy.get_param('/grid_step')
    goalX=node_number/(grid_size_y*grid_step)
    goalY=node_number%(grid_size_y*grid_step)
    return goalX,goalY
real_size_x = grid_size_x + ((grid_size_x - 1)*(1/grid_step - 1))
real_size_y = grid_size_y + ((grid_size_y - 1)*(1/grid_step - 1))
print('Additional', (grid_size_x*(1/grid_step - 1)), (grid_size_y*(1/grid_step - 1)))
print("Grid Size (%d, %d)" % (grid_size_x, grid_size_y))
print('Real Size (%d, %d)' % (real_size_x, real_size_y))

def convert_vertex_to_xy(v):
    x = math.floor(((v - 1) / real_size_y)) * grid_step - math.floor(grid_size_x / 2.0) + 1 - grid_size_x % 2
    y = ((v - 1) % real_size_y) * grid_step - math.floor(grid_size_y / 2.0) + 1 - grid_size_y % 2
    return x, y

def convert_xy_to_vertex(x, y):
    dx = x + math.floor(grid_size_x / 2.0) - 1 + grid_size_x % 2
    dy = y + math.floor(grid_size_y / 2.0) - 1 + grid_size_y % 2
    return int(1 + (dx * real_size_y)/grid_step + dy/grid_step)
    '''


def getXY(node_number):
    grid_size_x = rospy.get_param('/grid_size_x')
    grid_size_y = rospy.get_param('/grid_size_y')
    grid_step = rospy.get_param('/grid_step')
    real_size_x = grid_size_x + ((grid_size_x - 1) * (1 / grid_step - 1))
    real_size_y = grid_size_y + ((grid_size_y - 1) * (1 / grid_step - 1))
    x = math.floor(((node_number - 1) / real_size_y)) * grid_step - math.floor(grid_size_x / 2.0) + 1 - grid_size_x % 2
    y = ((node_number - 1) % real_size_y) * grid_step - math.floor(grid_size_y / 2.0) + 1 - grid_size_y % 2
    return x, y

def callback(data):
    print("callback called")
    goalX,goalY=getXY(data.data)
    kP = 0.75
    tolerence = rospy.get_param('/radius')
    print 'Goal position %d -> (%f, %f)' % (int(data.data), goalX, goalY)
    p = rospy.Publisher('/cmd_vel', Twist)
    while(True):
        odom = rospy.wait_for_message('/odom', Odometry)
        twist = Twist()
        pos = odom.pose.pose.position
        ori = odom.pose.pose.orientation
        dist = math.sqrt((pos.x - goalX) ** 2 + (pos.y - goalY) ** 2)
        quaternion = [ori.x, ori.y, ori.z, ori.w]
        angle = (tf.transformations.euler_from_quaternion(quaternion)[2]) * 180 / math.pi
        ori_vector = [math.cos(angle * math.pi / 180), math.sin(angle * math.pi / 180)]
        point_vector = [goalX - pos.x, goalY - pos.y]
        dot = point_vector[0] * ori_vector[0] + point_vector[1] * ori_vector[1]
        mag_ori = math.sqrt(ori_vector[0] ** 2 + ori_vector[1] ** 2)
        mag_point = math.sqrt(point_vector[0] ** 2 + point_vector[1] ** 2)
        angle_between = math.acos(dot / (mag_ori * mag_point)) * 180 / math.pi
        angle_sign = ori_vector[0] * point_vector[1] - ori_vector[1] * point_vector[0]
        angle_between = math.copysign(angle_between, angle_sign)
        # rospy.loginfo('Robot distance = %.2f angle = %.2f', dist, angle)
        # rospy.loginfo('Angle between = %.2f', angle_between)
        # The case where we are at the point
        if dist < tolerence:
            rospy.loginfo("Successfully moved to (%f, %f) Robots real position (%.2f, %.2f)", goalX, goalY, pos.x, pos.y)
            twist = Twist()
            p.publish(twist)
            return
        # This is the case where the angle is not alligned
        elif abs(angle_between) > 3:
            twist = Twist()
            speed = abs(0.05 * angle_between)
            if speed > 0.2:
                speed = 0.2
            if angle_between > 0:
                twist.angular.z = speed
            else:
                twist.angular.z = -1 * speed
            p.publish(twist)
        # Else we keep moving forward
        else:
            twist = Twist()
            twist.linear.x = 0.2
            p.publish(twist)
        '''
        msg= rospy.wait_for_message('/odom', Odometry)
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        currX = msg.pose.pose.position.x
        currY = msg.pose.pose.position.y
        print "============CURRENT POSITION==========="
        print(currX)
        print(currY)
        twist=Twist()
        heading  = math.atan2((goalY - currY),(goalX - currX))
        error = kP * (heading - yaw)
        errDist = math.sqrt((goalX - currX)*(goalX - currX)+(goalY - currY)*(goalY - currY))
        if errDist < tolerence:
            print "GOAL REACHED"
            twist.angular.z = 0
            twist.linear.x = 0
            p.publish(twist)
            return
        twist.angular.z = error
        twist.linear.x = 0.1
        p.publish(twist)
        rospy.loginfo("Moving")
        rospy.sleep(0.1)'''

if __name__ == '__main__':
    rospy.init_node('move_robot')
    print("initialized")

    while not rospy.is_shutdown():
        rospy.Subscriber("/goal", Int32, callback, queue_size=1)
        rospy.sleep(5)




