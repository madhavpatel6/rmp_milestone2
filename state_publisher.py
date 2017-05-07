#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf
import math
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud

def checkdir(goalX,goalY):
    msg= rospy.wait_for_message('/my_cloud', PointCloud)
    od= rospy.wait_for_message('/odom', Odometry)
    grid_step = rospy.get_param('/grid_step')
    currX = int(round(od.pose.pose.position.x))
    currY = int(round(od.pose.pose.position.y))
    if(goalX-currX==grid_step):
        xStep=float(currX)+0.1
        while(xStep<=goalX):
            for pt in msg.points:
                scan=np.array((pt.x,pt.y))
                rob=np.array((currX,currY))
                dist_from_rob=np.linalg.norm(rob-scan)
                step=np.array((xStep,currY))
                dist=np.linalg.norm(step-scan)
                if(dist<0.2 and dist_from_rob<1.5):
                    return np.inf
            xStep+=0.1
    if(goalY-currY==grid_step):
        yStep=float(currY)+0.1
        while(yStep<=goalY):
            for pt in msg.points:
                scan=np.array((pt.x,pt.y))
                rob=np.array((currX,currY))
                dist_from_rob=np.linalg.norm(rob-scan)
                step=np.array((currX,yStep))
                dist=np.linalg.norm(step-scan)
                if(dist<0.2 and dist_from_rob<1.5):
                    return np.inf
            yStep+=0.1
    if(goalX-currX==-grid_step):
        xStep=float(currX)-0.1
        while(xStep>=goalX):
            for pt in msg.points:
                scan=np.array((pt.x,pt.y))
                rob=np.array((currX,currY))
                dist_from_rob=np.linalg.norm(rob-scan)
                step=np.array((xStep,currY))
                dist=np.linalg.norm(step-scan)
                if(dist<0.2 and dist_from_rob<1.5):
                    return np.inf
            xStep-=0.1
    if(goalY-currY==-grid_step):
        yStep=float(currY)-0.1
        while(yStep>=goalY):
            for pt in msg.points:
                scan=np.array((pt.x,pt.y))
                rob=np.array((currX,currY))
                dist_from_rob=np.linalg.norm(rob-scan)
                step=np.array((currX,yStep))
                dist=np.linalg.norm(step-scan)
                if(dist<0.2 and dist_from_rob<1.5):
                    return np.inf
            yStep-=0.1
    return 1

'''
def getNodeNumber(currX,currY):
    grid_size_x = rospy.get_param('/grid_size_x') +1
    grid_size_y = rospy.get_param('/grid_size_y') +1
    grid_step = rospy.get_param('/grid_step')
    node_number = 1
    node_number += (currX*grid_size_y)/grid_step
    node_number += currY/grid_step
    return node_number'''

def getNodeNumber(currX, currY):
    grid_size_x = rospy.get_param('/grid_size_x') + 1
    grid_size_y = rospy.get_param('/grid_size_y') + 1
    grid_step = rospy.get_param('/grid_step')
    dx = currX + math.floor(float(grid_size_x) / 2.0) - 1 + grid_size_x % 2
    dy = currY + math.floor(float(grid_size_y) / 2.0) - 1 + grid_size_y % 2
    return int(1 + (dx * grid_size_y) + dy)

def check_bounds(x, y, grid_size_x, grid_size_y):
    x_min = - math.floor(grid_size_x / 2.0) + 1 - grid_size_x % 2
    x_max = math.floor(grid_size_x / 2.0)
    y_min = - math.floor(grid_size_y / 2.0) + 1 - grid_size_y % 2
    y_max = math.floor(grid_size_y / 2.0)
    return x_min <= x <= x_max and y_min <= y <= y_max

if __name__ == '__main__':
    rospy.init_node('dstar')
    print("initialized")
    rad = rospy.get_param('/radius')


    while not rospy.is_shutdown():
        grid_size_x = rospy.get_param('/grid_size_x') + 1
        grid_size_y = rospy.get_param('/grid_size_y') + 1
        grid_step = rospy.get_param('/grid_step')
        x_min = - math.floor(grid_size_x / 2.0) + 1 - grid_size_x % 2
        x_max = math.floor(grid_size_x / 2.0)
        y_min = - math.floor(grid_size_y / 2.0) + 1 - grid_size_y % 2
        y_max = math.floor(grid_size_y / 2.0)
        print("Bounds (%d, %d) x (%d, %d)" % (x_min, x_max, y_min, y_max))
        msg= rospy.wait_for_message('/odom', Odometry)
        currX = int(round(msg.pose.pose.position.x))
        currY = int(round(msg.pose.pose.position.y))

        ActX=msg.pose.pose.position.x
        ActY=msg.pose.pose.position.y

        node_position=np.array((currX,currY))
        rob_position=np.array((ActX,ActY))
        dist_from_node=np.linalg.norm(rob_position-node_position)
        if(dist_from_node<=rad):
            current_node_publisher = rospy.Publisher('/current_node', String, queue_size=1)
            edge_cost_publisher = rospy.Publisher('/edge_costs', String, queue_size=1)
            edge_costs=""
            #edge_costs+="\n"
            rospy.sleep(0.1)
            dir=0
            node_number = getNodeNumber(currX, currY)
            print("Current node number: %d -> %d, %d" % (node_number, currX, currY))
            while(dir<4):
                ret=0
                if(dir==0 and check_bounds(currX, currY+1, grid_size_x, grid_size_y)):
                    ret=checkdir(currX,currY+1)
                    goalX=currX
                    goalY=currY+1
                elif(dir==1 and check_bounds(currX+1, currY, grid_size_x, grid_size_y)):
                    ret=checkdir(currX+1,currY)
                    goalX=currX+1
                    goalY=currY
                elif(dir==2 and check_bounds(currX, currY-1, grid_size_x, grid_size_y)):
                    ret=checkdir(currX,currY-1)
                    goalX=currX
                    goalY=currY-1
                elif(dir==3 and check_bounds(currX-1, currY, grid_size_x, grid_size_y)):
                    ret=checkdir(currX-1,currY)
                    goalX=currX-1
                    goalY=currY
                if(ret != 0):
                    edge_costs+=str(getNodeNumber(currX,currY))
                    edge_costs+="	"
                    edge_costs+=str(getNodeNumber(goalX,goalY))
                    edge_costs+="	"
                    edge_costs+=str(ret)
                    edge_costs+="\n"
                dir+=1
            rospy.sleep(1.0)
            current_node_publisher.publish(str(node_number))
            edge_cost_publisher.publish(edge_costs)
        else:
            print("robot too far from node")
            rospy.sleep(0.1)




