import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # Replace with your specific message types
from yolov8_msgs.msg import Yolov8Inference
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan

import numpy as np
import math

from rclpy.qos import qos_profile_sensor_data

list_class = ["cat", "dog", "chair", "car", "apple", "stop sign"]
list_pos = [] #Contains [x,y]
dict_count = {"cat": 0, "dog": 0, "chair": 0, "car": 0, "apple": 0, "stop sign": 0} #Contains "cat": 1
item_list_pos = {"cat": [], "dog": [], "chair": [], "car": [], "apple": [], "stop sign": []}

def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    return yaw_z

#Given detected class and its position on the map, add to the count if it's not too close to existing stored positions
def add_to_count(class_name, position, limit_distance):
    print("spotted a: " + class_name)
    if (class_name in list_class):
        eligible = True
        #If list_pos is empty, make the first entry eligible
        if not item_list_pos[class_name]:
            print("list is empty")
            eligible = True
        for i in item_list_pos[class_name]:
            #if true keep going in the for loop
            #if false for all, eligible = True and break out
            if ((((position[0]-i[0])**2)+((position[1]-i[1])**2))<limit_distance):
                eligible = False
                #try this out
                item_list_pos[class_name].append(position)
                break
        if (eligible):
            dict_count[class_name] += 1
            item_list_pos[class_name].append(position)
            print(class_name + " added")
            print(dict_count)
            print(list_pos)


#Takes the diff angles and wall, and draws a line from the robot at that angle, sending the location when it hits a wall
def bresenhams_line_algorithm(theta, gamma, dist, wall, robot_column, robot_row):
    #print("bla just ran")
    beta = theta - gamma

    while (beta > 2*math.pi):
        beta -= 2*math.pi
    while (beta < 0):
        beta += 2*math.pi  
    xdist = abs(dist*math.cos(beta))
    ydist = abs(dist*math.sin(beta))
    c = robot_column
    r = robot_row
    print("C AND R AND BETA ARE")
    print(c)
    print(r)
    print(beta)
    i = 0
    j = 0
    if (beta <= math.pi/4):
        while (i<xdist):
            i += 1
            c += 1
            if (abs(c*math.tan(beta))<abs(r+1)):
                if ([c-1,r] in wall):
                    return [c-1,r]
            else:
                if ([c-1,r] in wall):
                    return [c-1,r]
                c -= 1
                if ([c-1,r] in wall):
                    return [c-1,r]
    elif (beta <= math.pi/2):
        while (i<ydist):
            i += 1
            r += 1
            if (abs(r/math.tan(beta))<abs(c+1)):
                if ([c,r-1] in wall):
                    return [c,r-1]
            else:
                if ([c,r-1] in wall):
                    return [c,r-1]
                c += 1
                if ([c,r-1] in wall):
                    return [c,r-1]
    elif (beta <= math.pi*3/4):
        while (i<ydist):
            i += 1
            r += 1
            if (abs(r/math.tan(beta))<abs(c-1)):
                if ([c,r-1] in wall):
                    return [c,r-1]
            else:
                if ([c,r-1] in wall):
                    return [c,r-1]
                c += 1
                if ([c,r-1] in wall):
                    return [c,r-1]
    elif (beta <= math.pi):
        while (i<abs(xdist)):
            i += 1
            c -= 1
            if (abs(c*math.tan(beta))<abs(r+1)):
                if ([c+1,r] in wall):
                    return [c+1,r]
            else:
                if ([c+1,r] in wall):
                    return [c+1,r]
                r += 1
                if ([c+1,r] in wall):
                    return [c+1,r]
    elif (beta <= math.pi*5/4):
        while (i<abs(xdist)):
            i += 1
            c -= 1
            if (abs(c*math.tan(beta))<abs(r-1)):
                if ([c+1,r] in wall):
                    return [c+1,r]
            else:
                if ([c+1,r] in wall):
                    return [c+1,r]
                r -= 1
                if ([c+1,r] in wall):
                    return [c+1,r]
    elif (beta <= math.pi*3/2):
        while (i<abs(ydist)):
            i += 1
            r -= 1
            if (abs(r/math.tan(beta))<abs(c-1)):
                if ([c,r+1] in wall):
                    return [c,r+1]
            else:
                if ([c,r+1] in wall):
                    return [c,r+1]
                c -= 1
                if ([c,r+1] in wall):
                    return [c,r+1]
    elif (beta <= math.pi*7/4):
        while (i<abs(ydist)):
            i += 1
            r -= 1
            if (abs(r/math.tan(beta))<abs(c+1)):
                if ([c,r+1] in wall):
                    return [c,r+1]
            else:
                if ([c,r+1] in wall):
                    return [c,r+1]
                c += 1
                if ([c,r+1] in wall):
                    return [c,r+1]
    elif (beta <= math.pi*2):
        while (i<abs(xdist)):
            i += 1
            c += 1
            if (abs(c*math.tan(beta))<abs(r-1)):
                if ([c-1,r] in wall):
                    return [c-1,r]
            else:
                if ([c-1,r] in wall):
                    return [c-1,r]
                r -= 1
                if ([c-1,r] in wall):
                    return [c-1,r]
    else:
        print("Angle greater than 2pi error")
    return None

class Class_Counter(Node):
    def __init__(self):
        super().__init__('class_counter')
        
        self.yolo_subscription = self.create_subscription(
            Yolov8Inference, '/Yolov8_Inference', self.yolo_callback, 10)
        self.yolo_subscription = self.create_subscription(
            OccupancyGrid, 'map', self.map_callback, 10)
        self.yolo_subscription = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)
        self.yolo_subscription = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, qos_profile_sensor_data)
        self.yolo_subscription

        # Variables to store latest messages
        self.latest_yolo_msg = None
        self.odom_msg = None
        self.odom_x = None
        self.odom_y = None
        self.odom_yaw = None
        self.map_msg = None
        self.map_resolution = None
        self.map_originX = None
        self.map_originY = None
        self.map_width = None
        self.map_height = None
        self.map_data = None
        self.map_pose_origin = None
        self.map_yaw = None

    def yolo_callback(self, data):
        latest_yolo_msg = data
        for r in latest_yolo_msg.yolov8_inference:
            class_name = r.class_name
            top = r.top
            left = r.left
            bottom = r.bottom
            right = r.right
            #self.get_logger().info(
            #    f"Received YOLO detection: "
            #    f"class_name={class_name}, top={top}, left={left}, bottom={bottom}, right={right}"
            #)
            self.perform_logic(class_name, top, left, bottom, right)
    
    def map_callback(self, msg):
        self.map_msg = msg
        self.map_resolution = self.map_msg.info.resolution
        self.map_originX = self.map_msg.info.origin.position.x
        self.map_originY = self.map_msg.info.origin.position.y
        self.map_width = self.map_msg.info.width
        self.map_height = self.map_msg.info.height
        self.map_data = self.map_msg.data
        self.map_yaw = euler_from_quaternion(self.map_msg.info.origin.orientation.x, self.map_msg.info.origin.orientation.y, self.map_msg.info.origin.orientation.z, self.map_msg.info.origin.orientation.w)


    def odom_callback(self, msg):
        self.odom_msg = msg
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        self.odom_yaw = euler_from_quaternion(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                                              msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        
        #print("yaw: " + str(self.odom_yaw))
        #print("odom_x: " + str(self.odom_x))
        #print("odom_y: " + str(self.odom_y))

    def scan_callback(self, msg):
        self.scan_msg = msg
        self.ranges = msg.ranges
        self.angle_increment = msg.angle_increment

    def perform_logic(self, class_name, top, left, bottom, right):
        picture_y = (top+bottom)/2
        picture_x = (left+right)/2#Consider adding check to avoid extra chair detection
        if (picture_y>65):
            angle = (picture_x-160)*(1.204)/(320.0) #convert location on screen to angle (roughly)
            #distance = self.ranges[min(round((self.odom_yaw+math.pi)/self.angle_increment),len(self.ranges)-1)]
            #print(min(round((self.odom_yaw+math.pi)/self.angle_increment),len(self.ranges)-1))
            distance = self.ranges[0]
            position = [self.odom_x+distance*math.cos(angle), self.odom_y+distance*math.sin(angle)]
            limit_distance = 3
            #print(position)
            #print(distance)
            add_to_count(class_name, position, limit_distance) #add somedistance, can divide by map resolution also


def main(args=None):
    rclpy.init(args=args)
    node = Class_Counter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
