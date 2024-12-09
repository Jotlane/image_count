import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # Replace with your specific message types
from yolov8_msgs.msg import Yolov8Inference
from nav_msgs.msg import OccupancyGrid, Odometry

import numpy as np
import math

list_class = ["cat", "dog", "chair", "car", "apple", "stop sign"]
list_pos = [] #Contains [x,y]
dict_count = {} #Contains "cat": 1


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
def add_to_count(class_name, position, somedistance):
    if (class_name in list_class):
        eligible = False
        for i in list_pos:
            #(((position_x-i[0])^2)+((position_y-i[1])^2))<somedistance
            #if the above is true keep going in the for loop
            #if the above is false, eligible = True and break out
            if ((((position[0]-i[0])^2)+((position[1]-i[1])^2))>somedistance):
                eligible = True
                break
        if (eligible == True):
            dict_count[class_name] += 1
            list_pos.apppend(position)
            print(class_name + " added")
            print(dict_count)


#Takes the diff angles and wall, and calculates whether 
def bresenhams_line_algorithm(alpha, theta, gamma, dist, wall, robot_x, robot_y):
    beta = alpha +theta + gamma #Assuming beta is a number from 0 to 359.999
    xdist = dist*math.cos(beta)
    ydist = dist*math.sin(beta)
    x = robot_x
    y = robot_y
    mark = []
    if (beta <= math.pi/4):
        while (x<xdist):
            x += 1
            if (x*math.tan(beta)<y+1):
                if ([x-1,y] in wall):
                    return [x-1,y]
            else:
                if ([x-1,y] in wall):
                    return [x-1,y]
                y += 1
                if ([x-1,y] in wall):
                    return [x-1,y]
    elif (beta <= math.pi/2):
        while (y<ydist):
            y += 1
            if (y/math.tan(beta)<x+1):
                if ([x,y-1] in wall):
                    return [x,y-1]
            else:
                if ([x,y-1] in wall):
                    return [x,y-1]
                x += 1
                if ([x,y-1] in wall):
                    return [x,y-1]
    elif (beta <= math.pi*3/4):
        while (y<ydist):
            y += 1
            if (abs(y/math.tan(beta))<abs(x-1)):
                if ([x,y-1] in wall):
                    return [x,y-1]
            else:
                if ([x,y-1] in wall):
                    return [x,y-1]
                x -= 1
                if ([x,y-1] in wall):
                    return [x,y-1]
    elif (beta <= math.pi):
        while (abs(x)<abs(xdist)):
            x -= 1
            if (abs(x*math.tan(beta))<y+1):
                if ([x+1,y] in wall):
                    return [x+1,y]
            else:
                if ([x+1,y] in wall):
                    return [x+1,y]
                y += 1
                if ([x+1,y] in wall):
                    return [x+1,y]
    elif (beta <= math.pi*5/4):
        while (abs(x)<abs(xdist)):
            x -= 1
            if (abs(x*math.tan(beta))<abs(y-1)):
                if ([x+1,y] in wall):
                    return [x+1,y]
            else:
                if ([x+1,y] in wall):
                    return [x+1,y]
                y -= 1
                if ([x+1,y] in wall):
                    return [x+1,y]
    elif (beta <= math.pi*3/2):
        while (abs(y)<abs(ydist)):
            y -= 1
            if (abs(y/math.tan(beta))<abs(x+1)):
                if ([x,y+1] in wall):
                    return [x,y+1]
            else:
                if ([x,y+1] in wall):
                    return [x,y+1]
                x -= 1
                if ([x,y+1] in wall):
                    return [x,y+1]
    elif (beta <= math.pi*7/4):
        while (abs(y)<abs(ydist)):
            y -= 1
            if (abs(y/math.tan(beta))<abs(x+1)):
                if ([x,y+1] in wall):
                    return [x,y+1]
            else:
                if ([x,y+1] in wall):
                    return [x,y+1]
                x += 1
                if ([x,y+1] in wall):
                    return [x,y+1]
    elif (beta <= math.pi*2):
        while (abs(x)<abs(xdist)):
            x += 1
            if (abs(x*math.tan(beta))<abs(y-1)):
                if ([x-1,y] in wall):
                    return [x-1,y]
            else:
                if ([x-1,y] in wall):
                    return [x-1,y]
                y -= 1
                if ([x-1,y] in wall):
                    return [x-1,y]
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

    def yolo_callback(self, data):
        latest_yolo_msg = data
        for r in latest_yolo_msg.yolov8_inference:
            class_name = r.class_name
            top = r.top
            left = r.left
            bottom = r.bottom
            right = r.right
            self.get_logger().info(
                f"Received YOLO detection: "
                f"class_name={class_name}, top={top}, left={left}, bottom={bottom}, right={right}"
            )
            self.perform_logic(class_name, top, left, bottom, right)
    
    def map_callback(self, msg):
        self.map_msg = msg
        self.map_resolution = self.map_msg.info.resolution
        self.map_originX = self.map_msg.info.origin.position.x
        self.map_originY = self.map_msg.info.origin.position.y
        self.map_width = self.map_msg.info.width
        self.map_height = self.map_msg.info.height
        self.map_data = self.map_msg.data


    def odom_callback(self, msg):
        self.odom_msg = msg
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        self.odom_yaw = euler_from_quaternion(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                                              msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)

    def perform_logic(self, class_name, top, left, bottom, right):
        # Ensure both messages are available
        if self.map_msg is not None and self.odom_msg is not None and class_name is not None:
            #Extract walls from map data
            reshaped_data = np.array(self.map_data).reshape(self.map_height, self.map_width)
            wall = np.where(reshaped_data == 100)
            wall = np.column_stack(wall)

            #Robot position to calculate angle from map frame
            robot_x = int((self.odom_y - self.map_originY)/self.map_resolution)
            robot_y = int((self.odom_x - self.map_originX) / self.map_resolution)

            #Robot yaw to calculate angle camera is facing
            robot_yaw = self.odom_yaw

            #Picture xy to calculate angle of picture from lens
            picture_x = (top+bottom)/2
            picture_y = (left+right)/2#Consider adding check to avoid extra chair detection
            angle = picture_x *5#some calculation CHANGE THIS
            
            #CHeck occupancygrid for the code below. The origin isn't 0,0??
            picture_mapped_location = bresenhams_line_algorithm(math.tan(robot_y-self.map_originY,robot_x-self.map_originX), robot_yaw, angle, dist, wall, robot_x, robot_y)
            #for each step forward in bl, check if it's a wall
            if (picture_mapped_location is not None):
                add_to_count(class_name, picture_mapped_location, somedistance) #add somedistance, can divide by map resolution also
        self.latest_msg1 = None
        self.latest_msg2 = None

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
