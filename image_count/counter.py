import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # Replace with your specific message types
from yolov8_msgs.msg import Yolov8Inference
from nav_msgs.msg import OccupancyGrid, Odometry

import numpy as np
import math

list_class = ["cat", "dog", "chair", "car", "apple", "stop sign"]
list_pos = [] #Contains [x,y]
dict_count = {"cat": 0, "dog": 0, "chair": 0, "car": 0, "apple": 0, "stop sign": 0} #Contains "cat": 1


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
    print(class_name + " detected at " + str(position[0]) + ", " + str(position[1]))
    if (class_name in list_class):
        #print(class_name + " is in list_class")
        eligible = False
        #If list_pos is empty, make the first entry eligible
        if not list_pos:
            eligible = True
        for i in list_pos:
            #if true keep going in the for loop
            #if false, eligible = True and break out
            if ((((position[0]-i[0])^2)+((position[1]-i[1])^2))>somedistance):
                eligible = True
                break
        if (eligible == True):
            dict_count[class_name] += 1
            list_pos.append(position)
            print(class_name + " added")
            print(dict_count)


#Takes the diff angles and wall, and calculates whether 
def bresenhams_line_algorithm(theta, gamma, dist, wall, robot_x, robot_y):
    #print("bla just ran")
    beta = theta + gamma
    print("beta: " +str(beta))
    print("theta: " +str(theta))
    print("gamma: " +str(gamma))
    #add the robot angle relative to origin here PLS
    #and fix the whole x/y row/column thing. swap back?? change the robot_x robot_y??
    #figure out how the initial robot yaw is decided
    #and also how the initial map angle is decided
    #Make beta a number from 0 to 2pi
    #let it run and see how the robot yaw and map angle stuff changes
    while (beta > 2*math.pi):
        beta -= 2*math.pi
    while (beta < 0):
        beta += 2*math.pi  
    print("beta(new): " +str(beta))
    xdist = dist*math.cos(beta)
    ydist = dist*math.sin(beta)
    x = robot_x
    y = robot_y
    if (beta <= math.pi/4):
        while (x<xdist):
            x += 1
            if (x*math.tan(beta)<y+1):
                if ([y,x-1] in wall):
                    return [y,x-1]
            else:
                if ([y,x-1] in wall):
                    return [y,x-1]
                y += 1
                if ([y,x-1] in wall):
                    return [y,x-1]
    elif (beta <= math.pi/2):
        while (y<ydist):
            y += 1
            if (y/math.tan(beta)<x+1):
                if ([y-1,x] in wall):
                    return [y-1,x]
            else:
                if ([y-1,x] in wall):
                    return [y-1,x]
                x += 1
                if ([y-1,x] in wall):
                    return [y-1,x]
    elif (beta <= math.pi*3/4):
        while (y<ydist):
            y += 1
            if (abs(y/math.tan(beta))<abs(x-1)):
                if ([y-1,x] in wall):
                    return [y-1,x]
            else:
                if ([y-1,x] in wall):
                    return [y-1,x]
                x -= 1
                if ([y-1,x] in wall):
                    return [y-1,x]
    elif (beta <= math.pi):
        while (abs(x)<abs(xdist)):
            x -= 1
            if (abs(x*math.tan(beta))<y+1):
                if ([y,x+1] in wall):
                    return [y,x+1]
            else:
                if ([y,x+1] in wall):
                    return [y,x+1]
                y += 1
                if ([y,x+1] in wall):
                    return [y,x+1]
    elif (beta <= math.pi*5/4):
        while (abs(x)<abs(xdist)):
            x -= 1
            if (abs(x*math.tan(beta))<abs(y-1)):
                if ([y,x+1] in wall):
                    return [y,x+1]
            else:
                if ([y,x+1] in wall):
                    return [y,x+1]
                y -= 1
                if ([y,x+1] in wall):
                    return [y,x+1]
    elif (beta <= math.pi*3/2):
        while (abs(y)<abs(ydist)):
            y -= 1
            if (abs(y/math.tan(beta))<abs(x+1)):
                if ([y+1,x] in wall):
                    return [y+1,x]
            else:
                if ([y+1,x] in wall):
                    return [y+1,x]
                x -= 1
                if ([y+1,x] in wall):
                    return [y+1,x]
    elif (beta <= math.pi*7/4):
        while (abs(y)<abs(ydist)):
            y -= 1
            if (abs(y/math.tan(beta))<abs(x+1)):
                if ([y+1,x] in wall):
                    return [y+1,x]
            else:
                if ([y+1,x] in wall):
                    return [y+1,x]
                x += 1
                if ([y+1,x] in wall):
                    return [y+1,x]
    elif (beta <= math.pi*2):
        while (abs(x)<abs(xdist)):
            x += 1
            if (abs(x*math.tan(beta))<abs(y-1)):
                if ([y,x-1] in wall):
                    return [y,x-1]
            else:
                if ([y,x-1] in wall):
                    return [y,x-1]
                y -= 1
                if ([y,x-1] in wall):
                    return [y,x-1]
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
        print(self.map_msg.info.origin.orientation.x)
        print(self.map_msg.info.origin.orientation.y)
        print(self.map_msg.info.origin.orientation.z)
        print(self.map_msg.info.origin.orientation.w)
        self.map_yaw = euler_from_quaternion(self.map_msg.info.origin.orientation.x, self.map_msg.info.origin.orientation.y, self.map_msg.info.origin.orientation.z, self.map_msg.info.origin.orientation.w)


    def odom_callback(self, msg):
        self.odom_msg = msg
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        self.odom_yaw = euler_from_quaternion(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                                              msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)

    def perform_logic(self, class_name, top, left, bottom, right):
        print(self.odom_yaw)
        # Ensure both messages are available
        if self.map_msg is not None and self.odom_msg is not None and class_name is not None:
            #Extract walls from map data
            print("og wall:")
            print(self.map_data)
            reshaped_data = np.array(self.map_data).reshape(self.map_height, self.map_width)
            print("reshaped_data:")
            print(reshaped_data)
            wall = np.where(reshaped_data == 100)
            print("wall 1:")
            print(wall)
            wall = np.column_stack(wall) #check what this actually is
            print("wall2")
            print(wall)

            #Robot position to calculate angle from map frame
            robot_x = int((self.odom_y - self.map_originY)/self.map_resolution)
            robot_y = int((self.odom_x - self.map_originX) / self.map_resolution)

            #Robot yaw to calculate angle camera is facing
            robot_yaw = self.odom_yaw

            #Picture xy to calculate angle of picture from lens
            picture_x = (top+bottom)/2
            picture_y = (left+right)/2#Consider adding check to avoid extra chair detection
            angle = (picture_x-160)*(math.pi/2)/(320.0) #convert location on screen to angle (roughly) 
            print("robot_x: " + str(robot_x))
            print("robot_y: " + str(robot_y))
            print("robot_yaw: " + str(robot_yaw))
            print("map_originX: " + str(self.map_originX))
            print("map_originY: " + str(self.map_originY))
            print("odom_x: " + str(self.odom_x))
            print("odom_y: " + str(self.odom_y))
            print("map_yaw: " + str(self.map_yaw))
            #Check occupancygrid for the code below. After checking, looks like the robot_x and robot_y are taken with respect to the origin, and thus no need to take into account the map_originX and map_originY
            #picture_mapped_location = bresenhams_line_algorithm(math.tan(robot_y-self.map_originY,robot_x-self.map_originX), robot_yaw, angle, dist, wall, robot_x, robot_y)
            #TEMPORARY
            dist = 50
            picture_mapped_location = bresenhams_line_algorithm(robot_yaw, angle, dist, wall, robot_x, robot_y)
            #for each step forward in bl, check if it's a wall
            if (picture_mapped_location is not None):
                #TEMPORARY
                somedistance = 5
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
