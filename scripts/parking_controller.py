#!/usr/bin/env python

import rospy
import numpy as np

from visual_servoing.msg import ConeLocation, ParkingError
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32

class ParkingController():
    """
    A controller for parking in front of a cone.
    Listens for a relative cone location and publishes control commands.
    Can be used in the simulator and on the real robot.
    """
    def __init__(self):
        rospy.Subscriber("/relative_cone", ConeLocation,
            self.relative_cone_callback)

        DRIVE_TOPIC = rospy.get_param("~drive_topic") # set in launch file; different for simulator vs racecar
        direction_topic = "/direct_pub"

        self.drive_pub = rospy.Publisher(DRIVE_TOPIC,
            AckermannDriveStamped, queue_size=10)
        self.error_pub = rospy.Publisher("/parking_error",
            ParkingError, queue_size=10)

        #Listen to the direction pub
        self.stop_sub = rospy.Subscriber(direction_topic, Float32, self.direct_cb)

        self.parking_distance = 0.25 # meters; try playing with this number! 0.25 = FUNCTIONAL
        # self.parking_distance = 0.5 # meters; try playing with this number!
        self.relative_x = 0
        self.relative_y = 0
        self.speed = 0.5
        self.previous_turn = 0.0 #keeps track of the previous turn of the robot
        self.line_side = 0.0

        self.s = "/home/racecar/error_log_parking" + str(self.speed) + ".csv"

        with open(self.s, "w") as self.error_log:
            self.error_log.write("")
        self.error_log = open(self.s, "a")
    
    def direct_cb(self,msg):
        self.line_side = msg.data

    def relative_cone_callback(self, msg):
        self.relative_x = msg.x_pos
        self.relative_y = msg.y_pos
        drive_cmd = AckermannDriveStamped()

        #################################
        # YOUR CODE HERE
        # Use relative position and your control law to set drive_cmd
        
        #find L1 = distance between car and cone (basic distance equation)
        l = np.sqrt(self.relative_x**2 + self.relative_y**2)
        #set car length
        L = .325
        #find nu = angle between car and cone, aTan(y/x)
        nu = np.arctan2(self.relative_y, self.relative_x)
        #drive angle = the thing we are setting
        if l <= self.parking_distance + 0.2 + self.speed*0.3: #l <= self.parking_distance + 0.2 + self.speed*0.3
            rospy.logwarn("too close")
            drive_cmd.drive.speed = -0.3 #higher speed = go back farther, -0.5 = FUNCTIONAL
            if self.previous_turn <= 0.2 and self.previous_turn >= -0.2:
                #print(self.line_side)
                drive_cmd.drive.steering_angle = 1.0 *self.line_side
            else:
                drive_cmd.drive.steering_angle = self.previous_turn
            
            #np.arctan(2*L*np.sin(nu)/l)*l
            #old equation
            #np.arctan(2*L*np.sin(nu)/l)*l
        else:
            rospy.logwarn(l)
            drive_cmd.drive.speed = self.speed
            drive_cmd.drive.steering_angle = np.arctan(2*L*np.sin(nu)/l)*l
            self.previous_turn = drive_cmd.drive.steering_angle
        
        # write the error to a file
        self.error_log.write(str(rospy.get_rostime().to_sec())+",")
        self.error_log.write(str(self.relative_x)+",")
        self.error_log.write(str(self.relative_y)+",")
        self.error_log.write(str(np.sqrt(self.relative_x**2 + self.relative_y**2))+"\n")

        #################################

        self.drive_pub.publish(drive_cmd)
        self.error_publisher()

    def error_publisher(self):
        """
        Publish the error between the car and the cone. We will view this
        with rqt_plot to plot the success of the controller
        """
        error_msg = ParkingError()

        #################################

        # YOUR CODE HERE
        # Populate error_msg with relative_x, relative_y, sqrt(x^2+y^2)
        error_msg.x_error = self.relative_x
        error_msg.y_error = self.relative_y
        error_msg.distance_error = np.sqrt(self.relative_x**2 + self.relative_y**2)

        #################################
        
        self.error_pub.publish(error_msg)

if __name__ == '__main__':
    try:
        rospy.init_node('ParkingController', anonymous=True)
        ParkingController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
