#!/usr/bin/env python
# license removed for brevity

import rospy
import actionlib
import numpy as np
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from random import uniform, randint
from nav_msgs.msg import OccupancyGrid
from actionlib_msgs.msg import GoalStatus
# from move_base_msgs.msg import MoveBaseFeedback
from geometry_msgs.msg import Twist, PoseStamped
import numpy.linalg as LA


class RandomExplorer:

    def sampling(self):
        while self.map is None:
            rospy.loginfo('Waiting for map to be ready...')
            rospy.sleep(1)

        rospy.loginfo('Map is ready!')
        # move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        # move_base_client.wait_for_server()
        
        
        while not rospy.is_shutdown():
            if len(self.unoccupied_indices) == 0:
                rospy.logwarn('No unoccupied areas in map.')
                continue
            rospy.loginfo("Inside main while loop")
            i = randint(0, len(self.unoccupied_indices) - 1)
            index = self.unoccupied_indices[i]
            x = self.map_origin_x + (index % self.map_width + 0.5) * self.map_resolution     # goal x point
            y = self.map_origin_y + (index // self.map_width + 0.5) * self.map_resolution    # goal y point

            client = actionlib.SimpleActionClient('move_base',MoveBaseAction)   # SIMPLE ACTION CLIENT
            client.wait_for_server()

            goal = MoveBaseGoal()      # Creating an instance of MoveBaseGoal class
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = x
            goal.target_pose.pose.position.y = y
            goal.target_pose.pose.orientation.w = 1.0
            rospy.loginfo(goal)

            last_sample_time = rospy.Time.now()
            client.send_goal(goal)
            wait = client.wait_for_result()
            status = client.get_state()
            euclidean_distance = ((self.x_coord-x)**2 + (self.y_coord-y)**2)**0.5

            # while status != GoalStatus.SUCCEEDED and (rospy.Time.now() - last_sample_time).to_sec() < 30:
            while euclidean_distance>=0.1 and (rospy.Time.now() - last_sample_time).to_sec() < 30:
                euclidean_distance = ((self.x_coord-x)**2 + (self.y_coord-y)**2)**0.5
                print(euclidean_distance)
                rospy.loginfo('Moving to goal...')
                print('t = ' + str((rospy.Time.now() - last_sample_time).to_sec()) + " seconds")
                rospy.sleep(1)
                
            ### The above while loop will be running when searching for goal.

            # if status == GoalStatus.SUCCEEDED:
            if euclidean_distance<0.1:
                rospy.loginfo(status)
                rospy.loginfo('Reached goal')
            elif (rospy.Time.now() - last_sample_time).to_sec() > 30 :
                rospy.loginfo('Aborted Point')

            # ROTATION IN PLACE
            rotation = Twist()    #  An instance of the Twist class
            rotation.linear.x = 0
            rotation.angular.z = 0.6282
            SpinEndTime = rospy.Time.now() + self.spin_duration
            while rospy.Time.now()<=SpinEndTime:   
                print("Rotating to search for tags.")
                self.rotation_publisher.publish(rotation)
                rospy.sleep(0.1)
        
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else: 
            return client.get_result() 
    
    
    def map_callback(self, data):
        # rospy.loginfo(data)
        self.map = data.data
        self.map_resolution = data.info.resolution
        self.map_origin_x = data.info.origin.position.x
        self.map_origin_y = data.info.origin.position.y
        self.map_width = data.info.width
        self.map_height = data.info.height

        # Find unoccupied indices
        self.unoccupied_indices = [i for i in range(len(self.map)) if self.map[i] == 0]
        self.sampling()

    def feedback_callback(self,feedback):
        self.x_coord = feedback.pose.position.x
        self.y_coord = feedback.pose.position.y
        self.z_coord = feedback.pose.position.z

    def __init__(self):
        rospy.init_node('random_explorer')
        #self.listener = TransformListener()
        self.map = None
        self.map_resolution = None
        self.map_origin_x = None
        self.map_origin_y = None
        self.map_width = None
        self.map_height = None
        self.unoccupied_indices = []
        self.spin_duration = rospy.Duration(10)
        self.x_coord = 0
        self.y_coord = 0
        self.z_coord = 0

        # Set up subscribers and publishers
        rospy.Subscriber('move_base/feedback',PoseStamped,self.feedback_callback)
        rospy.Subscriber('/cmap', OccupancyGrid, self.map_callback)
        self.rotation_publisher = rospy.Publisher('/cmd_vel',Twist,queue_size = 10)    # Defining a publisher for in-place rotation motion. 




if __name__ == '__main__':
    explorer = RandomExplorer()   # Creating an instance of the RandomExplorer() class.
    explorer.sampling()


