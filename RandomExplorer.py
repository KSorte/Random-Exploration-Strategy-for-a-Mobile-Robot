#!/usr/bin/env python
# license removed for brevity

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from random import uniform, randint
from nav_msgs.msg import OccupancyGrid
from tf import TransformListener

class RandomExplorer:

    def sampling(self):
        while self.map is None:
            rospy.loginfo('Waiting for map to be ready...')
            rospy.sleep(1)

        rospy.loginfo('Map is ready!')
        move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        move_base_client.wait_for_server()
        
        
        while not rospy.is_shutdown():
            rospy.sleep(20)
            # Generate random goal from unoccupied areas
            if len(self.unoccupied_indices) == 0:
                rospy.logwarn('No unoccupied areas in map.')
                continue

            i = randint(0, len(self.unoccupied_indices) - 1)
            index = self.unoccupied_indices[i]
            x = self.map_origin_x + (index % self.map_width + 0.5) * self.map_resolution
            y = self.map_origin_y + (index // self.map_width + 0.5) * self.map_resolution
            client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
            client.wait_for_server()
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = x
            goal.target_pose.pose.position.y = y
            goal.target_pose.pose.orientation.w = 1.0
            rospy.loginfo(goal)
            client.send_goal(goal)
            wait = client.wait_for_result()
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

        # Set up subscribers
        rospy.Subscriber('/cmap', OccupancyGrid, self.map_callback)




if __name__ == '__main__':
    explorer = RandomExplorer()
    explorer.sampling()


