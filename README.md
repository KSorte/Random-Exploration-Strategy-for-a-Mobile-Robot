I have written a node that subscribes to a SLAM map from Cartographer and plans an exploration by randomly sampling the unoccupied regions. The objective of this exploration is to detect victims.
April Tags are victims in this case. 
Version 1 : Robot does not get time to reach sampled points. Points sampled very fast. 
Version 2: A thirty seconds delay added that allows the robot to reach the sampled goal. 
Version 2.5 : Robot rotated about itself when it reaches the goal point to further improve April Tag detection ability. 
Version 3 : STILL IN PROGRESS. 
