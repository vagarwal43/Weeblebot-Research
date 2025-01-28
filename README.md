# Weeble_Bot package Description and Members
Independent Study in UW-Madison with Professor Peter Adamczyk focusing on new bipedal robotic configuration.
<br>Team Members: (Ahphabetic)</br>
&emsp;Alli Willhite</br>
&emsp;Charles Qi</br>
&emsp;Mark Xia</br>
&emsp;Vasvi Agarwal</br>
# Setup
Steps to do to use this repo:</br>
1. Create a catkin_ws following the ROS tutorial</br>
2. Make a package called "weeble_bot" with libraries rospy and roscpp</br>
3. Type "rm -rf ~/catkin_ws/src/weeble_bot" to remove the ROS created directory; We'll substitute it with our git repo.</br>
4. Type "git clone git@github.com:07yhxiaxy/weeble_bot.git" under directory "~/carkin_ws/src"</br>
5. Generate ssh public key and private key pairs; upload the public key to your github settings under "SSH and GPG keys"</br>
&emsp;For reference: https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent</br>
&emsp;Note: the file with .pub extension is the public key while the one without it is the private key.</br>
6. Add private key to your linux machine.</br>
&emsp;First, start ssh-agent by typing "eval $(ssh-agent)"</br>
&emsp;Second, add priv key by typing "ssh-add ~/.ssh/"name of private key file"</br>
7. You should be all set for version control on git and Github. Try making minor changes to this README file and push it by typing:</br>
&emsp;1. "git add ." (this adds all files under this directory to keep track of)</br>
&emsp;2. <git commit -m "whatever you want to say, this is a commit message used to document what you did. TYPE WITH QUOTATION MARK!"></br>
&emsp;3. "git push origin main" or simply "git push"</br>
&emsp;Note: you can check the file status by typing "git status"</br>
# ROS Learning
Steps to learn ROS (borrowed from ECE/ME 439 by Professor Peter Adamczyk):</br>
	----------------ELEMENTARY STUFF----------------</br>
	1. Understand how ROS made modular programming possible</br>
	2. Understand ROS nodes, topics, and messages</br>
	3. Understand ROS publisher and subscriber</br>
	----------------INTERMEDIATE STUFF--------------</br>
	4. rosbag, rospaunch, rosparam, rqt_graph, rqt_plot, topic remapping</br>
	5. custom messages, ROS networking, using sensors to control motors
# ROS Tutorial. Be sure to check all topics
Reference links:</br>
	http://wiki.ros.org/ROS/Tutorials/UnderstandingNodes</br>
	
NOTE: PLEASE CHANGE THE FIRST LINE OF talker.py AND listener.py to #!/usr/bin/env python3 INSTEAD OF #!/usr/bin/env python</br>
This is because we will use python3 to develop the robot in the future and ROS Noetic uses python3 by default</br>
