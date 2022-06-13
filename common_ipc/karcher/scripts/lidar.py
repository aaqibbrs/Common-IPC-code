#!/usr/bin/env python

import rospy
import roslaunch

from std_msgs.msg import Int8

#location = "/home/bros/linorobot_ws/src/common_ipc/karcher/launch/lidar0.launch"
#location1 = "/home/bros/linorobot_ws/src/common_ipc/karcher/launch/lidar1.launch"

class lidar_check():
    def __init__(self):

        rospy.init_node("LidarCheck")

        rospy.Subscriber('/lidar_check',Int8,self.callback)

        self.done = 0
        location = rospy.get_param("/location")
        location1 = rospy.get_param("/location1")
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.uuid)

        self.launch = roslaunch.parent.ROSLaunchParent(self.uuid, [location])
        self.launch1 = roslaunch.parent.ROSLaunchParent(self.uuid, [location1])

        try :
            self.launch.start()
        except :
            rospy.logwarn("Correct the location of launch file")
        else :
            rospy.loginfo("First Lidar Launch done correctly")

    def callback(self, msg):
        if(self.done == 0 ) :
            data = msg.data
            if(data == 1) :
                self.launch.shutdown()
                rospy.sleep(3)

                try :
                    rospy.loginfo("Trying to launch second launch file")
                    self.launch1.start()
                except :
                    rospy.logwarn("Correct the location of launch file")
                else :
                    rospy.loginfo("Second lidar launch done correctly")
                data = 0

            if(data == 2) :
                self.done = 1
                rospy.logdebug("You may START operation now")

if __name__ == '__main__':
    try:
        lidar_check()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass