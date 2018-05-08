import os
import rospy
import rosbag
import Tkinter as tk
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalID
from AbstractAction import AbstractAction
from std_msgs.msg import Float64MultiArray

class recordDemonstration(AbstractAction):

    def _start_action(self):
        ## start the rosbag for saving the demonstration
        self.goal_id = rospy.Time.now().to_nsec()
        #folder = '%s/workspaces/museum_ws/data/recover_trajectories'  % os.path.expanduser("~")
        #filepath = '%s/%s.txt' % (folder, self.goal_id)
        self.saving_bag = rosbag.Bag(str(self.goal_id), "w")

        ## create graphical interface for starting demonstration
        window_s = tk.Tk()
        self.start = False
        def start_cb():
            self.start = True
            window_s.destroy()

        label = tk.Label(window_s, text="Press start when you are ready to demonstrate")
        label.pack()
        btn = tk.Button(window_s, text="Start", command = start_cb, height = 10, width = 20)
        btn.pack()
        window_s.mainloop()

        if self.start:
            # subscribe to laser scan window and twist
            rospy.Subscriber("LaserScanWindow", Float64MultiArray, self._scan_window_callback)
            rospy.Subscriber("cmd_vel", Twist, self._twist_callback)

            # wait until the user moves the robot
            self._robot_moved = False
            rate = rospy.Rate(20)
            while not self._robot_moved:
                rate.sleep()

            # starting time action
            self.params[len(self.params):] = [rospy.Time.now().to_sec()]
            self.params[len(self.params):] = ["recording"]
        else:
            self.params[len(self.params):] = ["done"]

    def _stop_action(self):
        if "goal_id" in dir(self):
            # stop recording trajectory
            self.params = ["done"]

            # close the bag
            self.saving_bag.close()

            # signal that a new demonstration has been received
            signal_pub = rospy.Publisher("new_recovery_demonstration", String, latch=True, queue_size=10)
            msg = String("")
            signal_pub.publish(msg)

    @classmethod
    def is_goal_reached(cls, params):
        if len(params) > 0 and params[-1] == "done":
            return True

        if len(params) > 1:
            elapsed_time = rospy.Time.now().to_sec() - params[-2]
            if params[-1] == "recording" and elapsed_time > 6:
                return True

        return False


    def _twist_callback(self, msg):
        if len(self._params) > 1 and params[-1] == "recording":
            # we started recording, save in the bag then
            self.saving_bag.write("cmd_vel", msg)
        else:
            # set _robot_moved to true when the linear or angular velocity of the robot is greater than move_thr
            move_thr = 0.001
            if abs(msg.linear.x) > move_thr \
                    or abs(msg.angular.z) > move_thr:
                self._robot_moved = True


    def _scan_window_callback(self, msg):
        if len(self._params) > 1 and params[-1] == "recording":
            # we started recording, save in the bag then
            self.saving_bag.write("scan_window", msg)
