import os
import rospy
import Tkinter as tk
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalID
from AbstractAction import AbstractAction
from pnp_msgs.srv import PNPStartStateActionSaver, PNPStopStateActionSaver

class recordDemonstration(AbstractAction):

    def _start_action(self):
        ## call the service for saving the trajectories
        starting_sp = rospy.ServiceProxy("start_state_action_saver", PNPStartStateActionSaver)
        self.goal_id = rospy.Time.now().to_nsec()
        folder = '%s/workspaces/museum_ws/data/recover_trajectories'  % os.path.expanduser("~")
        filepath = '%s/%s.txt' % (folder, self.goal_id)

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

            # wait until the user moves the robot
            rospy.Subscriber("cmd_vel", Twist, self._twist_callback)
            self._robot_moved = False
            rate = rospy.Rate(20)
            while not self._robot_moved:
                rate.sleep()

	    # save the laserscan window for the environment state and the twist for the action
            response = starting_sp(str(self.goal_id), filepath, ["LaserScanWindow"], ["Twist"], False).succeeded

            # starting time action
            if response:
                self.params[len(self.params):] = [rospy.Time.now().to_sec()]
                self.params[len(self.params):] = ["recording"]
            else:
                self.params[len(self.params):] = ["done"]
        else:
            self.params[len(self.params):] = ["done"]

    def _stop_action(self):
        if "goal_id" in dir(self):
	    # stop recording trajectory
            stopping_sp = rospy.ServiceProxy("stop_state_action_saver", PNPStopStateActionSaver)
            response = stopping_sp(str(self.goal_id)).succeeded

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
	# set _robot_moved to true when the linear or angular velocity of the robot is greater than move_thr
	move_thr = 0.001
        if abs(msg.linear.x) > move_thr\
                or abs(msg.angular.z) > move_thr:
            self._robot_moved = True
