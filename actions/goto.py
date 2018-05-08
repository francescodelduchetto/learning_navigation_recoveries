""" Makes the robot goto a certain topological points.

   Uses the STRANDS topological navigation functionalities, if you prefer another topological 
   navigation module change the code accordingly.
"""
import rospy
import actionlib

from topological_navigation.msg import GotoNodeActionGoal, GotoNodeAction
from actionlib_msgs.msg import GoalID
from AbstractAction import AbstractAction
from pnp_msgs.srv import PNPCondition
from geometry_msgs.msg import Twist


class goto(AbstractAction):

    def _start_action(self):
        goal_topo = str(self.params[0])

        self.nav_goal = GotoNodeActionGoal()
        self.nav_goal.goal.target = goal_topo

	# connect to topological navigation action-server
        self.nav_ac = actionlib.SimpleActionClient("/topological_navigation", GotoNodeAction)
        rospy.loginfo("Connecting to /topological_navigation AS...")
        self.nav_ac.wait_for_server()
        rospy.loginfo("Connected.")

        # send navigation goal
        self.nav_ac.send_goal(self.nav_goal.goal)
        rospy.loginfo("Waiting for result...")

        print "START ACTION GOTO"


    def _stop_action(self):
	# send navigation stop command
        cancel_goal = GoalID()
        cancel_goal.id = self.nav_goal.goal_id.id
        pub = rospy.Publisher('/topological_navigation/cancel', GoalID, queue_size=10)
        pub.publish(cancel_goal)

        print "STOP ACTION GOTO"

    @classmethod
    def is_goal_reached(cls, params):
        goal_node = str(params[0])

        service_proxy = rospy.ServiceProxy("/PNPConditionEval", PNPCondition)

	# check the PNP condition for the current topological node
        condition = "CurrentNode_" + goal_node
        reached = service_proxy(condition).truth_value
        return reached
