import os
import GPy
import rospy
import threading
import numpy as np
from std_srvs.srv import Empty
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray


model_lock = threading.Lock()
cmdVelPub = None
running = False
model = None
meanX = None
stdX = None



def start_recovery_execution():
    global running
    rospy.loginfo("Starting recovery execution")
    running = True
    return {}

def stop_recovery_execution():
    global running
    rospy.loginfo("Stopping recovery execution")
    running = False
    return {}

def update_model():
    global model, model_lock, meanX, stdX
    # extract target and label data from trajectories
    folder = '%s/workspaces/museum_ws/data/recover_trajectories'  % os.path.expanduser("~")
    Xtrain = []
    Ytrain = []
    for filename in os.listdir(folder):
        filepath = folder + "/" + filename
        if filename.endswith(".txt"):
            for i, line in enumerate(open(filepath, "r")):
                y = []
                x = []

                state_conds = line.split("\t")[0].split("  ")
                for state_cond in state_conds:
                    # convert to list
                    x += list(eval(state_cond.split("_")[1]))
                action_conds = line.split("\t")[1].split("  ")
                for action_cond in action_conds:
                    # convert to list
                    y += list(eval(action_cond.split("_")[1]))
                Xtrain.append(x)
                Ytrain.append(y)


    if len(Xtrain) > 0:
        multiLS = False
        LS = 400.
        F_DIM = len(Xtrain[0])

        # normalize dataset
        Xtrain = np.array(Xtrain)
        Ytrain = np.array(Ytrain)
        print "XTRAIN SHAPE", Xtrain.shape
        meanX = Xtrain.mean()
        stdX = Xtrain.std()
        Xtrain -= meanX
        Xtrain /= stdX

        kernelExpo = GPy.kern.Exponential(input_dim=F_DIM,
                                      lengthscale=LS,
                                      ARD=multiLS)
        kernel = kernelExpo # + GPy.kern.White(F_DIM)
        model_lock.acquire()
        model = GPy.models.GPRegression(Xtrain, Ytrain, kernel)
        model.optimize(max_f_eval = 1000)
        model_lock.release()

        print "RecoveryActionServer"
        print "X.shape", Xtrain.shape, "Y.shape", Ytrain.shape
        print model


def execution_callback(msg):
    global running, model, model_lock
    if running:
        if model is not None:
           # take current scan window
            current_scan_window = msg.data
            Xtest = np.array(current_scan_window)
            Xtest.shape = (1, len(current_scan_window))
            Xtest -= meanX
            Xtest /= stdX

        	# prediction
        	print "predicting"
            model_lock.acquire()
            (Yp, var) = model.predict(Xtest)
            model_lock.release()
            print "predicted"

        	# check the variance of the detection
            if var < 0.03:
                # send predicted cmd_vel
                cmdVel = Twist()
                cmdVel.linear.x = Yp[0][0]
                cmdVel.angular.z = Yp[0][1]

                cmdVelPub.publish(cmdVel)
                print "Xtest shape", Xtest.shape
                print "predicted", Yp, "with variance", var
            else:
                print "HIGH VARIANCE: ", var, "(std:", stdX,")"
                print ">>>>>> STOPPED BECAUSE I DON'T KNOW WHAT TO DO <<<<<<"
                running = False

        else:
            rospy.logwarn("The recovery model has not been generated")

if __name__ == "__main__":
    # init init
    rospy.init_node("recovery_generator")

    update_model("")

    rospy.Service("start_recovery_execution", Empty, start_recovery_execution)

    rospy.Service("stop_recovery_execution", Empty, stop_recovery_execution)

    # scan history Subscriber
    rospy.Subscriber("scan_history", Float64MultiArray, execution_callback)

    # signal that a new demonstration arrived
    rospy.Subscriber("new_recovery_demonstration", String, update_model)

    # cmd_vel publisher
    cmdVelPub = rospy.Publisher('/cmd_vel', Twist, latch=True, queue_size=1)
