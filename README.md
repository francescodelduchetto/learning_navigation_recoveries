# learning_local_recovery_navigation
Source code of the Learning-by-Demonstration framework illustrated in "Don't Make the Same Mistakes Again and Again: Learning Local Recovery Policies for Navigation from Human Demonstrations"

The code is based on ROS kinetic and it runs the experiments using the MORSE simulator.

## Installation
Install ros-kinetic, mongodb, topological navigation, stramds_morse, strands_movebase, tmule, GPy, PetriNetPlans *branch* and how to install,  

## Configuration
Edit the file `launch/launch_tmule.launch` to put the absolute path to the `config` folder for the `TMULE_CONFIG` argument.

Edit the `scripts/setup.sh` file to put the path to your catkin workspace.

Copy pnp_ros, pnp_msgs, and learning_local_recovery_navigation into the `src/` folder of your catkin workspace, then (when inside your workspace folder) build everything with `catkin_make`.

## Executing

`roslaunch llrn launch_tmule.launch`

Then open `0.0.0.0:9999` in a browser.

Launch all the windows up to `create_topological_map`, then create a topological map as explained in [link](link).

Stop the window `create_topological_map` and start all the remaining.

Now you can execute a navigation plan (one of those in `pnp_ros/plans`, or you can write your own and generate it by `pnpgen_linear YOUR_PLAN_NAME.plan YOUR_EXECUTION_RULES.er`) by publishing on the topic `/planToExec`, e.g. `rostopic pub /planToExec std_msgs/String "data: 'passdoor12'"`.

You will be able to signal a failure by pressing the signal button, after some signalling the model should start detecting some similar failure (that you can discard or confirm). After a confirmation you are asked to demonstrate a recovery trajectory that will be automatically used to learn the recovery regression model.


Maintainer: fdelduchetto@lincoln.ac.uk
