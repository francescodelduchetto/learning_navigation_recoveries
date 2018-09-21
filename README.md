# learning_local_recovery_navigation
Source code of the Learning-by-Demonstration framework illustrated in "Don't Make the Same Mistakes Again and Again: Learning Local Recovery Policies for Navigation from Human Demonstrations"

The code is based on **ROS kinetic** and it runs the experiments using the MORSE simulator.

## Installation
1. Install TMuLE: `pip install tmule`.

2. Install GPy: `pip install GPy`.

3. Install the STRANDS repository (quick setup in https://strands.readthedocs.io/en/latest/quick_setup.html). In particular you will need the following packages `strands_desktop`, `strands_movebase`, `strands_morse`, `topological_navigation`, `mongodb_store`.

4. Clone the Petri Net plans repository (costmap branch) from https://github.com/francescodelduchetto/PetriNetPlans/tree/costmap.

5. Clone this repository.

7. Create a catkin_workspace (http://wiki.ros.org/catkin/Tutorials/create_a_workspace)

6. Link `PetriNetPlans/PNPros/ROS_bridge/pnp_ros/pnp_ros`, `PetriNetPlans/PNPros/ROS_bridge/pnp_ros/pnp_msgs`, and `learning_local_recovery_navigation` into the `src/` folder of your catkin workspace, then (when inside your workspace folder) build everything with `catkin_make`.

## Configuration
- Edit the file `launch/launch_tmule.launch` to put the absolute path to the `config` folder for the `TMULE_CONFIG` argument.

- Edit the `scripts/setup.sh` file to put the path to your catkin workspace.

- Create the empty directory `~/mongodb/test`, or in you preferred place but change accordingly `MONGO_FOLDER` in `scripts/setup.sh`.

## Executing

1. `roslaunch llrn launch_tmule.launch`

2. Open `0.0.0.0:9999` in a browser.

3. Launch all the windows up to the one named `create_topological_map`, then create a topological map as explained in [link](link).

4. Stop the window `create_topological_map` and start all the remaining.

Now you can execute a navigation plan (one of those in `pnp_ros/plans`, or you can write your own and generate it by `pnpgen_linear YOUR_PLAN_NAME.plan YOUR_EXECUTION_RULES.er`) by publishing on the topic `/planToExec`, e.g. `rostopic pub /planToExec std_msgs/String "data: 'passdoor12'"`.

You will be able to signal a failure by pressing the signal button, after some signalling the model should start detecting some similar failure (that you can discard or confirm). After a confirmation you are asked to demonstrate a recovery trajectory that will be automatically used to learn the recovery regression model.


Maintainer: fdelduchetto@lincoln.ac.uk
