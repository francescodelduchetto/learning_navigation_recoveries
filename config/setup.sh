export MONGO_FOLDER=~/mongodb/test
export MAP=uol_bl
export MAP_PATH=/opt/ros/kinetic/share/strands_morse/uol/maps/$MAP.yaml
export TMAP=topomap
export WS_HOME=~/workspaces/test_workspace # TODO edit

mkdir -p $WS_HOME/data/detect_trajectories
mkdir -p $WS_HOME/data/recover_trajectories

if [ -r "$WS_HOME" ]; then source "$WS_HOME/devel/setup.sh"; else source /opt/ros/kinetic/setup.sh; fi
