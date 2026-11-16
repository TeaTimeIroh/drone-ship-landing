# Set up a shell for this project. Use it with:   source env.sh
# Needed by the VS Code tasks, which do not read ~/.bashrc. Harmless to run twice.

ARDU_WS="${ARDU_WS:-$HOME/ardu_ws}"

source /opt/ros/humble/setup.bash
source "$ARDU_WS/install/setup.bash"

export PATH="$HOME/.local/bin:$PATH"          # mavproxy.py lives here
export GZ_VERSION=harmonic

# The ardupilot_gazebo package does not add its own share folder to the model path,
# and the drone model cannot be found without it.
case ":$GZ_SIM_RESOURCE_PATH:" in
  *":$ARDU_WS/install/ardupilot_gazebo/share:"*) ;;
  *) export GZ_SIM_RESOURCE_PATH="$ARDU_WS/install/ardupilot_gazebo/share:$GZ_SIM_RESOURCE_PATH" ;;
esac
export SDF_PATH="$GZ_SIM_RESOURCE_PATH"
