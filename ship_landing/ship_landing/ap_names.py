"""Find where ArduPilot's ROS 2 interface lives.

The topic and service names depend on the autopilot parameter DDS_USE_NS:
'/ap/...' when it is 0 and '/ap/v<sysid>/...' when it is 1. Looking the
prefix up at run time keeps the nodes working with either setting.
"""

MODE_SWITCH_TYPE = 'ardupilot_msgs/srv/ModeSwitch'
SUFFIX = '/mode_switch'


def find_ap_prefix(node):
    """Return the prefix, for example '/ap/v1', or None if the autopilot is not visible yet."""
    for name, types in node.get_service_names_and_types():
        if name.endswith(SUFFIX) and MODE_SWITCH_TYPE in types:
            return name[:-len(SUFFIX)]
    return None
