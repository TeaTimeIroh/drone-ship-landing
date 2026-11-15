"""Autonomous takeoff, short velocity cruise and landing through ArduPilot's ROS 2 (DDS) interface.

Run with the simulator already up and the vehicle disarmed on the ground:

    ros2 run ship_landing takeoff_demo --ros-args -p use_sim_time:=true
"""
from ship_landing.flight_base import FlightNode, run


class TakeoffDemo(FlightNode):
    AIRBORNE_STATES = ('CLIMB', 'CRUISE')
    FIRST_MISSION_STATE = 'CRUISE'

    def __init__(self):
        super().__init__('takeoff_demo')
        self.cruise_speed = self.declare_parameter('cruise_speed', 1.0).value
        self.cruise_time = self.declare_parameter('cruise_time', 5.0).value

    def fly(self, now):
        # CRUISE: fly east at a fixed speed for a fixed time, then land.
        self.send_map_velocity(now, east=self.cruise_speed)
        if self.time_in_state(now) > float(self.cruise_time):
            self.enter('LAND', now)


def main():
    run(TakeoffDemo)


if __name__ == '__main__':
    main()
