''' 
Date: 2023-01-31 22:23:17
LastEditTime: 2023-03-01 16:50:27
Description:
    Copyright (c) 2022-2023 Safebench Team

    This file is modified from <https://github.com/carla-simulator/scenario_runner/tree/master/srunner/scenarios>
    Copyright (c) 2018-2020 Intel Corporation

    This work is licensed under the terms of the MIT license.
    For a copy, see <https://opensource.org/licenses/MIT>
'''

import math
import carla

from safebench.scenario.tools.scenario_operation import ScenarioOperation
from safebench.scenario.tools.scenario_utils import calculate_distance_transforms
from safebench.scenario.scenario_manager.carla_data_provider import CarlaDataProvider
from safebench.scenario.scenario_definition.basic_scenario import BasicScenario
from safebench.scenario.tools.scenario_helper import get_location_in_distance_from_wp


class DynamicObjectCrossing(BasicScenario):
    """
        The scenario realizes the user controlled ego vehicle moving along the road and encountering a cyclist ahead.
    """

    def __init__(self, world, ego_vehicle, config, timeout=60):
        super(DynamicObjectCrossing, self).__init__("DynamicObjectCrossing-CC", config, world)
        self.ego_vehicle = ego_vehicle
        self.timeout = timeout

        # parameters = [left_lane_objects, right_lane_objects]
        self.parameters = config.parameters
        self._map = CarlaDataProvider.get_map()

        self._reference_waypoint = self._map.get_waypoint(config.trigger_points[0].location)

        self.right_transform = None
        self.left_transform = None

        self._trigger_location = config.trigger_points[0].location

        # Total Number of attempts to relocate a vehicle before spawning
        self._number_of_attempts = 20
        # Number of attempts made so far
        self._spawn_attempted = 0

        # always should be set
        self.scenario_operation = ScenarioOperation()
        self.ego_max_driven_distance = 150
        self.trigger_distance_threshold = 5

    def _calculate_base_transform(self, _start_distance, waypoint):
        lane_width = waypoint.lane_width

        # Patches false junctions
        if self._reference_waypoint.is_junction:
            stop_at_junction = False
        else:
            stop_at_junction = True

        location, _ = get_location_in_distance_from_wp(waypoint, _start_distance, stop_at_junction)
        waypoint = self._map.get_waypoint(location)
        offset = {"orientation": 270, "position": 90, "z": 0.6, "k": 1.0}
        position_yaw = waypoint.transform.rotation.yaw + offset['position']
        orientation_yaw = waypoint.transform.rotation.yaw + offset['orientation']
        offset_location = carla.Location(
            offset['k'] * lane_width * math.cos(math.radians(position_yaw)),
            offset['k'] * lane_width * math.sin(math.radians(position_yaw)))
        location += offset_location
        location.z = self._trigger_location.z + offset['z']
        return carla.Transform(location, carla.Rotation(yaw=orientation_yaw)), orientation_yaw

    def initialize_actors(self):
        _start_distance = 0#self.parameters[0]

        waypoint_right = self._reference_waypoint.get_left_lane()
        waypoint_left = waypoint_right.get_left_lane()

        self.base_transform_right, orientation_yaw_right = self._calculate_base_transform(_start_distance,
                                                                                          waypoint_right)
        self.base_transform_left, orientation_yaw_left = self._calculate_base_transform(_start_distance, waypoint_left)

        # objects
        objects_right = self.parameters[1]
        objects_left = self.parameters[2]

        # object transforms
        gap_right = waypoint_right.lane_width / (len(objects_right) + 1)
        transforms_right = []
        for i in range(len(objects_right)):
            loc_y = self.base_transform_right.location.y - waypoint_right.lane_width / 2 + (i + 1) * gap_right
            transforms_right.append(
                carla.Transform(
                    carla.Location(self.base_transform_right.location.x, loc_y,
                                   self.base_transform_right.location.z),
                    self.base_transform_right.rotation
                )
            )

        gap_left = waypoint_left.lane_width / (len(objects_left) + 1)
        transforms_left = []
        for i in range(len(objects_left)):
            loc_y = self.base_transform_left.location.y - waypoint_left.lane_width / 2 + (i + 1) * gap_left
            transforms_left.append(
                carla.Transform(
                    carla.Location(self.base_transform_left.location.x, loc_y,
                                   self.base_transform_left.location.z),
                    self.base_transform_left.rotation
                )
            )

        # pass results
        self.actor_type_list = objects_right + objects_left
        self.actor_transform_list = transforms_right + transforms_left
        self.other_actors = self.scenario_operation.initialize_vehicle_actors(self.actor_transform_list,
                                                                              self.actor_type_list)
        self.reference_actor = self.other_actors[0]  # used for triggering this scenario

    def create_behavior(self, scenario_init_action):
        assert scenario_init_action is None, f'{self.name} should receive [None] action. A wrong scenario policy is used.'

    def update_behavior(self, scenario_action):
        assert scenario_action is None, f'{self.name} should receive [None] action. A wrong scenario policy is used.'
        # no update needed

    def check_stop_condition(self):
        return False
