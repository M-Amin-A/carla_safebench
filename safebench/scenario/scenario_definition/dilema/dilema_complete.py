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
import random

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

        self.parameters = config.parameters
        self._map = CarlaDataProvider.get_map()

        self._reference_waypoint = self._map.get_waypoint(config.trigger_points[0].location)
        self._trigger_location = config.trigger_points[0].location


        # hyperparameters
        self._walker_velocity = 2
        self._car_velocity = 5
        self._offset_mean, self._offset_std = 0, 0.25

        # self.start_distance = 10
        # Total Number of attempts to relocate a vehicle before spawning
        self._number_of_attempts = 5
        # Number of attempts made so far
        self._spawn_attempted = 0

        # location = self._map.get_waypoint(location).transform.location

        # always should be set
        self.scenario_operation = ScenarioOperation()
        self.ego_max_driven_distance = 150
        self.trigger_distance_threshold = 40

    def calculate_transform(self, base_waypoint, _start_distance, side_offset, orientation):
        # side offset, positive is right, negative is left
        # orientation 0 upward, 90 right, 180 backward, 270 left

        stop_at_junction = self._reference_waypoint.is_junction

        location, _ = get_location_in_distance_from_wp(base_waypoint, _start_distance, stop_at_junction)
        waypoint = self._map.get_waypoint(location)

        object_offset = {"orientation": orientation, "z": 0.6}
        orientation_yaw = waypoint.transform.rotation.yaw + object_offset['orientation']

        side_offset_location = carla.Location(
            -side_offset * math.sin(math.radians(waypoint.transform.rotation.yaw)),
            side_offset * math.cos(math.radians(waypoint.transform.rotation.yaw))
        )
        location += side_offset_location
        location.z = self._trigger_location.z + object_offset['z']

        return carla.Transform(location, carla.Rotation(yaw=orientation_yaw)), orientation_yaw

    def find_side_lane(self, waypoint, direction='right'):
        reference_yaw = waypoint.transform.rotation.yaw

        while True:
            wp_next = waypoint.get_right_lane() if direction == 'right' else waypoint.get_left_lane()

            if wp_next is None or wp_next.lane_type == carla.LaneType.Sidewalk:
                break
            elif wp_next.lane_type == carla.LaneType.Shoulder:
                # Filter Parkings considered as Shoulders
                # if wp_next.lane_width > 2:
                #     waypoint = wp_next
                break
            # other way road
            elif abs(wp_next.transform.rotation.yaw - reference_yaw) > 170:
                break
            else:
                waypoint = wp_next
        return waypoint

    def get_lanes(self, waypoint):
        reference_yaw = waypoint.transform.rotation.yaw

        waypoint = self.find_side_lane(waypoint, 'right')
        waypoints = []

        while True:
            waypoints.append(waypoint)
            wp_next = waypoint.get_left_lane()

            if wp_next is None or wp_next.lane_type == carla.LaneType.Sidewalk:
                break
            elif wp_next.lane_type == carla.LaneType.Shoulder:
                # Filter Parkings considered as Shoulders
                # if wp_next.lane_width > 2:
                #     waypoint = wp_next
                break
            # other way road
            elif abs(wp_next.transform.rotation.yaw - reference_yaw) > 170:
                break
            else:
                waypoint = wp_next
        return waypoints


    def initialize_actors(self):
        # parameters
        # [actor_types: static, walker, car]
        # [actor_names: static.prop.atm, walker.*, vehicle.nissan.patrol]
        # [start_distances: 10, 15, 8]
        # [lane_indices (0 is the most right): 0, 1, 0]

        actor_types = self.parameters[0]
        actor_names = self.parameters[1]
        start_dists = self.parameters[2]
        lane_indices = self.parameters[3]

        assert len(actor_types) == len(actor_names) == len(start_dists) == len(lane_indices)

        lane_width = self._reference_waypoint.lane_width
        lane_waypoints = self.get_lanes(self._reference_waypoint)

        transforms = []
        for i in range(len(actor_types)):
            actor_type = actor_types[i]
            start_dist = start_dists[i]
            lane_index = lane_indices[i]

            if lane_index >= len(lane_waypoints):
                raise Exception('cannot find lane at index: ', lane_index)

            if actor_type == 'car':
                side_offset = 0
                orientation = 180

            elif actor_type == 'human':
                if lane_index < len(lane_waypoints) // 2:
                    side_offset = lane_width / 2
                    orientation = 270
                else:
                    side_offset = -lane_width / 2
                    orientation = 90

            elif actor_type == 'static':
                side_offset = random.gauss(self._offset_mean, self._offset_std)
                orientation = 0

            else:
                raise Exception('cannot find actor type: ', actor_type)

            obj_transform, _ = self.calculate_transform(lane_waypoints[lane_index], start_dist, side_offset, orientation)
            transforms.append(obj_transform)

        # pass results
        self.actor_type_list = actor_names
        self.actor_transform_list = transforms
        self.other_actors = self.scenario_operation.initialize_vehicle_actors(self.actor_transform_list,
                                                                              self.actor_type_list)
        self.reference_actor = self.other_actors[0]  # used for triggering this scenario

    def create_behavior(self, scenario_init_action):
        assert scenario_init_action is None, f'{self.name} should receive [None] action. A wrong scenario policy is used.'

    def update_behavior(self, scenario_action):
        assert scenario_action is None, f'{self.name} should receive [None] action. A wrong scenario policy is used.'

        actor_types = self.parameters[0]
        for i, actor_type in enumerate(actor_types):
            if actor_type == 'human':
                self.scenario_operation.walker_go_straight(self._walker_velocity, i)
            elif actor_type == 'car':
                self.scenario_operation.go_straight(self._car_velocity, i)
    def check_stop_condition(self):
        return False
