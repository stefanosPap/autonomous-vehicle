import carla
import numpy as np
import pandas as pd
import time  
from scipy.interpolate import interp1d
from agents.navigation.controller import VehiclePIDController
from traffic import Traffic
from vehicle_move import spawn
from utilities import change_coordinate_system, plot_axis, draw_vehicle_box
from vehicle import Vehicle
from client import Client
from obstacle_manager import ObstacleManager
from walker_manager import WalkerManager
from experiments import Experiment 
from sensor import CollisionDetector

from communicationMQTT import   VehiclePublisherMQTT, \
                                VehicleSubscriberStartStopMQTT, \
                                VehicleSubscriberVelocityMQTT, \
                                VehicleSubscriberLeftRightMQTT, \
                                VehicleSubscriberPositionMQTT, \
                                VehicleSubscriberChangeGoalMQTT, \
                                VehicleSubscriberAggressiveMQTT, \
                                VehicleSubscriberLawfulMQTT

class Behavior(object):
    def __init__(self, vehicle_actor, waypoints, trajectory, map, world, blueprint, vehicle_list, walker_list, experiment, exp):

        # basic variables initialization         
        self.vehicle_actor     = vehicle_actor
        self.waypoints         = waypoints
        self.velocity          = 0
        self.trajectory        = trajectory
        self.map               = map
        self.world             = world
        self.blueprint         = blueprint
        self.vehicle_list      = vehicle_list
        self.walker_list       = walker_list
        self.start_time        = 0
        self.stop_time         = float("inf")
        self.experiment        = experiment
        self.experiment_object = exp

        '''
        self._dt = 1.0 / 20.0
        args_lateral_dict = {
            'K_P': 1.95,
            'K_D': 0.2,
            'K_I': 0.07,
            'dt': self._dt}
        args_longitudinal_dict = {
            'K_P': 1.0,
            'K_D': 0,
            'K_I': 0.05,
            'dt': self._dt}
        '''
        # controller initialization
        self.custom_controller = VehiclePIDController(vehicle_actor, 
                                                      args_lateral      = {'K_P': 1, 'K_D': 0, 'K_I': 0.0},
                                                      args_longitudinal = {'K_P': 1, 'K_D': 0, 'K_I': 0.0},
                                                      max_throttle=0.8, 
                                                      max_brake=0.4,
                                                      max_steering=0.8)

        self.safe_distances = { 0 :  1.50, 
                                5 :  4.22, 
                                10:  6.72, 
                                15:  9.49, 
                                20: 12.54, 
                                25: 15.86, 
                                30: 19.46, 
                                35: 23.33, 
                                40: 27.49, 
                                45: 31.91,
                                50: 34.50,
                                55: 38.20,
                                60: 41.56,
                                65: 44.82,
                                70: 48.40,
                                75: 52.33,
                                80: 55.40,
                                85: 60.31,
                                90: 65.45,
                                95: 70.32,
                               100: 76.34}

        # obstacle manager initialization 
        self.obstacle_manager = ObstacleManager(map, vehicle_actor, self.vehicle_list, world)
        
        # walker manager initialization 
        self.walker_manager = WalkerManager(map, vehicle_actor, self.walker_list, world)

        #
        # Communication attributes initialization
        # 
        # Publishers
        self.pub_vel         = VehiclePublisherMQTT           (topic='speed_configure'  )
        self.pub_notify      = VehiclePublisherMQTT           (topic='turn notification')
        self.pub             = VehiclePublisherMQTT           (topic='speed_topic'      )
        self.pub_agg         = VehiclePublisherMQTT           (topic='aggressive'       )
        self.pub_law         = VehiclePublisherMQTT           (topic='lawful'           )

        # Subscribers
        self.sub             = VehicleSubscriberStartStopMQTT (topic='start_stop_topic' )
        self.speed_sub       = VehicleSubscriberVelocityMQTT  (topic='speed_configure'  )
        self.turn_sub        = VehicleSubscriberLeftRightMQTT (topic='turn'             )
        self.sub_pos         = VehicleSubscriberPositionMQTT  (topic='position'         )
        self.sub_change_goal = VehicleSubscriberChangeGoalMQTT(topic='change_goal'      )
        self.sub_agg         = VehicleSubscriberAggressiveMQTT(topic='aggressive'       )
        self.sub_law         = VehicleSubscriberLawfulMQTT    (topic='lawful'           )
                
        self.behavior_score       =  0
        self.aggressive_score     =  0
        self.lawful_score         =  0

        self.aggressive_parameter =  1 
        self.lawful_parameter     =  0 
        

        self.convert_aggressive_value                         = interp1d([0, 10 ], [ 20,  10])
        self.convert_lawful_value                             = interp1d([0, 10 ], [-1 ,   1])

        self.convert_offset_value                             = interp1d([0, 100], [1  ,  20])
        self.convert_aggressive_value_for_behaviors           = interp1d([0, 10 ], [1  ,   2])
        self.convert_aggressive_value_for_behaviors_slow_down = interp1d([0, 10 ], [0.2, 0.8])
        self.convert_aggressive_value_for_behaviors_speed_up  = interp1d([0, 10 ], [1.2, 1.6])
        self.convert_value_1                                  = interp1d([0, 10 ], [4  , 2.5])

    def slow_down(self, desired_velocity):
        
        if self.current_velocity > desired_velocity:
            self.velocity = desired_velocity
            #self.publish_velocity()

    def speed_up(self, desired_velocity):
        
        if self.current_velocity < desired_velocity:
            self.velocity = desired_velocity
            #self.publish_velocity()

    def emergency_stop(self):
        
        self.velocity = 0
        #self.publish_velocity()

    def publish_velocity(self):
        self.wait(5)
        self.pub_vel.publish({'velocity': self.velocity})

    def get_current_velocity(self):
        velocity_vector = self.vehicle_actor.get_velocity()
        velocity_array = [velocity_vector.x, velocity_vector.y, velocity_vector.z]
        velocity_norm = np.linalg.norm(velocity_array)
        self.current_velocity = 3.6 * velocity_norm

    def change_lane(self, turn, i, desired_vel):

        if not self.trajectory.change and i != len(self.waypoints):
        
            if self.current_state == "INIT":

                if turn is not None:
                    prev = self.waypoints[self.index]
                    w = self.trajectory.change_waypoint(waypoint=i, direction=turn)

                    if w is not None:
                        self.waypoints[self.index] = w
                        if w == prev:
                            self.current_state = "INIT"
                        else:
                            self.current_state = turn
                            self.pub_notify.publish({'value': self.current_state})
                            self.slow_down(desired_velocity=desired_vel)
                            if turn == "LEFT":
                                self.left_turns += 1
                            elif turn == "RIGHT":
                                self.right_turns += 1
                    else:
                        self.current_state = "INIT"
                    self.turn_sub.set_turn(None)

                else:
                    self.current_state = "INIT"
                    self.turn_sub.set_turn(None)

            elif self.current_state == "LEFT" or self.current_state == "RIGHT":

                if turn == self.current_state or turn == None:   
                   
                    prev = self.waypoints[self.index]
                    if self.current_state == "LEFT":
                        w = self.trajectory.change_waypoint(waypoint=i, direction="LEFT")
                    elif self.current_state == "RIGHT":
                        w = self.trajectory.change_waypoint(waypoint=i, direction="RIGHT")

                    if w != None:
                        self.waypoints[self.index] = w

                        if w == prev or w.lane_type == carla.LaneType.Shoulder:
                            self.waypoints[self.index] = self.waypoints[self.index + self.lane_change_offset]
                            self.index += self.lane_change_offset
                            if self.current_state == "LEFT":
                                self.right_turns += 1
                            elif self.current_state == "RIGHT":
                                self.left_turns += 1
                            self.current_state = "INIT"
                            self.pub_notify.publish({'value': self.current_state})
                            self.slow_down(desired_velocity=desired_vel)
                    else:
                        if self.current_state == "LEFT":
                            self.right_turns += 1
                        elif self.current_state == "RIGHT":
                            self.left_turns += 1
                        self.current_state = "INIT"
                        self.pub_notify.publish({'value': self.current_state})
                        self.slow_down(desired_velocity=desired_vel)

                    self.turn_sub.set_turn(None)

                elif turn != self.current_state:
                    self.current_state = "INIT"
                    self.pub_notify.publish({'value': self.current_state})
                    self.turn_sub.set_turn(None)
                    self.slow_down(desired_velocity=desired_vel)
                    if turn == "LEFT":
                        self.left_turns += 1
                    elif turn == "RIGHT":
                        self.right_turns += 1

    def overtake(self):

        if self.turn_obstacle != None:     
            self.trajectory.change = False
            self.index += self.lane_change_offset
            self.change_lane(self.turn_obstacle, self.index, self.velocity)
            self.overtake_direction = self.turn_obstacle
            self.overtake_completed = False
            self.turn_obstacle = None
            return True
        
        return False
    
    def complete_overtake(self):
        if self.overtake_direction == "RIGHT":
            
            if self.obstacle_manager.closest_rear_left_vehicle != None:               
            
                overtaked_obstacle_distance = self.obstacle_manager.closest_distance_from_rear_left_vehicle
                closest_front_side_obstacle_distance = self.obstacle_manager.closest_distance_from_front_left_vehicle

            else:
                return

        elif self.overtake_direction == "LEFT":
            if self.obstacle_manager.closest_rear_right_vehicle != None:    
                overtaked_obstacle_distance = self.obstacle_manager.closest_distance_from_rear_right_vehicle    
                closest_front_side_obstacle_distance = self.obstacle_manager.closest_distance_from_front_right_vehicle
            
            else:
                return
        
        else:
            return
        
        # first condition is for checking if the overtaked vehicle has been overtaked
        # second condition is for checking if another vehicle exists, in front of the overtaked vehicle

        condition_for_overtaked_vehicle = overtaked_obstacle_distance > self.overtaked_obstacle_distance_threshold and overtaked_obstacle_distance != float('inf')
        condition_for_front_side_vehicle = (closest_front_side_obstacle_distance > self.closest_front_side_obstacle_distance_threshold and closest_front_side_obstacle_distance != float('inf')) or closest_front_side_obstacle_distance == float('inf')

        if condition_for_overtaked_vehicle and condition_for_front_side_vehicle:
            if (self.index + self.lane_change_offset >= len(self.waypoints)):
                return
            if self.overtake_direction == "RIGHT":
                self.turn_obstacle = "LEFT"
            elif self.overtake_direction == "LEFT":
                self.turn_obstacle = "RIGHT"
            else:
                return

            self.overtake()
            self.overtake_direction == None
            self.overtake_completed = True

    def manual_lane_change(self):
        
        # first check is for possible push button but not in the same direction as the current state is (self.turn != self.current_state),
        # second check is for possible lane change, due to the path that has been created by the A*.
        # third check is for non smooth lane change
        
        loc1 = self.waypoints[self.index].transform.location
        loc2 = self.waypoints[self.index + 1].transform.location

        if (self.turn != None and self.turn != self.current_state) or (self.turn == None and self.current_state == "INIT" and loc1.distance(loc2) > 4):
            
            self.trajectory.change = False
            if self.index + self.lane_change_offset < len(self.waypoints):
                self.index += self.lane_change_offset
            else:
                self.turn = None 

        self.change_lane(self.turn, self.index, self.velocity)
    
    def vehicle_in_back(self):
        
        if self.obstacle_manager.closest_distance_from_rear_vehicle < self.rear_obstacle_distance_threshold:
            
            obstacle_detected = self.obstacle_manager.closest_rear_vehicle.id

            if self.previous_back_obstacle_detected != obstacle_detected and "vehicle" in self.obstacle_manager.closest_rear_vehicle.type_id:
            
                self.previous_back_obstacle_detected = obstacle_detected

        #    if not self.action_performed:
                
                print("New back obstacle")
            
                return True
            
        #else:
        #    self.action_performed = False

        return False

    def car_follow(self):

        if self.obstacle_manager.closest_front_vehicle is None:
            return
        
        kd = 0.2
        ku = 0.2

        self.get_front_obstacle_velocity()

        ego_loc = self.vehicle_actor.get_location()
        other_loc = self.obstacle_manager.closest_front_vehicle.get_location()
        
        delay_t = 3.5
        delta_d = ego_loc.distance(other_loc)
        delta_u = self.current_velocity - self.front_obstacle_velocity

        acc = kd * delta_d + ku * delta_u
        
        target_speed = self.current_velocity + acc * delay_t

        if target_speed != self.current_velocity:
            self.velocity = target_speed / 3.6 
            #self.publish_velocity()

    def get_front_obstacle_velocity(self):

        velocity_vec = self.obstacle_manager.closest_front_vehicle.get_velocity()
        velocity_obs_array = [velocity_vec.x, velocity_vec.y, velocity_vec.z]
        self.front_obstacle_velocity = np.linalg.norm(velocity_obs_array)
        self.front_obstacle_velocity = round(3.6 * self.front_obstacle_velocity, 1)


    def get_rear_obstacle_velocity(self):

        velocity_vec = self.obstacle_manager.closest_rear_vehicle.get_velocity()
        velocity_obs_array = [velocity_vec.x, velocity_vec.y, velocity_vec.z]
        self.rear_obstacle_velocity = np.linalg.norm(velocity_obs_array)
        self.rear_obstacle_velocity = round(3.6 * self.rear_obstacle_velocity, 1)
    
    # --------------------------------------------------------------------------- #
    # |                                                                         | #
    # |                          Rules been checked                             | #
    # |                                                                         | #
    # --------------------------------------------------------------------------- #

    # 1st rule
    def has_ego_higher_velocity_than_front_obstacle(self):

        if self.obstacle_manager.closest_front_vehicle is None:
            return False

        self.get_front_obstacle_velocity()
        if self.current_velocity > self.front_obstacle_velocity:
            return True
        
        return False

    # 2nd rule
    def vehicle_in_front(self):
        self.still_front = False
        if self.obstacle_manager.closest_distance_from_front_vehicle < self.front_obstacle_distance_threshold:
            if abs(self.vehicle_actor.get_transform().rotation.yaw - self.obstacle_manager.closest_front_vehicle.get_transform().rotation.yaw) > 10:
                self.still_front = False
                return False

            self.obstacle_detected = self.obstacle_manager.closest_front_vehicle.id
            
            if self.previous_front_obstacle_detected != self.obstacle_detected and "vehicle" in self.obstacle_manager.closest_front_vehicle.type_id:
            #if not self.action_performed:
                print("New obstacle in front")
                self.start_time = time.time()
                self.previous_front_obstacle_detected = self.obstacle_detected
                
                return True
            elif self.previous_front_obstacle_detected == self.obstacle_detected:
                self.still_front = True 

        if self.obstacle_manager.closest_front_vehicle == None or time.time() - self.start_time > 38:
            self.previous_front_obstacle_detected = None
        
        return False

    # 3rd rule
    def exists_intersection(self, meters):

        self.intersection = False

        # check for intersection
        if 1 in self.junctions[self.index:]:
            index = self.junctions[self.index:].index(1)
        else:
            index = float("inf")
        
        if index < meters: 
            self.intersection = True

        # check for intersection closely 
        if self.intersection:
            return True
    
        return False
    
    
    # 4th rule
    def exists_stop(self, meters):

        self.stop_sign = False
        distance = float("inf")
        if len(self.traffic_signs) != 0:
        
            for i in range(len(self.traffic_signs)):
                
                #check for stop sign
                if self.traffic_signs[i].type == "206":
                    self.stop_detected = self.traffic_signs[i].id

                    orientation = str(self.traffic_signs[i].orientation)
                    distance = self.traffic_signs[i].distance
                    if self.waypoints[self.index].lane_id >= 0 and (orientation == "Positive" or orientation == "Both"):
                        self.stop_sign = True
                        self.stop_sign_id = self.traffic_signs[i].id

                    elif self.waypoints[self.index].lane_id < 0 and (orientation == "Negative" or orientation == "Both"):
                        self.stop_sign = True
                        self.stop_sign_id = self.traffic_signs[i].id
                
        # check for stop sign closely 
        if self.stop_sign and distance < meters:    
            if self.previous_stop_detected != self.stop_detected:
                self.stop_time = time.time()
                self.previous_stop_detected = self.stop_detected
            return True
    
        return False
    

    # 5th rule
    def is_end_location_closely(self):
        current_loc = self.waypoints[self.index].transform.location
        end_location = self.waypoints[len(self.waypoints) - 1].transform.location
        end_distance = current_loc.distance(end_location)
        if end_distance < 7:
            return True 
        return False

    # 6th rule
    def is_front_free_for_a_while(self):
        
        if self.end_trigger_waypoint_number - self.start_trigger_waypoint_number > (25 / self.convert_aggressive_value_for_behaviors(self.aggressive_score)) and not self.vehicle_in_front(): 
            self.start_trigger_waypoint_number = self.index
            return True
        return False

    # 7th rule
    def red_traffic_light(self):

        red_traffic_light = False
        self.traffic_light = False        
        self.traffic_light_object = None
        values = list(self.traffic_signs_active.values())[0] 

        if values != []:            
                
            for i in range(len(values)):
            
                #check for traffic light
                if values[i].type == "1000001":
                    self.front_location = self.vehicle_actor.get_location() + carla.Location(self.vehicle_actor.bounding_box.extent.x, 0, 0)
                    self.rear_location  = self.vehicle_actor.get_location() - carla.Location(self.vehicle_actor.bounding_box.extent.x, 0, 0)
                    self.front_location = carla.Location(self.front_location.x, self.front_location.y, self.front_location.z)
                    self.rear_location = carla.Location(self.rear_location.x, self.rear_location.y, self.rear_location.z)
                                    
                    orientation = str(values[i].orientation)
                    if self.waypoints[self.index].lane_id >= 0 and (orientation == "Positive" or orientation == "Both"):
                        self.traffic_light = True 
                        self.traffic_light_object = self.world.get_traffic_light(values[i])
                        self.traffic_light_id = values[i].id
                            
                    elif self.waypoints[self.index].lane_id < 0 and (orientation == "Negative" or orientation == "Both"):
                        self.traffic_light = True         
                        self.traffic_light_object = self.world.get_traffic_light(values[i])
                        self.traffic_light_id = values[i].id

        
        if self.traffic_light:
            if self.traffic_light_object.get_state() == carla.TrafficLightState.Red:
                red_traffic_light = True

        #if self.front_location.distance((values[i].transform.location)) > self.rear_location.distance((values[i].transform.location)):
        #    print("Attt:", self.index)
        #    return False

        #if self.traffic_light_state == "RED" or red_traffic_light:
        if red_traffic_light:
            return True
        return False

    # 8th rule
    def check_zero_velocity(self):
        self.get_current_velocity()
        
        if self.current_velocity < 0.001 and self.index > 3:
            self.start_trigger_waypoint_number = self.index
            return True
        return False

    # 9th rule
    def is_right_lane_safe(self):
        
        if self.waypoints[self.index].get_right_lane() == None:
            return False

        if self.current_state == "RIGHT":
            return False

        if self.obstacle_manager.closest_distance_from_rear_right_vehicle  < self.rear_right_vehicle_threshold:
            return False

        if self.obstacle_manager.closest_distance_from_front_right_vehicle < self.front_right_vehicle_threshold:
            return False

        return True

    # 10th rule
    def is_left_lane_safe(self):

        if self.current_state == "LEFT":
            return False

        if self.waypoints[self.index].get_left_lane() == None:
            return False

        if self.obstacle_manager.closest_distance_from_rear_left_vehicle  < self.rear_left_vehicle_threshold:
            return False

        if self.obstacle_manager.closest_distance_from_front_left_vehicle < self.front_left_vehicle_threshold:
            return False
        
        return True
        
    # 11th rule
    def has_left_lane_different_direction(self):
        if self.current_state == "LEFT":
            return False

        if self.waypoints[self.index].get_left_lane() != None:
            if self.waypoints[self.index].get_left_lane().lane_type != carla.LaneType.Bidirectional:
                sign = self.waypoints[self.index].get_left_lane().lane_id * self.waypoints[self.index].lane_id
                if sign < 0:
                    return True
        return False
    
    # 12th rule 
    def is_right_lane_denser_than_left(self):
        if len(self.obstacle_manager.vehicles_in_right_lane) > len(self.obstacle_manager.vehicles_in_left_lane) and self.current_state == "INIT":
            return True
        return False

    # 13th rule
    def is_left_lane_denser_than_right(self):
        if len(self.obstacle_manager.vehicles_in_left_lane) > len(self.obstacle_manager.vehicles_in_right_lane) and self.current_state == "INIT":
            return True
        return False

    # 14th rule 
    def is_right_pedestrian_closely(self):
        if self.walker_manager.closest_distance_from_front_right_walker < 10:
            return True
        return False

    # 15th rule
    def is_left_pedestrian_closely(self):
        if self.walker_manager.closest_distance_from_front_left_walker  < 10:
            return True
        return False

    # 16th rule
    def is_front_pedestrian_closely(self):
        if self.walker_manager.closest_distance_from_front_walker  < 10:
            return True
        return False
    
    # 17th rule    
    def is_right_lane_marking_not_legal(self):
        if self.current_state == "RIGHT":
            return False
        
        self.illegal_right_lane_marking = ("Solid" in str(self.waypoints[self.index].right_lane_marking.type) and ("Right" not in str(self.waypoints[self.index].lane_change) or "Both" not in str(self.waypoints[self.index].lane_change)))
        if self.illegal_right_lane_marking:
            return True
        return False

    # 18th rule    
    def is_left_lane_marking_not_legal(self):
        if self.current_state == "LEFT":
            return False
        
        self.illegal_left_lane_marking = ("Solid" in str(self.waypoints[self.index].left_lane_marking.type) and ("Left" not in str(self.waypoints[self.index].lane_change) or "Both" not in str(self.waypoints[self.index].lane_change)))
        if self.illegal_left_lane_marking:
            return True
        return False

    
    # 19th rule
    def is_in_bidirectional_lane_for_long_time(self):
        if self.bidirectional_end_trigger_waypoint_number - self.bidirectional_start_trigger_waypoint_number > 10: 
            self.bidirectional_start_trigger_waypoint_number = self.index
            self.is_at_bidirectional = False
            return True
        return False

    # 20th rule
    def is_left_lane_bidirectional(self):
        if self.current_state == "LEFT":
            return False
        
        if self.waypoints[self.index].get_left_lane() == None:
            return False

        if self.waypoints[self.index].get_left_lane().lane_type == carla.LaneType.Bidirectional:
            return True
        return False

    # 21th rule
    def speed_is_above_the_limit(self):

        self.speed_limit_sign = False
        orientation = None
        value = float("inf")
        try:
            if len(self.traffic_signs) is not 0:
            
                for i in range(len(self.traffic_signs)):
                    
                    #check for speed limit sign sign
                    if self.traffic_signs[i].type == "274":
                        self.speed_limit_sign = True
                        self.speed_limit_id = self.traffic_signs[i].id
                        orientation = str(self.traffic_signs[i].orientation)
                        value = 14 # value is set to 14 for better results, real values are really high 
                        #value = self.traffic_signs[i].value 
                        self.lane_limit[self.waypoints[self.index].lane_id] = value
                        self.lane_orientation[self.waypoints[self.index].lane_id] = orientation


            elif self.lane_limit[self.waypoints[self.index].lane_id] != float("inf"):
                self.speed_limit_sign = True
                value = 14 # value is set to 14 for better results, real values are really high 
                #value = self.lane_limit[self.waypoints[self.index].lane_id]
                orientation = self.lane_orientation[self.waypoints[self.index].lane_id]
        except:
            pass 
        velocity_vector = self.vehicle_actor.get_velocity()
        velocity_array = [velocity_vector.x, velocity_vector.y, velocity_vector.z]
        velocity_norm = np.linalg.norm(velocity_array)

        if self.speed_limit_sign:
            if self.waypoints[self.index].lane_id >= 0 and (orientation == "Positive" or orientation == "Both"):
                if self.current_velocity > value:
                    return True

            elif self.waypoints[self.index].lane_id < 0 and (orientation == "Negative" or orientation == "Both"):
                if round(3.6 * velocity_norm, 1) > value:
                    return True
        return False
    
    # 22th rule 
    def automatic_right_lane_change(self):
        if self.current_state == "RIGHT":
            return False
        if self.index + self.lane_change_offset < len(self.waypoints):
            final_point = change_coordinate_system(self.waypoints[self.index].transform, self.waypoints[self.index + self.lane_change_offset].transform.location)
        else:
            return False
        loc1 = self.waypoints[self.index].transform.location
        loc2 = self.waypoints[self.index + 1].transform.location

        if (self.turn != None and self.turn != self.current_state) or (self.turn == None and self.current_state == "INIT" and loc1.distance(loc2) > 4):
            if (final_point.x < 0 and final_point.y < 0) or (final_point.x > 0 and final_point.y > 0):
                return True
        return False
    
    # 23th rule 
    def automatic_left_lane_change(self):
        if self.current_state == "LEFT":
            return False
        if self.index + self.lane_change_offset < len(self.waypoints):
            final_point = change_coordinate_system(self.waypoints[self.index].transform, self.waypoints[self.index + self.lane_change_offset].transform.location)
        else:
            return False

        loc1 = self.waypoints[self.index].transform.location
        loc2 = self.waypoints[self.index + 1].transform.location

        if (self.turn != None and self.turn != self.current_state) or (self.turn == None and self.current_state == "INIT" and loc1.distance(loc2) > 4):
            if (final_point.x > 0 and final_point.y < 0) or (final_point.x < 0 and final_point.y > 0):
                return True
        return False
    
    # 24th rule 
    def is_vehicle_still_front(self):
        if self.still_front == True:
            return True 
        return False

    # 25th rule
    def is_junction_in_front_of_stop_free(self):
        self.free_junction = True
        if self.exists_stop(self.lawful_score + self.convert_value_1(self.aggressive_score) * int(self.convert_offset_value(self.current_velocity))):

            junction = None
            for waypoint in self.waypoints[self.index:]:
                if waypoint.is_junction:
                    junction = waypoint.get_junction()
                    break

            for vehicle in self.obstacle_manager.front_general_vehicles:
                if junction != None:
                    loc = vehicle.get_location()
                    wp = self.map.get_waypoint(loc, project_to_road=False, lane_type=carla.LaneType.Any)
                    if wp.is_junction:
                        junction2 = wp.get_junction()
                        if junction.id == junction2.id:
                            self.free_junction = False

        if self.exists_stop(self.lawful_score + self.convert_value_1(self.aggressive_score) * int(self.convert_offset_value(self.current_velocity))) and time.time() - self.stop_time > 10 and self.free_junction:
            return True 
        return False

    # --------------------------------------------------------------------------- #
    # |                                                                         | #
    # |              Help functions for calculating the rules                   | #
    # |                                                                         | #
    # --------------------------------------------------------------------------- #
    def check_if_junction_exists(self):
    
        self.junctions = []
        for i in range(1, len(self.waypoints) - 1):
            
            if self.waypoints[i].is_junction and not self.waypoints[i - 1].is_junction:
                self.junctions.append(True)
                self.world.debug.draw_string(self.waypoints[i].transform.location, "AA", draw_shadow=False, color=carla.Color(r=0, g=0, b=255), life_time=1000, persistent_lines=True)
                
            else:
                self.junctions.append(False)

    # --------------------------------------------------------------------------- #
    # |                                                                         | #
    # |             Functions for calculating sliders' score                    | #
    # |                                                                         | #
    # --------------------------------------------------------------------------- #

    def lawful(self):

        self.lawful_score = self.sub_law.get_lawful()

    def aggressive(self):
        
        self.aggressive_score = self.sub_agg.get_aggressive()
                
    def calculate_safe_distance(self):
        
        self.safe_distance = -float("inf")
        
        if self.obstacle_manager.closest_front_vehicle != None:
            velocity_vector = self.obstacle_manager.closest_front_vehicle.get_velocity()
            velocity_array = [velocity_vector.x, velocity_vector.y, velocity_vector.z]
            other_velocity = np.linalg.norm(velocity_array)
            other_velocity = round(3.6 * other_velocity, 1)

            diff = self.current_velocity - other_velocity
            if 0 <= diff <= 100: 
                keys = list(self.safe_distances.keys())
                values = list(self.safe_distances.values())

                inter_function = interp1d(keys, values)
                self.safe_distance = max(10, float(inter_function(diff)))
    
    def check_new_triggers(self):
                
        #self.difference_in_waypoints = False
        #if self.end_trigger_waypoint_number - self.start_trigger_waypoint_number > 2: 
        #    self.start_trigger_waypoint_number = self.index
        #    self.difference_in_waypoints = True

        #if self.max_behavior == "OVERTAKE":
        #    if not self.action_completed:
        #        self.new_trigger = False
        #        return

        if list(self.row_filter[0]) != list(self.previous_filter[0]) and list(self.previous_filter[0]) != []:
            self.new_trigger = True
            self.action_performed = False
            return
        
        if self.sub_agg.get_change_aggressive() or self.sub_law.get_change_lawful():
            self.new_trigger = True
            self.action_performed = False
            return

        self.new_trigger = False

    def evaluate(self):

        if self.new_trigger:
            #aggressive_param = self.convert_aggressive_value_for_behaviors(self.aggressive_score)
            
            #self.updated_behaviors["OVERTAKE"         ][:] = self.updated_behaviors["OVERTAKE"         ][:] * aggressive_param
            #self.updated_behaviors["LEFT_LANE_CHANGE" ][:] = self.updated_behaviors["LEFT_LANE_CHANGE" ][:] * aggressive_param
            #self.updated_behaviors["RIGHT_LANE_CHANGE"][:] = self.updated_behaviors["RIGHT_LANE_CHANGE"][:] * aggressive_param
            #self.updated_behaviors["SPEED_UP"         ][:] = self.updated_behaviors["SPEED_UP"         ][:] * aggressive_param
            
            values = self.updated_behaviors.loc[:].values
            sum_values = values.sum(0)
            
            #max_complete_behaviors_value = max(sum_values[0:2])
            max_direction_behavior_value = max(sum_values[2:5])
            max_speed_behavior_value     = max(sum_values[5: ])
            
            #max_complete_behaviors_key = np.argmax(sum_values[0:2], 0)
            #self.max_complete_behavior = self.updated_behaviors.columns[0:2][max_complete_behaviors_key]
            
            max_direction_behavior_key = np.argmax(sum_values[2:5], 0)
            self.max_direction_behavior = self.updated_behaviors.columns[2:5][max_direction_behavior_key]

            max_speed_behavior_key = np.argmax(sum_values[5:], 0)
            self.max_speed_behavior = self.updated_behaviors.columns[5:][max_speed_behavior_key]

            #if max_complete_behaviors_value > max_direction_behavior_value:
            #    self.max_behavior = self.max_complete_behavior 
            #    self.max_speed_behavior = None
            #    if max_complete_behaviors_value == 0.0:
            #        self.max_behavior = "KEEP_STRAIGHT"

            #else:
            self.max_behavior = self.max_direction_behavior
            if max_direction_behavior_value == 0.0:
                self.max_behavior = "KEEP_STRAIGHT"
            if max_speed_behavior_value == 0.0:
                self.max_speed_behavior = "KEEP_VELOCITY"
            print("----------------------------")
            print(sum_values)
            print(self.row_activation_filter)
            print(self.max_behavior, self.max_speed_behavior)
            print(set([a*b for a,b in zip(list(self.row_activation_filter[0]),self.indexes)]))
            print("converted_lawful_score:", self.convert_lawful_value(self.lawful_score), "lawful_score:", self.lawful_score)
            print("converted_aggressive_score:", self.convert_aggressive_value_for_behaviors_speed_up(self.aggressive_score), "aggressive_score:", self.aggressive_score)
            print("----------------------------")

    def regulate_speed(self):
      
        if self.max_speed_behavior == "SLOW_DOWN":
            if not self.action_performed:

                speed = self.convert_aggressive_value_for_behaviors_slow_down(self.aggressive_score) * self.current_velocity
                print(self.current_velocity)
                speed = max(12, speed)    
                
                if (self.vehicle_in_front() or self.still_front) and self.max_behavior == "KEEP_STRAIGHT":
                    self.get_front_obstacle_velocity()
                    speed = self.front_obstacle_velocity
                
                if self.is_front_pedestrian_closely():
                    speed = 3 

                self.slow_down(speed)
                self.action_performed = True
                
        if self.max_speed_behavior == "SPEED_UP":
            if not self.action_performed:
                
                speed = self.convert_aggressive_value_for_behaviors_speed_up(self.aggressive_score) * self.current_velocity
                speed = min(speed, 38)
                speed = max(speed, 12)
                print(self.current_velocity)

                self.speed_up(speed)
                self.action_performed = True

        if self.max_speed_behavior == "STOP":
            if not self.action_performed:    
                
                # if RED light has activated when the vehicle was inside the junction then it is better to get out of the junction
                loc = self.vehicle_actor.get_location()
                wp = self.map.get_waypoint(loc, project_to_road=False, lane_type=carla.LaneType.Any)
                if not wp.is_junction:
                    self.emergency_stop()
                    self.world.debug.draw_string(wp.transform.location, "H", draw_shadow=False, color=carla.Color(r=200, g=0, b=0), life_time=1000, persistent_lines=True)
                else:
                    self.control_signal = self.custom_controller.run_step(self.velocity, self.waypoints[self.index])
                self.wait(10)
                self.action_performed = True

        if self.max_speed_behavior == "KEEP_VELOCITY":
            self.get_current_velocity() 
            if self.current_velocity < 0.5:
                self.speed_up(12)

    def wait(self, reps):
        for _ in range(reps):
            self.world.tick() 
    
    def follow_trajectory(self, world, vehicle_actor, spectator, get_front_obstacle, set_front_obstacle, get_other_actor, velocity):
        
        self.index         = 0
        self.current_state = "INIT"
        
        self.overtake_started   = False
        self.overtake_completed = False
        self.overtake_direction = None

        self.turn          = None
        self.turn_obstacle = None
        
        self.previous_front_obstacle_detected = None
        self.previous_back_obstacle_detected  = None     

        self.action_performed = False
        car_follow_behavior   = False
        
        self.stop_sign     = False
        self.traffic_light = False
        self.intersection  = False

        '''
        walker_list = []
        walker_loc = carla.Location(x=60.551256, y=-198, z=1)
        walker_rot = carla.Rotation(0, 45, 0)
        walker_trans = carla.Transform(walker_loc, walker_rot)
        walker = self.blueprint.filter("walker.pedestrian.0001")[0]      
        
        self.walker_actor = self.world.spawn_actor(walker, walker_trans)
        self.walker_list.append(self.walker_actor)
        
        walker_control = carla.WalkerControl(direction=carla.Vector3D(0.0, 1.0, 0.0), speed=0.2)
        
        self.walker_actor.apply_control(walker_control)
        '''
        '''
        start_point = carla.Transform(carla.Location(x=156.617218, y=-20.668285, z=0.980615), carla.Rotation(pitch=-5.188444, yaw=91.1591, roll=0.288564))

        thr = 0.2
        vehicle = Vehicle()                                  
        vehicle.choose_spawn_point(start_point)                 # spawn the vehicle 
        vehicle.choose_model('model3', self.blueprint, world)
        vehicle_actor1 = vehicle.get_vehicle_actor()
        control_signal1 = carla.VehicleControl(throttle=thr)
        #vehicle_actor1.apply_control(control_signal1)
        self.vehicle_list.append(vehicle_actor1)
        self.wait(10)
        #vehicle_actor1.set_autopilot(True)
        self.wait(10)
        '''
        '''
        start_point = carla.Transform(carla.Location(x=44.551256, y=-193, z=1), carla.Rotation(pitch=360.000, yaw=1.439560, roll=0.0))

        thr = 0.2
        vehicle = Vehicle()                                  
        vehicle.choose_spawn_point(start_point)                 # spawn the vehicle 
        vehicle.choose_model('model3', self.blueprint, world)
        vehicle_actor1 = vehicle.get_vehicle_actor()
        control_signal1 = carla.VehicleControl(throttle=thr)
        #vehicle_actor1.apply_control(control_signal1)
        self.vehicle_list.append(vehicle_actor1)
        self.wait(10)
        vehicle_actor1.set_autopilot(True)
        self.wait(10)
        
    
        start_point = carla.Transform(carla.Location(-74, 0,  1), carla.Rotation(pitch=0.0, yaw=-90.0, roll=0.0))
        thr = 0.2
        vehicle = Vehicle()                                  
        vehicle.choose_spawn_point(start_point)                 # spawn the vehicle 
        vehicle.choose_model('model3', self.blueprint, world)
        vehicle_actor1 = vehicle.get_vehicle_actor()
        control_signal1 = carla.VehicleControl(throttle=thr)
        #vehicle_actor1.apply_control(control_signal1)
        self.vehicle_list.append(vehicle_actor1)
        self.wait(10)
        #vehicle_actor1.set_autopilot(True)
        self.wait(10)
        '''

        
        self.action = False
        

        behaviors = {
                    "OVERTAKE"          : [ 0.4,  0.7, -0.3, -0.4, -0.9,  0.0,  0.0,  0.0,  0.0,  0.6,  0.3,  0.0, -0.5,  0.0, -0.5,  0.0,  0.0,  0.4,  0.0, -0.1,  0.0,  0.0,  0.0,  0.0,   0.0,  0.0,  0.0],
                    "CAR_FOLLOW"        : [-0.3, -0.3,  0.9,  0.1,  0.2,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.4,  0.0,  0.0,  0.0,  0.0,   0.0,  0.0,  0.0], 

                    "LEFT_LANE_CHANGE"  : [ 0.2,  0.4, -0.2, -0.3, -0.2,  0.0,  0.0,  0.0,  0.0,  0.7, -0.5,  0.5, -0.2,  0.4, -0.5,  0.1,  0.4, -0.4,  0.0, -0.3,  0.0,  0.0, -0.3,  0.1,   0.2, -0.5,  0.0],
                    "RIGHT_LANE_CHANGE" : [ 0.2,  0.4, -0.2, -0.3, -0.2,  0.0,  0.0,  0.0,  0.7,  0.0,  0.5, -0.2,  0.5, -0.5,  0.4,  0.1, -0.4,  0.4,  0.7,  0.4,  0.0, -0.3,  0.0,  0.1,  -0.5,  0.2,  0.0], 
                    "KEEP_STRAIGHT"     : [-0.1, -0.1,  0.8,  0.7,  0.5,  0.5,  0.5,  0.0,  0.0,  0.0,  0.5,  0.4,  0.4,  0.2,  0.2,  0.2,  0.4,  0.4,  0.5,  0.4,  0.0,  0.5,  0.5,  0.2,   0.4,  0.4,  0.3],
                        
                    "SPEED_UP"          : [-0.5, -0.1, -0.2, -0.3, -0.2,  0.8, -0.7,  0.6,  0.0,  0.0,  0.0,  0.0,  0.0,  0.1,  0.1, -0.5,  0.0,  0.0,  0.2,  0.0, -0.8, -0.1, -0.1, -0.1,  -0.2, -0.2,  0.4], 
                    "SLOW_DOWN"         : [ 0.8,  0.8,  0.7,  0.4,  0.9, -0.5,  0.2, -0.6,  0.0,  0.0,  0.0,  0.0,  0.0,  0.2,  0.2,  0.5,  0.0,  0.0,  0.1,  0.0,  0.5,  0.3,  0.3,  0.3,   0.2,  0.2, -0.1],
                    "KEEP_VELOCITY"     : [-0.1, -0.8,  0.4,  0.3,  0.5,  0.5, -0.4, -0.3,  0.0,  0.0,  0.0,  0.0,  0.0,  0.3,  0.3,  0.4,  0.0,  0.0,  0.3,  0.0,  0.2,  0.1,  0.1,  0.2,   0.1,  0.1,  0.0],
                    "STOP"              : [ 0.5,  0.6,  0.6,  0.6, -0.1, -0.5,  0.9, -0.6, -0.2, -0.2,  0.0,  0.0,  0.0,  0.1,  0.1,  0.6,  0.0,  0.0, -0.4,  0.0,  0.1,  0.4,  0.4,  0.1,   0.2,  0.2, -0.2]
                       
                    }

        columns = [ "OVERTAKE",
                    "CAR_FOLLOW",
                    "LEFT_LANE_CHANGE",
                    "RIGHT_LANE_CHANGE",
                    "KEEP_STRAIGHT",
                    "SPEED_UP",
                    "SLOW_DOWN",
                    "KEEP_VELOCITY",
                    "STOP"]

        self.indexes = ["ego has higher velocity than the obstacle's velocity",
                        "close front vehicle exists", 
                        "close distance from intersection",
                        "close distance from stop sign",
                        "close distance from the end", 
                        "front area is free",
                        "red light", 
                        "zero velocity",
                        "right lane safe",
                        "left lane safe",
                        "left lane not same direction",
                        "right lane denser than left",
                        "left lane denser than right",
                        "right pedestrian closely",
                        "left pedestrian closely",
                        "front pedestrian closely",
                        "right lane marking not legal",
                        "left lane marking not legal",
                        "in bidirectional for long time",
                        "is left lane bidirectional",
                        "speed above the limit",
                        "make automatic right lane change",
                        "make automatic left lane change",
                        "still front vehicle",
                        "right lane not safe",
                        "left lane not safe",
                        "junction in stop is free"]
   
        behaviors = pd.DataFrame(behaviors, index=self.indexes)
        self.max_behavior = "KEEP_STRAIGHT"
        self.max_speed_behavior = "KEEP_VELOCITY"
        
        self.new_trigger = False

        self.previous_filter = np.array([[]])
        self.start_trigger_waypoint_number = self.index
        self.is_at_bidirectional = False
        self.bidirectional_end_trigger_waypoint_number = 0
        self.bidirectional_start_trigger_waypoint_number = 0

        self.lane_limit = {self.waypoints[self.index].lane_id:  float("inf")}
        self.lane_orientation = {self.waypoints[self.index].lane_id:  None}

        self.check_if_junction_exists()
        self.traffic_signs_active = dict()
        self.still_front = False
        self.previous_stop_detected = None 

        self.right_turns        = 0
        self.left_turns         = 0 
        self.average_speed      = 0 
        self.overall_speed      = 0
        self.route_completion   = 0

        loc = self.vehicle_actor.get_location()
        vec = [loc.x, loc.y]
        self.locations = [vec]
        
        self.velocity = 20
        aggressive  = self.experiment['aggresssive']
        lawful      = self.experiment['lawful']
        self.wait(15)

        #self.publish_velocity()
        self.pub_agg. publish ({'aggressive': aggressive  })
        self.pub_law. publish ({'lawful'    : lawful      })
        self.wait(15)

        self.previous_off_road_event_time = 0
        self.route_completion = 0 
        self.off_road_event_time = 0 
        self.pedestrian_collision = 0
        self.vehicle_collision = 0 
        self.static_obstacle_collision = 0
        self.red_light_violations = 0
        self.stop_sign_violations = 0
        self.speed_limit_violations = 0

        self.start_time = time.time()

        bp = self.world.get_blueprint_library().find('sensor.other.collision')
        self.collision_detector = self.world.spawn_actor(bp, carla.Transform(), attach_to=self.vehicle_actor)
        self.prev_time = 0
        self.zero_vel = 0
        self.zero_vel_light = 0
        self.high_vel = 0
        self.prev_traffic_light_id = None
        self.prev_speed_limit_id   = None
        self.prev_stop_sign_id     = None
        self.traffic_light_id = None 
        self.speed_limit_id = None
        self.stop_sign_id     = None

        self.check_collissions()
        while True:
            
            try:
                
                #alive_actors = [i for i in self.world.get_actors() if i.is_alive]
                #self.vehicle_list = [i for i in self.vehicle_list if i.is_alive]
                
                self.get_current_velocity()

                # spectator method is called in order to place the view of the simulator exactly above the vehicle 
                # spectator()

                # corner case that is being executed, when the vehicle reaches its destination
                if self.index == len(self.waypoints) - 1:
                    self.overall_speed += self.current_velocity
                    self.world.debug.draw_string(self.waypoints[self.index].transform.location, "X", draw_shadow=False, color=carla.Color(r=0, g=0, b=255), life_time=1000, persistent_lines=True)
                    
                    self.control_signal = self.custom_controller.run_step(self.velocity, self.waypoints[self.index])
                    vehicle_actor.apply_control(self.control_signal)
                    self.wait(5)
                    self.velocity = 0
                    self.control_signal = self.custom_controller.run_step(self.velocity, self.waypoints[self.index])
                    vehicle_actor.apply_control(self.control_signal)
                    #self.publish_velocity()

                    # calculate average speed
                    self.average_speed = self.overall_speed / self.index 
                    self.off_road_event_time = 1 - self.off_road_event_time / (time.time() - self.start_time)

                    print("Route:"          ,   self.route_completion           )
                    print("Time percentage:",   self.off_road_event_time        )
                    print("Ped:"            ,   self.pedestrian_collision       )
                    print("Veh:"            ,   self.vehicle_collision          )
                    print("Stat:"           ,   self.static_obstacle_collision  )
                    print("Red:"            ,   self.red_light_violations       )
                    print("Stop:"           ,   self.stop_sign_violations       )
                    print("Speed:"          ,   self.speed_limit_violations     )
                    print("Right:"          ,   self.right_turns                )
                    print("Left:"           ,   self.left_turns                 )
                    print("Avg:"            ,   self.average_speed              )

                    x_l = [cor[0] for cor in self.locations]
                    y_l = [cor[1] for cor in self.locations]
                    
                    self.waypoints = [way.transform.location for way in self.waypoints]
                    self.waypoints = [[way.x, way.y] for way in self.waypoints]
                    
                    x_w = [cor[0] for cor in self.waypoints]
                    y_w = [cor[1] for cor in self.waypoints]

                    open('data_coordinates.txt', 'w').close()
                    data_file = open('data_coordinates.txt', 'a')
                    data_file.write(str(x_w))
                    data_file.write('\n')
                    data_file.write(str(y_w))
                    data_file.write('\n')
                    data_file.write(str(x_l))
                    data_file.write('\n')
                    data_file.write(str(y_l))
                    data_file.write('\n')
                    data_file.close()

                    self.evaluate_node()
                    break               
                
                # velocity's norm in km/h - publish the current velocity in the speedometer
                self.pub.publish({'velocity': round(self.current_velocity, 1)})

                # --------------------------------------------------------------------------- #
                # |                                                                         | #
                # |                        Vehicle's Behavior Definition                    | #
                # |                                                                         | #
                # --------------------------------------------------------------------------- #

                if self.sub_agg.get_change_aggressive():
                    self.aggressive()

                if self.sub_law.get_change_lawful():
                    self.lawful()


                self.calculate_safe_distance()
                self.front_obstacle_distance_threshold = self.safe_distance



                self.overtaked_obstacle_distance_threshold = self.convert_aggressive_value(self.aggressive_score) 
                self.closest_front_side_obstacle_distance_threshold = self.convert_aggressive_value(self.aggressive_score)
                self.lane_change_offset = int(self.convert_offset_value(self.current_velocity))
                self.rear_obstacle_distance_threshold = self.convert_aggressive_value(self.aggressive_score) / 2
                
                self.rear_right_vehicle_threshold  = self.convert_aggressive_value(self.aggressive_score) / 2
                self.front_right_vehicle_threshold = self.convert_aggressive_value(self.aggressive_score)
                self.rear_left_vehicle_threshold   = self.convert_aggressive_value(self.aggressive_score) / 2
                self.front_left_vehicle_threshold  = self.convert_aggressive_value(self.aggressive_score)

                self.end_trigger_waypoint_number = self.index

                if self.waypoints[self.index].lane_type == carla.LaneType.Bidirectional and self.is_at_bidirectional:
                    
                    self.bidirectional_end_trigger_waypoint_number = self.index

                elif self.waypoints[self.index].lane_type == carla.LaneType.Bidirectional and not self.is_at_bidirectional:
                    
                    self.bidirectional_start_trigger_waypoint_number = self.index
                    self.is_at_bidirectional = True
                
                if self.waypoints[self.index].lane_id not in list(self.lane_limit.keys()):
                    self.lane_limit[self.waypoints[self.index].lane_id] = float("inf")
                    self.lane_orientation[self.waypoints[self.index].lane_id] = None
                
                # --------------------------------------------------------------------------- #
                # |                                                                         | #
                # |                           Vehicle's Perception                          | #
                # |                                                                         | #
                # --------------------------------------------------------------------------- #

                # initialize traffic manager in order to handle traffic lights and traffic signs 
                traffic = Traffic(world, self.map)
                self.traffic_signs = traffic.check_signs(self.waypoints[self.index])
                
                if self.index not in self.traffic_signs_active.keys():
                    self.traffic_signs_active[self.index] = self.traffic_signs
                    if self.index - 1 in self.traffic_signs_active.keys():
                        self.traffic_signs_active.pop(self.index - 1)

                    if self.index - self.lane_change_offset in self.traffic_signs_active.keys():
                        self.traffic_signs_active.pop(self.index - self.lane_change_offset )

                self.traffic_light_state = traffic.check_traffic_lights(self.vehicle_actor)
                #traffic.get_lane_info(self.waypoints[self.index])

                #----------------------------#
                #|                          |#
                #|      check obstacles     |#
                #|                          |# 
                #----------------------------#

                # call obstacle manager for checking front and rear obstacles                                         
                self.obstacle_manager.check_obstacles()
                if self.obstacle_manager.closest_front_vehicle != None:
                    loc = self.obstacle_manager.closest_front_vehicle.get_location()
                    trans = self.obstacle_manager.closest_front_vehicle.get_transform()
                    rot = trans.rotation

                    draw_vehicle_box(self.world, self.obstacle_manager.closest_front_vehicle, loc, rot, 1)
                # call obstacle manager for checking general obstacles obstacles                                         
                self.obstacle_manager.check_general_closest_obstacles()

                # call obstacle manager for checking side front and side rear obstacles 
                self.obstacle_manager.check_side_obstacles(self.waypoints, self.index, self.current_state)

                # call walker manager for checking pedestrians' locations
                self.walker_manager.check_pedestrians(self.waypoints, self.index)

                # check if rear obstacle is closely - tailgating 
                vehicle_in_back_closely = self.vehicle_in_back()

                #----------------------------#
                #|                          |#
                #|      check buttons       |#
                #|                          |# 
                #----------------------------#

                # read stop and start buttons in case of stopping the vehicle                  
                self.stop_signal = self.sub.get_stop()

                # check if button for stopping has been pressed  
                if self.stop_signal:
                    self.emergency_stop()
                    self.control_signal = self.custom_controller.run_step(self.velocity, self.waypoints[self.index])
                    self.vehicle_actor.apply_control(self.control_signal)

                    while not self.sub.get_start():
                        self.world.tick()
                                        
                # check if button for lane change has been pushed 
                self.turn = self.turn_sub.get_turn()
                
                # change goal - stop and change the final goal  
                change_goal = self.sub_change_goal.get_change_goal()
                if change_goal == True :
                    self.sub_change_goal.set_change_goal(False)
                    self.emergency_stop()
                    self.control_signal = self.custom_controller.run_step(self.velocity, self.waypoints[self.index])
                    self.vehicle_actor.apply_control(self.control_signal)
                    break

                # read velocity from the slider 
                #self.velocity = self.speed_sub.get_velocity()

                #-----------------------------------#
                #|                                 |#
                #|      check trigger criteria     |#
                #|                                 |# 
                #-----------------------------------#

                #check if ego vehicle has higher velocity than the front vehicle - 1st criterion 
                higher_velocity_than_front_obstacle = self.has_ego_higher_velocity_than_front_obstacle() 
                
                # check if front vehicle is closely - 2nd criterion 
                vehicle_in_front_closely = self.vehicle_in_front()
                
                # check if intersection exists - 3rd criterion 
                intersection_closely = self.exists_intersection(1.5 * self.lawful_score + self.convert_value_1(self.aggressive_score) * int(self.convert_offset_value(self.current_velocity)))

                # check if stop sign exists closely - 4th criterion
                stop_closely = self.exists_stop(self.lawful_score + self.convert_value_1(self.aggressive_score) * int(self.convert_offset_value(self.current_velocity)))

                # check if destination is close - 5th criterion 
                destination_close = self.is_end_location_closely()

                # check if front side is free for a while - 6th criterion 
                is_front_free = self.is_front_free_for_a_while()
               
                # check if light is red - 7th criterion
                red_light = self.red_traffic_light()

                # check if velocity is equal to zero - 8th criterion
                zero_velocity = self.check_zero_velocity()

                # check if right lane is safe - 9th criterion 
                safe_right_lane = self.is_right_lane_safe()                    
                
                # check if left lane is safe - 10th criterion 
                safe_left_lane = self.is_left_lane_safe()  
                
                # check if left lane has the same direction - 11th criterion 
                left_lane_different_direction = self.has_left_lane_different_direction()
                
                # check if right lane is denser than the left - 12th criterion
                right_lane_denser_than_left = self.is_right_lane_denser_than_left()
                
                # check if left lane is denser than the right - 13th criterion
                left_lane_denser_than_right = self.is_left_lane_denser_than_right()
                
                # check if right pedestrian is closely - 14th criterion
                right_pedestrian_closely = self.is_right_pedestrian_closely()

                # check if right pedestrian is closely - 15th criterion
                left_pedestrian_closely  = self.is_left_pedestrian_closely()

                # check if right pedestrian is closely - 16th criterion
                front_pedestrian_closely = self.is_front_pedestrian_closely()
                
                # check if right lane marking is legal - 17th criterion
                right_lane_marking_not_legal = self.is_right_lane_marking_not_legal()
         
                # check if left lane marking is legal - 18th criterion
                left_lane_marking_not_legal = self.is_left_lane_marking_not_legal()
                
                # check if vehicle is in bidirectional for long time - 19th criterion
                in_bidirectional_lane_for_long_time = self.is_in_bidirectional_lane_for_long_time()
                
                # check if left lane is bidirectional lane - 20th criterion
                left_lane_bidirectional = self.is_left_lane_bidirectional()
                
                # check if speed limit is above the acceptable - 21th criterion 
                speed_above_the_limit = self.speed_is_above_the_limit()

                # check if necessary right lane change may happen - 22th criterion 
                make_automatic_right_lane_change = self.automatic_right_lane_change()

                # check if necessary right lane change may happen - 23th criterion 
                make_automatic_left_lane_change = self.automatic_left_lane_change()

                # check if vehicle is still front - 24th criterion
                vehicle_still_front = self.is_vehicle_still_front()
                
                # check if junction in front of stop sign is free - 25th criterion 
                junction_free = self.is_junction_in_front_of_stop_free()
                               
                
                self.row_activation_filter    = np.array([[ 1 * higher_velocity_than_front_obstacle and 1 * vehicle_in_front_closely, 
                                                            1 * vehicle_in_front_closely, 
                                                            1 * intersection_closely, 
                                                            1 * stop_closely,
                                                            1 * destination_close,
                                                            1 * is_front_free,
                                                            1 * red_light,
                                                            1 * zero_velocity,
                                                            1 * safe_right_lane and (1 * vehicle_in_front_closely or 1 * make_automatic_right_lane_change),
                                                            1 * safe_left_lane and (1 * vehicle_in_front_closely or make_automatic_left_lane_change),
                                                            1 * left_lane_different_direction and 1 * vehicle_in_front_closely,
                                                            1 * right_lane_denser_than_left and 1 * vehicle_in_front_closely,
                                                            1 * left_lane_denser_than_right and 1 * vehicle_in_front_closely,
                                                            1 * right_pedestrian_closely,
                                                            1 * left_pedestrian_closely,
                                                            1 * front_pedestrian_closely,
                                                            1 * right_lane_marking_not_legal and 1 * vehicle_in_front_closely,
                                                            1 * left_lane_marking_not_legal and 1 * vehicle_in_front_closely,
                                                            1 * in_bidirectional_lane_for_long_time and 1 * safe_right_lane,
                                                            1 * left_lane_bidirectional and 1 * vehicle_in_front_closely,
                                                            1 * speed_above_the_limit,
                                                            1 * make_automatic_right_lane_change,
                                                            1 * make_automatic_left_lane_change,
                                                            1 * vehicle_still_front,
                                                            1 * (not safe_right_lane) and (1 * vehicle_in_front_closely or 1 * make_automatic_right_lane_change),
                                                            1 * (not safe_left_lane) and (1 * vehicle_in_front_closely or make_automatic_left_lane_change),
                                                            1 * junction_free]])

                self.row_filter_for_lawful_values = np.array([[     1 ,
                                                                    1 , 
                                                                    1 , 
                                                                    1 * self.convert_lawful_value(self.lawful_score),
                                                                    1 ,
                                                                    1 ,
                                                                    1 * self.convert_lawful_value(self.lawful_score),
                                                                    1 ,
                                                                    1 ,
                                                                    1 ,
                                                                    1 * self.convert_lawful_value(self.lawful_score),
                                                                    1 ,
                                                                    1 ,
                                                                    1 ,
                                                                    1 ,
                                                                    1 ,
                                                                    1 * self.convert_lawful_value(self.lawful_score),
                                                                    1 * self.convert_lawful_value(self.lawful_score),
                                                                    1 ,
                                                                    1 ,
                                                                    1 * self.convert_lawful_value(self.lawful_score),
                                                                    1 ,
                                                                    1 ,
                                                                    1 ,
                                                                    1 ,
                                                                    1 ,
                                                                    1 ]])
                
                
                self.column_filter_for_aggressive_values = np.array([[1,
                                                                      self.convert_aggressive_value_for_behaviors(self.aggressive_score), 
                                                                      max(0, self.convert_aggressive_value_for_behaviors(self.aggressive_score)), 
                                                                      max(0, self.convert_aggressive_value_for_behaviors(self.aggressive_score)), 
                                                                      1, 
                                                                      max(0, self.convert_aggressive_value_for_behaviors(self.aggressive_score)), 
                                                                      1, 
                                                                      1, 
                                                                      1]])

                self.column_filter = np.array([[1 * vehicle_in_front_closely * 0,
                                                1 * vehicle_in_front_closely * 0, 
                                                1, 
                                                1, 
                                                1, 
                                                1, 
                                                1, 
                                                1, 
                                                1]])

                self.column_filter = self.column_filter_for_aggressive_values * self.column_filter
                self.row_filter    = self.row_filter_for_lawful_values        * self.row_activation_filter

                # --------------------------------------------------------------------------- #
                # |                                                                         | #
                # |                        Vehicle's Action block                           | #
                # |                                                                         | #
                # --------------------------------------------------------------------------- #


                # behaviors:
                #               1 - CAR_FOLLOW 
                #               2 - OVERTAKE
                #
                #               3 - RIGHT_LANE_CHANGE 
                #               4 - LEFT_LANE_CHANGE 
                #               5 - KEEP_STRAIGHT              
                #
                #               6 - SPEED_UP 
                #               7 - SLOW_DOWN
                #               8 - KEEP_VELOCITY 
                #               9 - STOP
               
                
                self.check_new_triggers()

                self.sub_agg.set_change_aggressive(False)
                self.sub_law.set_change_lawful(False)


                self.updated_behaviors = pd.DataFrame(np.multiply(behaviors.values, self.row_filter.T), columns=columns, index=self.indexes)
                self.updated_behaviors = pd.DataFrame(np.multiply(self.updated_behaviors.values.T, self.column_filter.T).T, columns=columns, index=self.indexes)


                self.previous_filter = self.row_filter




                self.evaluate()

                #self.max_behavior = "KEEP_STRAIGHT"
                #self.max_speed_behavior = "SPEED_UP"

                if self.max_behavior == "OVERTAKE":
                    if not self.action_performed:                    
                        self.turn_obstacle = "LEFT"  
                        self.start_trigger_waypoint_number = self.index
                        self.action_performed = True
                    else:
                        self.turn_obstacle = None

                    self.overtake_started = self.overtake()

                    self.action_completed = False                
                    if not self.overtake_completed:
                        self.complete_overtake()

                    if self.overtake_completed:
                        self.action_completed = True
                        self.max_behavior = "KEEP_STRAIGHT"

                    if not self.overtake_started:
                        self.manual_lane_change()

                if self.max_behavior == "CAR_FOLLOW":
                    self.car_follow()


                if self.max_behavior == "RIGHT_LANE_CHANGE":

                    self.start_trigger_waypoint_number = self.index

                    if not self.action_performed:                    
                        self.turn_obstacle = "RIGHT"  
                        self.still_front = False
                        self.action_performed = True
                        self.wait(2)

                    self.regulate_speed()
                    self.overtake_started = self.overtake()
                    self.turn_obstacle = None

                    if not self.overtake_started:
                        self.manual_lane_change()   

                if self.max_behavior == "LEFT_LANE_CHANGE":
                    self.start_trigger_waypoint_number = self.index

                    if not self.action_performed:                    
                        self.turn_obstacle = "LEFT"  
                        self.still_front = False
                        self.action_performed = True
                        self.wait(2)
                    
                    self.regulate_speed()
                    self.overtake_started = self.overtake()
                    self.turn_obstacle = None

                    if not self.overtake_started:
                        self.manual_lane_change()  


                if self.max_behavior == "KEEP_STRAIGHT":
                    
                    self.regulate_speed()
                
                    self.manual_lane_change()
                
                self.off_road_event()
                self.check_stops()
                self.check_lights()
                self.check_speed_limit()

                
                # print the route's trace in the simulator
                self.world.debug.draw_string(self.waypoints[self.index].transform.location, "X", draw_shadow=False, color=carla.Color(r=0, g=0, b=255), life_time=2000, persistent_lines=True)
                
                self.control_signal = self.custom_controller.run_step(self.velocity, self.waypoints[self.index])
                                
                if isinstance(self.waypoints[self.index], carla.libcarla.Waypoint):
                    p1 = carla.Location(self.waypoints[self.index].transform.location.x, self.waypoints[self.index].transform.location.y,
                                        self.waypoints[self.index].transform.location.z)
                    
                elif isinstance(self.waypoints[self.index], carla.libcarla.Transform):
                    p1 = carla.Location(self.waypoints[self.index].location.x, self.waypoints[self.index].location.y,
                                        self.waypoints[self.index].location.z)

                p2 = carla.Location(vehicle_actor.get_location().x, vehicle_actor.get_location().y,
                                    vehicle_actor.get_location().z)
                dist = p1.distance(p2)
                #print('Distance from waypoint {}'.format(i), dist) 

                if dist < 2:
                    
                    self.index += 1
                    self.trajectory.change = False
                    self.get_current_velocity()
                                        
                    self.route_completion = self.index / (len(self.waypoints) - 1)
                    #print(self.route_completion)

                    self.overall_speed += self.current_velocity
                    loc = self.vehicle_actor.get_location()
                    vec = [loc.x, loc.y]
                    self.locations.append(vec)
                vehicle_actor.apply_control(self.control_signal)
                world.tick()
            except:
 
 
                self.average_speed = self.overall_speed / self.index 
                self.off_road_event_time = 1 - self.off_road_event_time / (time.time() - self.start_time)

                print("Route:"          ,   self.route_completion           )
                print("Time percentage:",   self.off_road_event_time        )
                print("Ped:"            ,   self.pedestrian_collision       )
                print("Veh:"            ,   self.vehicle_collision          )
                print("Stat:"           ,   self.static_obstacle_collision  )
                print("Red:"            ,   self.red_light_violations       )
                print("Stop:"           ,   self.stop_sign_violations       )
                print("Speed:"          ,   self.speed_limit_violations     )
                print("Right:"          ,   self.right_turns                )
                print("Left:"           ,   self.left_turns                 )
                print("Avg:"            ,   self.average_speed              )
 
                x_l = [cor[0] for cor in self.locations]
                y_l = [cor[1] for cor in self.locations]
                    
                self.waypoints_traj = [way.transform.location for way in self.waypoints]
                self.waypoints_traj = [[way.x, way.y] for way in self.waypoints_traj]
                    
                x_w = [cor[0] for cor in self.waypoints_traj]
                y_w = [cor[1] for cor in self.waypoints_traj]

                open('data_coordinates.txt', 'w').close()
                data_file = open('data_coordinates.txt', 'a')
                data_file.write(str(x_w))
                data_file.write(str(y_w))
                data_file.write(str(x_l))
                data_file.write(str(y_l))
                data_file.close()
                self.evaluate_node()

                print(111111)

    def off_road_event(self):
        w = self.map.get_waypoint(self.vehicle_actor.get_location(), lane_type=carla.LaneType.Any)
        if not str(w.lane_type) in ["Driving", "Bidirectional", "Shoulder", "Parking"]:
            self.off_road_event_time += time.time() - self.previous_off_road_event_time
        self.previous_off_road_event_time = time.time()
        
    def check_collissions(self):
        self.collision_detector.listen(lambda collision: self.collision_callback(collision))

    def collision_callback(self, collision):
        
        time = collision.timestamp
        if time - self.prev_time < 3:
            return
        if "pedestrian" in collision.other_actor.type_id:
            self.pedestrian_collision += 1
        elif "vehicle" in collision.other_actor.type_id:
            self.vehicle_collision += 1
        else:
            self.static_obstacle_collision += 1
        print(collision.other_actor.type_id)
        self.prev_time = time

    def check_stops(self):
        if not self.check_zero_velocity() and self.exists_stop(15):
                self.zero_vel += 1
                #print(self.zero_vel)

        else:
            self.zero_vel = 0

        if 40 < self.zero_vel and self.stop_sign_id != self.prev_stop_sign_id:
            self.stop_sign_violations += 1
            print("REEEEEEEEEEEEEEEEEEEEE STOPPPPPPPPPPPPPPPPPPPPPP")
            self.zero_vel = 0
            self.prev_stop_sign_id = self.stop_sign_id

    def check_lights(self):
        if not self.check_zero_velocity() and self.red_traffic_light():
            self.zero_vel_light += 1
            #print(self.zero_vel_light)

        else:
            self.zero_vel_light = 0
        
        if 25 < self.zero_vel_light and self.traffic_light_id != self.prev_traffic_light_id:
            self.red_light_violations += 1
            print("REEEEEEEEEEEEEEEEEEEEE FANARRRRRRRRRRRRRRRRRRRR")
            self.zero_vel_light = 0
            self.prev_traffic_light_id = self.traffic_light_id
    
    def check_speed_limit(self):
        if self.speed_is_above_the_limit() and self.current_velocity > 14:
            self.high_vel += 1
            #print(self.high_vel)
        else:
            self.high_vel = 0

        if 40 < self.high_vel and self.speed_limit_id != self.prev_speed_limit_id:
            self.speed_limit_violations += 1
            print("REEEEEEEEEEEEEEEEEEEEE TREXSSSSSSSSSSSSSSS")
            self.high_vel = 0
            self.prev_speed_limit_id = self.speed_limit_id

    def evaluate_node(self):
        
        data = {    'route_completion'         : self.route_completion, \
                    'off_road_event_time'      : self.off_road_event_time, \
                    'pedestrian_collision'     : self.pedestrian_collision,\
                    'vehicle_collision'        : self.vehicle_collision,\
                    'static_obstacle_collision': self.static_obstacle_collision,\
                    'red_light_violations'     : self.red_light_violations,\
                    'stop_sign_violations'     : self.stop_sign_violations,\
                    'speed_limit_violations'   : self.speed_limit_violations,\
                    'right_turns'              : self.right_turns,\
                    'left_turns'               : self.left_turns,\
                    'average_speed'            : self.average_speed}
        
        self.experiment_object.save_experiment_data(data)