import carla
import numpy as np
import pandas as pd 
from scipy.interpolate import interp1d
from agents.navigation.controller import VehiclePIDController
from traffic import Traffic
from vehicle_move import spawn
from utilities import change_coordinate_system, plot_axis
from vehicle import Vehicle
from client import Client
from obstacle_manager import ObstacleManager
from communicationMQTT import   VehiclePublisherMQTT, \
                                VehicleSubscriberStartStopMQTT, \
                                VehicleSubscriberVelocityMQTT, \
                                VehicleSubscriberLeftRightMQTT, \
                                VehicleSubscriberPositionMQTT, \
                                VehicleSubscriberChangeGoalMQTT, \
                                VehicleSubscriberAggressiveMQTT, \
                                VehicleSubscriberCautiousMQTT, \
                                VehicleSubscriberLawfulMQTT


class Behavior(object):
    def __init__(self, vehicle_actor, waypoints, trajectory, map, world, blueprint, vehicle_list):

        # basic variables initialization         
        self.vehicle_actor = vehicle_actor
        self.waypoints     = waypoints
        self.velocity      = 0
        self.trajectory    = trajectory
        self.map           = map
        self.world         = world
        self.blueprint     = blueprint
        self.vehicle_list  = vehicle_list

        # controller initialization
        self.custom_controller = VehiclePIDController(vehicle_actor, 
                                                      args_lateral      = {'K_P': 1, 'K_D': 0, 'K_I': 0},
                                                      args_longitudinal = {'K_P': 1, 'K_D': 0, 'K_I': 0},
                                                      max_throttle=0.75, 
                                                      max_brake=0.4,
                                                      max_steering=0.8)

        self.safe_distances = { 0 :  2.10, 
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
        self.obstacle_manager = ObstacleManager(map, vehicle_actor, vehicle_list, world)
        
        #
        # Communication attributes initialization
        # 
        # Publishers
        self.pub_vel         = VehiclePublisherMQTT           (topic='speed_configure'  )
        self.pub_notify      = VehiclePublisherMQTT           (topic='turn notification')
        self.pub             = VehiclePublisherMQTT           (topic='speed_topic'      )
        self.pub_agg         = VehiclePublisherMQTT           (topic='aggressive'       )
        self.pub_caut        = VehiclePublisherMQTT           (topic='cautious'         )
        self.pub_law         = VehiclePublisherMQTT           (topic='lawful'           )

        # Subscribers
        self.sub             = VehicleSubscriberStartStopMQTT (topic='start_stop_topic' )
        self.speed_sub       = VehicleSubscriberVelocityMQTT  (topic='speed_configure'  )
        self.turn_sub        = VehicleSubscriberLeftRightMQTT (topic='turn'             )
        self.sub_pos         = VehicleSubscriberPositionMQTT  (topic='position'         )
        self.sub_change_goal = VehicleSubscriberChangeGoalMQTT(topic='change_goal'      )
        self.sub_agg         = VehicleSubscriberAggressiveMQTT(topic='aggressive'       )
        self.sub_caut        = VehicleSubscriberCautiousMQTT  (topic='cautious'         )
        self.sub_law         = VehicleSubscriberLawfulMQTT    (topic='lawful'           )
                
        self.behavior_score       =  0
        self.cautious_score       =  0
        self.aggressive_score     =  0
        self.lawful_score         =  0

        self.aggressive_parameter =  1 
        self.cautious_parameter   = -1 
        self.lawful_parameter     =  0 
        
    def slow_down(self, desired_velocity):
        
        if self.velocity > desired_velocity:
            self.velocity = desired_velocity
            self.publish_velocity()

    def speed_up(self, desired_velocity):
        
        if self.velocity < desired_velocity:
            self.velocity = desired_velocity
            self.publish_velocity()

    def emergency_stop(self):
        
        self.velocity = 0
        self.publish_velocity()

    def publish_velocity(self):
        self.pub_vel.publish({'velocity': self.velocity})

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
                        if w == prev:
                            self.waypoints[self.index] = self.waypoints[self.index + 10]
                            self.index += self.lane_change_offset
                            self.current_state = "INIT"
                            self.pub_notify.publish({'value': self.current_state})
                            self.slow_down(desired_velocity=desired_vel)
                    else:
                        self.current_state = "INIT"
                        self.pub_notify.publish({'value': self.current_state})
                        self.slow_down(desired_velocity=desired_vel)
                    self.turn_sub.set_turn(None)

                elif turn != self.current_state:
                    self.current_state = "INIT"
                    self.pub_notify.publish({'value': self.current_state})
                    self.turn_sub.set_turn(None)
                    self.slow_down(desired_velocity=desired_vel)

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
        
        if self.overtake_direction == "RIGHT" and self.current_state != "INIT":

            if self.obstacle_manager.closest_rear_left_vehicle != None:               
            
                overtaked_obstacle_distance = self.obstacle_manager.closest_distance_from_rear_left_vehicle
                closest_front_side_obstacle_distance = self.obstacle_manager.closest_distance_from_front_left_vehicle

            else:
                return

        elif self.overtake_direction == "LEFT" and self.current_state != "INIT":
           
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
        delta_u = self.velocity - self.front_obstacle_velocity

        acc_cmd = kd * delta_d + ku * delta_u
        
        target_speed = self.velocity + acc_cmd * delay_t

        if target_speed != self.velocity:
            self.velocity = target_speed / 3.6 
            self.publish_velocity()

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
    
    def stop(self):
    
        # check for red lights and stop button 
        if self.traffic_light_state == "RED":

            # if RED light has activated when the vehicle was inside the junction then it is better to get out of the junction
            if not self.waypoints[self.index].is_junction:
                self.emergency_stop()
            else:
                self.control_signal = self.custom_controller.run_step(self.velocity, self.waypoints[self.index])  

        elif self.stop_signal:
            self.emergency_stop()

    def vehicle_in_front(self):
        if self.obstacle_manager.closest_distance_from_front_vehicle < self.front_obstacle_distance_threshold:

            self.obstacle_detected = self.obstacle_manager.closest_front_vehicle.id
            
            if self.previous_front_obstacle_detected != self.obstacle_detected and "vehicle" in self.obstacle_manager.closest_front_vehicle.type_id:

                print("New obstacle in front")

                self.previous_front_obstacle_detected = self.obstacle_detected
                
                return True

        return False
    
    def cautious(self):
        
        self.cautious_score = self.sub_caut.get_cautious()
        
        self.calculate_overall_score()

    def lawful(self):

        self.lawful_score = self.sub_law.get_lawful()

        self.calculate_overall_score()

    def aggressive(self):
        
        self.aggressive_score = self.sub_agg.get_aggressive()
        
        self.calculate_overall_score()
    
    def define_behavior_parameters(self, aggressive_parameter, cautious_parameter, lawful_parameter):
        
        self.aggressive_parameter = aggressive_parameter
        self.cautious_parameter   = cautious_parameter
        self.lawful_parameter     = lawful_parameter

    def calculate_overall_score(self):
        self.behavior_score = self.aggressive_parameter * self.aggressive_score + self.cautious_parameter * self.cautious_score + self.lawful_parameter * self.lawful_score
    
    def turn_decision(self):

        self.turn_obstacle = None 
        
        self.turn_law()
        
        self.is_right_lane_safe()
        self.is_left_lane_safe()
        
        if self.waypoints[self.index].get_left_lane() != None and self.condition_for_left_lane and self.rear_left_is_safe and self.front_left_is_safe:
            self.turn_obstacle = "LEFT"
                                                    
        elif self.waypoints[self.index].get_right_lane() != None and self.condition_for_right_lane and self.rear_right_is_safe and self.front_right_is_safe:
            self.turn_obstacle = "RIGHT"

    def turn_law(self):
        if self.lawful_score > 0:
            self.condition_for_right_lane = ("Solid" not in str(self.waypoints[self.index].right_lane_marking.type) and  ("Right" in str(self.waypoints[self.index].lane_change) or "Both" in str(self.waypoints[self.index].lane_change)))
            self.condition_for_left_lane  = ("Solid" not in str(self.waypoints[self.index].left_lane_marking.type)  and  ("Left"  in str(self.waypoints[self.index].lane_change) or "Both" in str(self.waypoints[self.index].lane_change)))

        else: 
            self.condition_for_right_lane = ("Right" in str(self.waypoints[self.index].lane_change) or "Both" in str(self.waypoints[self.index].lane_change))
            self.condition_for_left_lane  = ("Left"  in str(self.waypoints[self.index].lane_change) or "Both" in str(self.waypoints[self.index].lane_change))
    
    def is_right_lane_safe(self):
        
        self.rear_right_is_safe  = True
        self.front_right_is_safe = True

        if self.obstacle_manager.closest_distance_from_rear_right_vehicle  < self.rear_right_vehicle_threshold:
            self.rear_right_is_safe  = False

        if self.obstacle_manager.closest_distance_from_front_right_vehicle < self.front_right_vehicle_threshold:
            self.front_right_is_safe = False

    def is_left_lane_safe(self):
        
        self.rear_left_is_safe  = True
        self.front_left_is_safe = True

        if self.obstacle_manager.closest_distance_from_rear_left_vehicle  < self.rear_left_vehicle_threshold:
            self.rear_left_is_safe  = False

        if self.obstacle_manager.closest_distance_from_front_left_vehicle < self.front_left_vehicle_threshold:
            self.front_left_is_safe = False

    def calculate_safe_distance(self):
        
        self.safe_distance = -float("inf")
        
        if self.obstacle_manager.closest_front_vehicle != None:
            velocity_vector = self.obstacle_manager.closest_front_vehicle.get_velocity()
            velocity_array = [velocity_vector.x, velocity_vector.y, velocity_vector.z]
            other_velocity = np.linalg.norm(velocity_array)
            other_velocity = round(3.6 * other_velocity, 1)

            diff = self.velocity - other_velocity
            if 0 <= diff <= 100: 
                keys = list(self.safe_distances.keys())
                values = list(self.safe_distances.values())

                inter_function = interp1d(keys, values)
                self.safe_distance = max(10, float(inter_function(diff)))

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
        
        convert_aggressive_value = interp1d([0, 10 ], [30, 15])
        convert_cautious_value   = interp1d([0, 10 ], [0 , 10])
        convert_offset_value     = interp1d([0, 100], [5 , 20])
        convert_aggressive_value_for_behaviors = interp1d([0, 10 ], [1, 2])
     
        #spawn(self.vehicle_list)

        '''
        walker_list = []
        walker_loc = carla.Location(x=80.551256, y=-195, z=1)
        walker_rot = carla.Rotation(0, 0, 0)
        walker_trans = carla.Transform(walker_loc, walker_rot)
        walker = self.blueprint.filter("walker.pedestrian.0001")[0]      
        
        walker_actor = self.world.spawn_actor(walker, walker_trans)
        walker_list.append(walker_actor)

        pedestrian_location = walker_list[0].get_location()
        '''
        '''
        start_point = carla.Transform(carla.Location(x=25.551256, y=-193.809540, z=1), carla.Rotation(pitch=360.000, yaw=1.439560, roll=0.0))

        thr = 0.25
        vehicle = Vehicle()                                  
        vehicle.choose_spawn_point(start_point)                 # spawn the vehicle 
        vehicle.choose_model('model3', self.blueprint, world)
        vehicle_actor1 = vehicle.get_vehicle_actor()
        control_signal1 = carla.VehicleControl(throttle=thr)
        vehicle_actor1.apply_control(control_signal1)
        self.vehicle_list.append(vehicle_actor1)
        '''
        start_point = carla.Transform(carla.Location(x=73.551256, y=-196, z=1), carla.Rotation(pitch=360.000, yaw=1.439560, roll=0.0))

        thr = 0.2
        vehicle = Vehicle()                                  
        vehicle.choose_spawn_point(start_point)                 # spawn the vehicle 
        vehicle.choose_model('model3', self.blueprint, world)
        vehicle_actor1 = vehicle.get_vehicle_actor()
        control_signal1 = carla.VehicleControl(throttle=thr)
        vehicle_actor1.apply_control(control_signal1)
        self.vehicle_list.append(vehicle_actor1)
       
        '''
        start_point = carla.Transform(carla.Location(x=65.551256, y=-195.809540, z=1), carla.Rotation(pitch=360.000, yaw=1.439560, roll=0.0))
        thr = 0.2
        vehicle = Vehicle()                                  
        vehicle.choose_spawn_point(start_point)                 # spawn the vehicle 
        vehicle.choose_model('model3', self.blueprint, world)
        vehicle_actor1 = vehicle.get_vehicle_actor()
        control_signal1 = carla.VehicleControl(throttle=thr)
        vehicle_actor1.apply_control(control_signal1)
        self.vehicle_list.append(vehicle_actor1)
        '''
        
        velocity = 15
        self.pub_vel. publish ({'velocity'  : velocity})
        self.pub_agg. publish ({'aggressive':  0      })
        self.pub_caut.publish ({'cautious'  : 10      })
        self.pub_law. publish ({'lawful'    : 10      })
        self.wait(10)
        
        self.action = False
        

        behaviors = {
                        "OVERTAKE"          : [ 0.4,  0.7],
                        "CAR_FOLLOW"        : [-0.3 * (-100), -0.3], 

                        "LEFT_LANE_CHANGE"  : [ 0.2,  0.5],
                        "RIGHT_LANE_CHANGE" : [ 0.2,  0.5], 
                        "KEEP_STRAIGHT"     : [-0.2, -0.2],
                        
                        "SPEED_UP"          : [-0.1, -0.1], 
                        "SLOW_DOWN"         : [ 0.8,  0.8],
                        "KEEP_VELOCITY"     : [-0.1, -0.8],
                        "STOP"              : [ 0.5,  0.6]
                        
                        }

        behaviors = pd.DataFrame(behaviors, index=[
                                                   "higher velocity than obstacle's velocity",
                                                   "close front vehicle"
                                                   ])
        max_behavior = "KEEP_STRAIGHT"

        while True:
            """
            thr += 0.001
            control_signal1 = carla.VehicleControl(throttle=thr)
            vehicle_actor1.apply_control(control_signal1)
            """
            try:
                    
                # spectator method is called in order to place the view of the simulator exactly above the vehicle 
                # spectator()

                # corner case that is being executed, when the vehicle reaches its destination
                if self.index == len(self.waypoints) - 1:
                    self.control_signal = self.custom_controller.run_step(0, self.waypoints[self.index - 1])
                    vehicle_actor.apply_control(self.control_signal)
                    break

                # change goal - stop and change the final goal  
                change_goal = self.sub_change_goal.get_change_goal()
                if change_goal == True:
                    self.sub_change_goal.set_change_goal(False)
                    self.emergency_stop()

                    self.control_signal = self.custom_controller.run_step(self.velocity, self.waypoints[self.index])
                    vehicle_actor.apply_control(self.control_signal)

                    break                
                
                # velocity's norm in km/h - publish the current velocity in the speedometer 
                velocity_vector = vehicle_actor.get_velocity()
                velocity_array = [velocity_vector.x, velocity_vector.y, velocity_vector.z]
                velocity_norm = np.linalg.norm(velocity_array)
                self.pub.publish({'velocity': round(3.6 * velocity_norm, 1)})

                # --------------------------------------------------------------------------- #
                # |                                                                         | #
                # |                        Vehicle's Behavior Definition                    | #
                # |                                                                         | #
                # --------------------------------------------------------------------------- #

                if self.sub_agg.get_change_aggressive():
                    self.aggressive()
                    self.sub_agg.set_change_aggressive(False)

                if self.sub_caut.get_change_cautious():
                    self.cautious()
                    self.sub_caut.set_change_cautious(False)

                if self.sub_law.get_change_lawful():
                    self.lawful()
                    self.sub_law.set_change_lawful(False)  


                self.calculate_safe_distance()

                #self.front_obstacle_distance_threshold = convert_aggressive_value(self.aggressive_score) + convert_cautious_value(self.cautious_score)
                self.front_obstacle_distance_threshold = self.safe_distance
                
                self.overtaked_obstacle_distance_threshold = convert_aggressive_value(self.aggressive_score) / 2
                self.closest_front_side_obstacle_distance_threshold = 20
                self.lane_change_offset = int(convert_offset_value(self.velocity))
                self.rear_obstacle_distance_threshold = 5 #convert_aggressive_value(self.aggressive_score)
                
                self.rear_right_vehicle_threshold  = 10 #convert_aggressive_value(self.aggressive_score)
                self.front_right_vehicle_threshold = 15 #convert_aggressive_value(self.aggressive_score)
                self.rear_left_vehicle_threshold   = convert_aggressive_value(self.aggressive_score)
                self.front_left_vehicle_threshold  = convert_aggressive_value(self.aggressive_score)

                self.is_right_lane_safe()                    
                
                #print("----")
                #print(self.rear_right_is_safe , self.obstacle_manager.closest_rear_right_vehicle )
                #print(self.front_right_is_safe, self.obstacle_manager.closest_front_right_vehicle)
                #print(self.obstacle_manager.closest_distance_from_front_right_vehicle, self.front_right_vehicle_threshold)
                #print("----")
                # --------------------------------------------------------------------------- #
                # |                                                                         | #
                # |                           Vehicle's Perception                          | #
                # |                                                                         | #
                # --------------------------------------------------------------------------- #

                # initialize traffic manager in order to handle traffic lights and traffic signs 
                traffic = Traffic(world, self.map)
                #traffic_sign = traffic.check_signs(self.waypoints[self.index])
                self.traffic_light_state = traffic.check_traffic_lights(vehicle_actor)
                #traffic.get_lane_info(self.waypoints[self.index])

                # read stop and start buttons in case of stopping the vehicle                  
                self.stop_signal = self.sub.get_stop()
                
                # check if button for lane change has been pushed 
                self.turn = self.turn_sub.get_turn()

                # read velocity from the slider 
                self.velocity = self.speed_sub.get_velocity()
                
                # call obstacle manager for checking front and rear obstacles                                         
                self.obstacle_manager.check_obstacles()

                # call obstacle manager for checking side front and side rear obstacles 
                self.obstacle_manager.check_side_obstacles(self.waypoints, self.index)

                # check if rear obstacle is closely - tailgating 
                vehicle_in_back_closely = self.vehicle_in_back()
                
                # check if front vehicle is closely 
                vehicle_in_front_closely = self.vehicle_in_front()
                
                # --------------------------------------------------------------------------- #
                # |                                                                         | #
                # |                        Vehicle's Action block                           | #
                # |                                                                         | #
                # --------------------------------------------------------------------------- #

                #if self.behavior_score > 0:
                #    self.speed_up(self.aggressive_score)
                #    self.overtake_started = self.overtake()
                #    if self.overtake_started:
                #        print(1)
                    #if vehicle_in_back_closely:
                    #    self.speed_up(self.aggressive_score)
                    #    self.action_performed = True
                    
                #else:
                    #if vehicle_in_back_closely:
                    #    self.speed_up(self.cautious_score)
                    #    self.turn_decision()
                    #    self.overtake_started = self.overtake()
                    #    self.action_performed = True
                    #else:
                    #    self.overtake_started = False
                #    if self.obstacle_manager.closest_distance_from_front_vehicle < 15:
                #        self.car_follow()
                #        print(2)
                
                #if vehicle_in_front_closely and not car_follow_behavior:
                #    car_follow_behavior = True
                    
                #if car_follow_behavior:
                #    self.car_follow()
                
                '''
                if vehicle_in_back_closely:
                
                    self.action_performed = True
                
                    self.get_rear_obstacle_velocity()
                    
                    if self.rear_obstacle_velocity != self.velocity:
                        desired_velocity = int(self.rear_obstacle_velocity + 3 * self.rear_obstacle_velocity / 4)
                        self.speed_up(desired_velocity)
                '''

                '''
                if vehicle_in_front_closely:
                    
                    self.turn_decision()
                    
                    if self.turn_obstacle != None:    
                        self.overtake_started = self.overtake()
                    else:
                        car_follow_behavior = True
                        self.car_follow()
                else:
                    self.overtake_started = False
                
                if not self.overtake_completed:
                    self.complete_overtake()

                if not self.overtake_started:
                   self.manual_lane_change()         
                '''
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
                        
                if vehicle_in_front_closely:
                    
                    aggressive_param = convert_aggressive_value_for_behaviors(self.aggressive_score)
                    
                    behaviors["OVERTAKE"         ][:] = behaviors["OVERTAKE"         ][:] * aggressive_param
                    behaviors["LEFT_LANE_CHANGE" ][:] = behaviors["LEFT_LANE_CHANGE" ][:] * aggressive_param
                    behaviors["RIGHT_LANE_CHANGE"][:] = behaviors["RIGHT_LANE_CHANGE"][:] * aggressive_param
                    behaviors["SPEED_UP"         ][:] = behaviors["SPEED_UP"         ][:] * aggressive_param
                    
                    values = behaviors.loc[:].values
                    sum_values = values.sum(0)
                    
                    max_value = max(sum_values)
                    
                    max_key = np.argmax(sum_values, 0)
                    max_behavior = behaviors.columns[max_key]

                    #print(self.safe_distance, self.obstacle_manager.closest_distance_from_front_vehicle)
                   
                if max_behavior == "OVERTAKE":
                    
                    if not self.action_performed:                    
                        self.turn_obstacle = "LEFT"  
                        self.action_performed = True

                    else:
                        self.turn_obstacle = None

                    self.overtake_started = self.overtake()
                
                    if not self.overtake_completed:
                        self.complete_overtake()
                    
                    if self.overtake_completed:
                        max_behavior = "KEEP_STRAIGHT"

                    if not self.overtake_started:
                        self.manual_lane_change()

                if max_behavior == "CAR_FOLLOW":
                    self.car_follow()

                if max_behavior == "RIGHT_LANE_CHANGE":
                    
                    if not self.action_performed:                    
                        self.turn_obstacle = "RIGHT"  
                        self.action_performed = True

                    else:
                        self.turn_obstacle = None

                    self.overtake_started = self.overtake()
                    
                    if not self.overtake_started:
                        self.manual_lane_change()   

                if max_behavior == "LEFT_LANE_CHANGE":
                    
                    if not self.action_performed:                    
                        self.turn_obstacle = "LEFT"  
                        self.action_performed = True

                    else:
                        self.turn_obstacle = None

                    self.overtake_started = self.overtake()
                    
                    if not self.overtake_started:
                        self.manual_lane_change()  

                if max_behavior == "SLOW_DOWN":
                    if not self.action_performed:
                        self.get_front_obstacle_velocity()
                        self.slow_down(self.front_obstacle_velocity)
                        self.action_performed = True
                
                if max_behavior == "SPEED_UP":
                    if not self.action_performed:
                        self.speed_up(2 * self.velocity)
                        self.action_performed = True

                if max_behavior == "STOP":
                    if not self.action_performed:    
                        self.emergency_stop()
                        self.action_performed = True

                if max_behavior == "KEEP_VELOCITY" or max_behavior == "KEEP_STRAIGHT":
                    pass
                
                
                '''
                self.car_follow_behavior  = 0
                self.lane_change_behavior = 0
                self.speed_up_behavior    = 0
                self.slow_down_behavior   = 0
                
                if vehicle_in_back_closely and self.action == False:
                    
                    self.car_follow_behavior += -1
                    self.slow_down_behavior  += -1
                
                    if self.aggressive_score > 5:
                        self.speed_up_behavior    +=  convert_aggressive_value_for_behaviors(self.aggressive_score) 
                        self.slow_down_behavior   += -1
                        self.lane_change_behavior += -convert_aggressive_value_for_behaviors(self.aggressive_score)
                    
                    else:
                        self.speed_up_behavior    += -convert_aggressive_value_for_behaviors(self.aggressive_score) 
                        self.slow_down_behavior   += -1
                        self.lane_change_behavior +=  convert_aggressive_value_for_behaviors(self.aggressive_score)

                    behaviors["CAR_FOLLOW_BEHAVIOR" ] = self.car_follow_behavior
                    behaviors["LANE_CHANGE_BEHAVIOR"] = self.lane_change_behavior
                    behaviors["SPEED_UP_BEHAVIOR"   ] = self.speed_up_behavior
                    behaviors["SLOW_DOWN_BEHAVIOR"  ] = self.slow_down_behavior
                    
                    self.action == True
                    
                    max_key = max(behaviors, key=behaviors.get)
                    print(max_key, behaviors[max_key])
                '''
                # print the route's trace in the simulator
                self.world.debug.draw_string(self.waypoints[self.index].transform.location, "X", draw_shadow=False, color=carla.Color(r=0, g=0, b=255), life_time=1000, persistent_lines=True)
                
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

                vehicle_actor.apply_control(self.control_signal)
                world.tick()

            except KeyboardInterrupt:
                break
