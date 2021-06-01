import carla
import numpy as np
from agents.navigation.controller import VehiclePIDController
from traffic import Traffic
from vehicle_move import spawn
from utilities import change_coordinate_system
from vehicle import Vehicle
from client import Client
from obstacle_manager import ObstacleManager
from communicationMQTT import VehiclePublisherMQTT, \
    VehicleSubscriberStartStopMQTT, \
    VehicleSubscriberVelocityMQTT, \
    VehicleSubscriberLeftRightMQTT, \
    VehicleSubscriberPositionMQTT, \
    VehicleSubscriberBehaviorMQTT, \
    VehicleSubscriberAggressiveMQTT, \
    VehicleSubscriberCautiousMQTT


class Behavior(object):
    def __init__(self, vehicle_actor, waypoints, trajectory, map, world, vehicle_list):
        
        # controller 
        self.custom_controller = VehiclePIDController(vehicle_actor, args_lateral={'K_P': 1, 'K_D': 0, 'K_I': 0},
                                                      args_longitudinal={'K_P': 1, 'K_D': 0, 'K_I': 0})

        # Communication attributes
        # 
        # Publishers
        self.pub_vel = VehiclePublisherMQTT(topic='speed_configure')
        self.pub_notify = VehiclePublisherMQTT(topic='turn notification')
        self.pub = VehiclePublisherMQTT(topic='speed_topic')
        self.pub_agg = VehiclePublisherMQTT(topic='aggressive')
        self.pub_caut = VehiclePublisherMQTT(topic='cautious')

        # Subscribers
        self.sub = VehicleSubscriberStartStopMQTT(topic='start_stop_topic')
        self.speed_sub = VehicleSubscriberVelocityMQTT(topic='speed_configure')
        self.turn_sub = VehicleSubscriberLeftRightMQTT(topic='turn')
        self.sub_pos = VehicleSubscriberPositionMQTT(topic='position')
        self.sub_behavior = VehicleSubscriberBehaviorMQTT(topic='behavior')
        self.sub_agg = VehicleSubscriberAggressiveMQTT(topic="aggressive")
        self.sub_caut = VehicleSubscriberCautiousMQTT(topic="cautious")

        self.vehicle_actor = vehicle_actor
        self.waypoints = waypoints
        self.velocity = 0
        self.trajectory = trajectory
        self.map = map
        self.world = world
        self.vehicle_list = vehicle_list

        self.obstacle_manager = ObstacleManager(map, vehicle_actor, vehicle_list)
        
        self.behavior_score = 0
        self.cautious_score = 0
        self.aggressive_score = 0

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
                            self.waypoints[self.index] = self.waypoints[self.index + 5]
                            self.index += 5
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
            self.index += 5
            self.change_lane(self.turn_obstacle, self.index, self.velocity)
            self.overtake_direction = self.turn_obstacle
            self.turn_obstacle = None
            self.return_to_the_initial_lane = True
            return True
        
        return False
    
    def complete_overtake(self):
        if self.overtake_direction == "RIGHT":
            overtaked_obstacle_distance = self.obstacle_manager.closest_distance_from_rear_left_vehicle
            self.turn_obstacle = "LEFT"
        
        elif self.overtake_direction == "LEFT":
            overtaked_obstacle_distance = self.obstacle_manager.closest_distance_from_rear_right_vehicle
            self.turn_obstacle = "RIGHT"
        
        else:
            return
        
        if 20 > overtaked_obstacle_distance > 10 and self.return_to_the_initial_lane == True:
            self.overtake()
            self.return_to_the_initial_lane = False

    def manual_lane_change(self):
        
        # first check is for possible push button but not in the same direction as the current state is (self.turn != self.current_state),
        # second check is for possible lane change, due to the path that has been created by the A*.
          
        if (self.turn != None and self.turn != self.current_state) or (self.turn == None and self.current_state == "INIT" and self.waypoints[self.index].lane_id != self.waypoints[self.index + 1].lane_id):
           
            self.trajectory.change = False
            if self.index + 5 < len(self.waypoints):
                self.index += 5
            else:
                self.turn = None 
                
        self.change_lane(self.turn, self.index, self.velocity)
    
    def check_tailgating(self):
        if self.obstacle_manager.closest_distance_from_rear_vehicle < 10:
            
            obstacle_detected = self.obstacle_manager.closest_rear_vehicle.id

            if self.previous_back_obstacle_detected != obstacle_detected and "vehicle" in self.obstacle_manager.closest_rear_vehicle.type_id:
                
                print("New back obstacle")
            
                self.previous_back_obstacle_detected = obstacle_detected

                return True
        return False

    def car_follow(self):
        if self.obstacle_manager.closest_front_vehicle is None:
            return
        velocity_vec = self.obstacle_manager.closest_front_vehicle.get_velocity()
        velocity_obs_array = [velocity_vec.x, velocity_vec.y, velocity_vec.z]
        velocity_obstacle = np.linalg.norm(velocity_obs_array)
        velocity_obstacle = round(3.6 * velocity_obstacle, 1)
        if velocity_obstacle != self.velocity:
            self.velocity = velocity_obstacle
            self.publish_velocity()
    
    def cautious(self):
        
        self.cautious_score = self.sub_caut.get_cautious()
        
        self.calculate_overall_score()

    def neutral(self):
        pass

    def aggressive(self):
        
        self.aggressive_score = self.sub_agg.get_aggressive()
        
        self.calculate_overall_score()
        
    def calculate_overall_score(self):
        self.behavior_score = self.aggressive_score - self.cautious_score

    def follow_trajectory(self, world, vehicle_actor, spectator, get_front_obstacle, set_front_obstacle, get_other_actor, velocity):
        self.index = 0
        self.current_state = "INIT"
        success = False
        self.return_to_the_initial_lane = False 
        self.overtake_direction = None

        self.turn = None
        self.turn_obstacle = None
        
        vel = {'velocity': velocity}
        self.pub_vel.publish(vel)
        
        aggressive = {'aggressive': 0}
        self.pub_agg.publish(aggressive)
        
        cautious = {'cautious': 0}
        self.pub_caut.publish(cautious)
        
        self.previous_front_obstacle_detected = None
        self.previous_back_obstacle_detected = None 
        client = Client()                                       
        client.connect()                                        # connect the client 
        [blueprint, world, _]= client.get_simulation()
        
        spawn(self.vehicle_list)
        '''
        start_point = carla.Transform(carla.Location(x=5.551256, y=-197.809540, z=1), carla.Rotation(pitch=360.000, yaw=1.439560, roll=0.0))
        thr = 0.2
        vehicle = Vehicle()                                  
        vehicle.choose_spawn_point(start_point)                 # spawn the vehicle 
        vehicle.choose_model('model3', blueprint, world)
        vehicle_actor1 = vehicle.get_vehicle_actor()
        control_signal1 = carla.VehicleControl(throttle=thr)
        vehicle_actor1.apply_control(control_signal1)
        self.vehicle_list.append(vehicle_actor1)
        '''
        while True:
            ''' 
            thr += 0.0001
            control_signal1 = carla.VehicleControl(throttle=thr)
            vehicle_actor1.apply_control(control_signal1)
            '''
            try:
                
                # corner case that is executed when the vehicle reaches the destination
                if self.index == len(self.waypoints) - 1:
                    control_signal = self.custom_controller.run_step(0, self.waypoints[self.index - 1])
                    vehicle_actor.apply_control(control_signal)
                    break
                
                # spectator method is called in roder to place the view of the simulator exactly above the vehicle 
                # spectator()
               

                # velocity's norm in km/h
                # publish the current velocity in the speedometer 
                velocity_vector = vehicle_actor.get_velocity()
                velocity_array = [velocity_vector.x, velocity_vector.y, velocity_vector.z]
                velocity_norm = np.linalg.norm(velocity_array)
                vel = {'velocity': round(3.6 * velocity_norm, 1)}
                self.pub.publish(vel)

                # initialize traffic manager in order to handle traffic lights and traffic signs 
                traffic = Traffic(world, self.map)
                traffic_sign = traffic.check_signs(self.waypoints[self.index])
                traffic_light_state = traffic.check_traffic_lights(vehicle_actor)
                stop = self.sub.get_stop()
                self.velocity = self.speed_sub.get_velocity()

                # check for lane change
                self.turn = self.turn_sub.get_turn()
                '''
                # check if rear obstacle is closely
                tailgating = self.check_tailgating()
                if self.behavior_score > 0:
                    #success = self.overtake()
                    if tailgating:
                        self.speed_up(self.aggressive_score)

                else:
                    if tailgating:
                        self.turn_obstacle = "RIGHT"
                        success = self.overtake()
                    else:
                        success = False
                    #if self.obstacle_manager.closest_distance_from_front_vehicle < 15:
                    #    self.car_follow()
                '''
                
                #ATTENTION 
                # Na ilopoiso tin epistrofi stin proigoumeni lorida 
                                
                success = self.overtake()
                
                if not success:
                    self.manual_lane_change()         
                
                self.complete_overtake()

                self.obstacle_manager.check_obstacles()

                self.obstacle_manager.check_side_obstacles()
                
                # print the route's trace in the simulator
                self.world.debug.draw_string(self.waypoints[self.index].transform.location, "X", draw_shadow=False, color=carla.Color(r=0, g=0, b=255), life_time=1000, persistent_lines=True)

                # change goal --need-change-topic 
                behavior = self.sub_behavior.get_behavior()
                if behavior == True:
                    self.sub_behavior.set_behavior(False)
                    self.emergency_stop()

                    control_signal = self.custom_controller.run_step(self.velocity, self.waypoints[self.index])
                    vehicle_actor.apply_control(control_signal)

                    self.publish_velocity()
                    break
            
                if self.sub_agg.get_change_aggressive():
                    self.aggressive()
                    self.sub_agg.set_change_aggressive(False)

                if self.sub_caut.get_change_cautious():
                    self.cautious()
                    self.sub_caut.set_change_cautious(False)

                # check for red lights, front obstacles and stop button 
                if traffic_light_state == "RED" and False:

                    # if RED light has activated when vehicle was inside the junction then it is better to get out of
                    # the junction
                    if not self.waypoints[self.index].is_junction:
                        self.emergency_stop()
                    else:
                        control_signal = self.custom_controller.run_step(self.velocity, self.waypoints[self.index])

                elif self.obstacle_manager.closest_distance_from_front_vehicle < 15:

                    # set False in order to check if obstacle detector has triggered again
                    set_front_obstacle(False)

                    obstacle_detected = self.obstacle_manager.closest_front_vehicle.id
                  

                    if self.previous_front_obstacle_detected != obstacle_detected and "vehicle" in self.obstacle_manager.closest_front_vehicle.type_id:
                        
                        print("New obstacle")
                          
                        velocity_vec = self.obstacle_manager.closest_front_vehicle.get_velocity()
                        velocity_obs_array = [velocity_vec.x, velocity_vec.y, velocity_vec.z]
                        velocity_obstacle = np.linalg.norm(velocity_obs_array)
                        velocity_obstacle = round(3.6 * velocity_obstacle, 1)
                        
                        if self.waypoints[self.index].get_left_lane() != None and "Solid" not in str(self.waypoints[self.index].left_lane_marking.type):
                            self.turn_obstacle = "LEFT"
                                                    
                        elif self.waypoints[self.index].get_right_lane() != None and "Solid" not in str(self.waypoints[self.index].right_lane_marking.type):
                            self.turn_obstacle = "RIGHT"

                        self.previous_front_obstacle_detected = obstacle_detected

                    else:
                        pass

                elif stop:
                    self.emergency_stop()
                
                
                control_signal = self.custom_controller.run_step(self.velocity, self.waypoints[self.index])
                #if abs(control_signal.steer) > 0.6:
                #    self.slow_down(desired_velocity=10)
                #    control_signal = self.custom_controller.run_step(self.velocity, self.waypoints[self.index])

                
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

                vehicle_actor.apply_control(control_signal)
                world.tick()

            except KeyboardInterrupt:
                break
