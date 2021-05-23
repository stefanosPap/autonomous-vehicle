import carla
import numpy as np
from agents.navigation.controller import VehiclePIDController
from traffic import Traffic
from vehicle_move import spawn
from utilities import change_coordinate_system
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
                    turn = self.turn_sub.set_turn(None)

                else:
                    self.current_state = "INIT"
                    turn = self.turn_sub.set_turn(None)

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
                    turn = self.turn_sub.set_turn(None)

                elif turn != self.current_state:
                    self.current_state = "INIT"
                    self.pub_notify.publish({'value': self.current_state})
                    turn = self.turn_sub.set_turn(None)
                    self.slow_down(desired_velocity=desired_vel)

    def overtake(self):
        if self.turn_obstacle != None:
            self.trajectory.change = False
            self.index += 5
            self.change_lane(self.turn_obstacle, self.index, self.velocity)
            self.turn_obstacle = None
            return True
        return False
    
    def manual_lane_change(self):
        if self.turn != None and self.turn != self.current_state:
            self.trajectory.change = False
            self.index += 5 
        self.change_lane(self.turn, self.index, self.velocity)
    
    def cautious(self):
        self.velocity -= self.sub_caut.get_cautious()

    def neutral(self):
        pass

    def aggressive(self):
        self.velocity += self.sub_agg.get_aggressive()
        
    def follow_trajectory(self, world, vehicle_actor, spectator, get_front_obstacle, set_front_obstacle, get_other_actor, velocity):
        self.index = 0
        self.current_state = "INIT"
        
        self.turn = None
        self.turn_obstacle = None
        
        vel = {'velocity': velocity}
        self.pub_vel.publish(vel)

        previous_velocity = 0
        previous_obstacle_detected = None
        
        spawn(self.vehicle_list)
        
        while True:
            try:

                if self.index == len(self.waypoints):
                    control_signal = self.custom_controller.run_step(0, self.waypoints[self.index - 1])
                    vehicle_actor.apply_control(control_signal)
                    break

                # spectator()
                '''
                p1 = [self.waypoints[self.index].transform.location.x, self.waypoints[self.index].transform.location.y, self.waypoints[self.index].transform.location.z]
                p2 = [vehicle_actor.get_location().x, vehicle_actor.get_location().y, vehicle_actor.get_location().z]
                dist = np.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2 + (p2[2] - p1[2])**2)
                '''

                # velocity's norm in km/h
                velocity_vector = vehicle_actor.get_velocity()
                velocity_array = [velocity_vector.x, velocity_vector.y, velocity_vector.z]
                velocity_norm = np.linalg.norm(velocity_array)
                vel = {'velocity': round(3.6 * velocity_norm, 1)}
                self.pub.publish(vel)

                #draw_vehicle_box(world, vehicle_actor,
                # vehicle_actor.get_transform().location, vehicle_actor.get_transform().rotation, 0.05)

                traffic = Traffic(world, self.map)
                traffic_sign = traffic.check_signs(self.waypoints[self.index])
                traffic_light_state = traffic.check_traffic_lights(vehicle_actor)
                stop = self.sub.get_stop()
                self.velocity = self.speed_sub.get_velocity()
                '''
                left = self.waypoints[self.index].get_left_lane()
                right = self.waypoints[self.index].get_right_lane()
                if left != None:
                    world.debug.draw_string(left.transform.location, '{}'.format(0), draw_shadow=False, color=carla.Color(r=0, g=0, b=255), life_time=1000, persistent_lines=True)
                if right != None:
                    world.debug.draw_string(right.transform.location, '{}'.format(1), draw_shadow=False, color=carla.Color(r=255, g=0, b=0), life_time=1000, persistent_lines=True)
                '''
 
                front_location = vehicle_actor.get_location() + carla.Location(vehicle_actor.bounding_box.extent.x, 0, 0)
                rear_location = vehicle_actor.get_location() - carla.Location(vehicle_actor.bounding_box.extent.x, 0, 0)

                ego_vehicle = self.vehicle_list[0]
                
                vehicles_in_lane = []

                for vehicle in self.vehicle_list[1:]:

                    ego_waypoint = self.map.get_waypoint(ego_vehicle.get_location())
                    other_waypoint = self.map.get_waypoint(vehicle.get_location())
                    
                    if ego_waypoint.lane_id == other_waypoint.lane_id:
                        vehicles_in_lane.append(vehicle)
                
                front_vehicles = []
                front_distances = []

                rear_vehicles = []
                rear_distances = []
                
                closest_distance_from_front_vehicle = float('inf')
                closest_distance_from_rear_vehicle = float('inf')
                
                for vehicle in vehicles_in_lane:
                    vehicle_location = vehicle.get_location()

                    ego_distance_front = vehicle_location.distance(front_location)
                    ego_distance_rear = vehicle_location.distance(rear_location)
                    ego_distance = vehicle_location.distance(ego_vehicle.get_location())

                    if ego_distance_front < ego_distance_rear:
                        front_vehicles.append(vehicle)
                        front_distances.append(ego_distance)
                    else:
                        rear_vehicles.append(vehicle)
                        rear_distances.append(ego_distance)

                if len(front_distances) is not 0:
                    front_min_index = np.argmin(front_distances)
                    closest_front_vehicle = front_vehicles[front_min_index]
                    closest_distance_from_front_vehicle = front_distances[front_min_index]

                if len(rear_distances) is not 0:
                    rear_min_index = np.argmin(rear_distances)
                    closest_rear_vehicle = rear_vehicles[rear_min_index]
                    closest_distance_from_rear_vehicle = rear_distances[rear_min_index]

                #print("Front ", front_vehicles)
                #print("Rear ", rear_vehicles)
                
                # check for lane change
                self.turn = self.turn_sub.get_turn()
                print(self.turn)
                success = self.overtake()

                if not success:
                    self.manual_lane_change()         
                                        
                self.world.debug.draw_string(self.waypoints[self.index].transform.location, "X", draw_shadow=False, color=carla.Color(r=0, g=0, b=255), life_time=1000, persistent_lines=True)
                
               
                # change goal --need-change-topic 
                behavior = self.sub_behavior.get_behavior()
                if behavior == True:
                    behavior = self.sub_behavior.set_behavior(False)
                    self.emergency_stop()

                    control_signal = self.custom_controller.run_step(self.velocity, self.waypoints[self.index])
                    vehicle_actor.apply_control(control_signal)

                    self.publish_velocity()
                    break
            
                if self.sub_agg.get_aggressive():
                    self.aggressive()
                    self.sub_agg.set_aggressive()

                if self.sub_caut.get_cautious():
                    self.cautious()
                    self.sub_caut.set_cautious()

                # check for red lights, front obstacles and stop button 
                if traffic_light_state == "RED" and False:

                    # if RED light has activated when vehicle was inside the junction then it is better to get out of
                    # the junction
                    if not self.waypoints[self.index].is_junction:
                        self.emergency_stop()
                    else:
                        control_signal = self.custom_controller.run_step(self.velocity, self.waypoints[self.index])

                elif closest_distance_from_front_vehicle < 15:

                    # set False in order to check if obstacle detector has triggered again
                    set_front_obstacle(False)

                    obstacle_detected = vehicle.id
                  

                    if previous_obstacle_detected != obstacle_detected and "vehicle" in vehicle.type_id:
                        
                        print("New obstacle")
                          
                        velocity_vec = vehicle.get_velocity()
                        velocity_obs_array = [velocity_vec.x, velocity_vec.y, velocity_vec.z]
                        velocity_obstacle = np.linalg.norm(velocity_obs_array)
                        velocity_obstacle = round(3.6 * velocity_obstacle, 1)
                        
                        current_waypoint = self.map.get_waypoint(self.vehicle_actor.get_location(), project_to_road=True)

                        if self.waypoints[self.index].get_left_lane() != None and "Solid" not in str(self.waypoints[self.index].left_lane_marking.type):
                            self.turn_obstacle = "LEFT"
                                                    
                        elif self.waypoints[self.index].get_right_lane() != None and "Solid" not in str(self.waypoints[self.index].right_lane_marking.type):
                            self.turn_obstacle = "RIGHT"

                        previous_obstacle_detected = obstacle_detected

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

                previous_velocity = self.velocity

                vehicle_actor.apply_control(control_signal)
                world.tick()

            except KeyboardInterrupt:
                break
