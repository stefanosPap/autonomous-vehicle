import carla
import numpy as np
from agents.navigation.controller import VehiclePIDController
from traffic import Traffic
from vehicle_move import spawn
from utilities import change_coordinate_system
from vehicle import Vehicle
from client import Client
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
        if self.closest_distance_from_rear_vehicle < 10:
            self.speed_up(20)

    def car_follow(self):
        velocity_vec = self.closest_front_vehicle.get_velocity()
        velocity_obs_array = [velocity_vec.x, velocity_vec.y, velocity_vec.z]
        velocity_obstacle = np.linalg.norm(velocity_obs_array)
        velocity_obstacle = round(3.6 * velocity_obstacle, 1)
        if velocity_obstacle != self.velocity:
            self.velocity = velocity_obstacle
            self.publish_velocity()

    def check_side_obstacles(self):

        self.front_location = self.vehicle_actor.get_location() + carla.Location(self.vehicle_actor.bounding_box.extent.x, 0, 0)
        self.rear_location  = self.vehicle_actor.get_location() - carla.Location(self.vehicle_actor.bounding_box.extent.x, 0, 0)
        self.ego_vehicle    = self.vehicle_list[0]
        
        self.vehicles_in_right_lane = []
        self.front_right_vehicles   = []
        self.front_right_distances  = []
        self.rear_right_vehicles    = []
        self.rear_right_distances   = []

        self.vehicles_in_left_lane = []
        self.front_left_vehicles   = []
        self.front_left_distances  = []
        self.rear_left_vehicles    = []
        self.rear_left_distances   = []

        self.closest_distance_from_front_right_vehicle = float('inf')
        self.closest_distance_from_rear_right_vehicle  = float('inf')
        
        self.closest_distance_from_front_left_vehicle = float('inf')
        self.closest_distance_from_rear_left_vehicle  = float('inf')
        
        for vehicle in self.vehicle_list[1:]:

            ego_waypoint   = self.map.get_waypoint(self.ego_vehicle.get_location())
            other_waypoint = self.map.get_waypoint(vehicle.get_location())
            
            # check if right lane exists 
            if self.waypoints[self.index].get_right_lane() is not None:
                right_lane_id = self.waypoints[self.index].get_right_lane().lane_id
            
            # check if left lane exists 
            if self.waypoints[self.index].get_left_lane() is not None:
                left_lane_id = self.waypoints[self.index].get_left_lane().lane_id
            
            # check for side obstacles 
            if ego_waypoint.lane_id != other_waypoint.lane_id and ego_waypoint.road_id == other_waypoint.road_id: 

                if right_lane_id == other_waypoint.lane_id: 
                    self.vehicles_in_right_lane.append(vehicle)

                if left_lane_id == other_waypoint.lane_id:
                    self.vehicles_in_left_lane.append(vehicle)


        for vehicle in self.vehicles_in_right_lane:
            vehicle_location = vehicle.get_location()

            ego_distance_front_right = vehicle_location.distance(self.front_location)
            ego_distance_rear_right  = vehicle_location.distance(self.rear_location)
            ego_distance             = vehicle_location.distance(self.ego_vehicle.get_location())

            if ego_distance_front_right < ego_distance_rear_right:
                self.front_right_vehicles.append(vehicle)
                self.front_right_distances.append(ego_distance)
            else:
                self.rear_right_vehicles.append(vehicle)
                self.rear_right_distances.append(ego_distance)

        if len(self.front_right_distances) is not 0:
            front_min_index = np.argmin(self.front_right_distances)
            self.closest_front_right_vehicle = self.front_right_vehicles[front_min_index]
            self.closest_distance_from_front_right_vehicle = self.front_right_distances[front_min_index]

        if len(self.rear_right_distances) is not 0:
            rear_min_index = np.argmin(self.rear_right_distances)
            self.closest_rear_vehicle = self.rear_right_vehicles[rear_min_index]
            self.closest_distance_from_rear_right_vehicle = self.rear_right_distances[rear_min_index]

        for vehicle in self.vehicles_in_left_lane:
            vehicle_location = vehicle.get_location()

            ego_distance_front_left = vehicle_location.distance(self.front_location)
            ego_distance_rear_left  = vehicle_location.distance(self.rear_location)
            ego_distance             = vehicle_location.distance(self.ego_vehicle.get_location())

            if ego_distance_front_left < ego_distance_rear_left:
                self.front_left_vehicles.append(vehicle)
                self.front_left_distances.append(ego_distance)
            else:
                self.rear_left_vehicles.append(vehicle)
                self.rear_left_distances.append(ego_distance)

        if len(self.front_left_distances) is not 0:
            front_min_index = np.argmin(self.front_left_distances)
            self.closest_front_left_vehicle = self.front_left_vehicles[front_min_index]
            self.closest_distance_from_front_left_vehicle = self.front_left_distances[front_min_index]

        if len(self.rear_left_distances) is not 0:
            rear_min_index = np.argmin(self.rear_left_distances)
            self.closest_rear_left_vehicle = self.rear_left_vehicles[rear_min_index]
            self.closest_distance_from_rear_left_vehicle = self.rear_left_distances[rear_min_index]

    def check_obstacles(self):

        self.front_location = self.vehicle_actor.get_location() + carla.Location(self.vehicle_actor.bounding_box.extent.x, 0, 0)
        self.rear_location  = self.vehicle_actor.get_location() - carla.Location(self.vehicle_actor.bounding_box.extent.x, 0, 0)
        self.ego_vehicle    = self.vehicle_list[0]
        
        self.vehicles_in_lane = []
        self.front_vehicles   = []
        self.front_distances  = []
        self.rear_vehicles    = []
        self.rear_distances   = []

        self.closest_distance_from_front_vehicle = float('inf')
        self.closest_distance_from_rear_vehicle  = float('inf')
        
        for vehicle in self.vehicle_list[1:]:

            ego_waypoint   = self.map.get_waypoint(self.ego_vehicle.get_location())
            other_waypoint = self.map.get_waypoint(vehicle.get_location())
            
            if ego_waypoint.lane_id == other_waypoint.lane_id:
                self.vehicles_in_lane.append(vehicle)
       
        for vehicle in self.vehicles_in_lane:
            vehicle_location = vehicle.get_location()

            ego_distance_front = vehicle_location.distance(self.front_location)
            ego_distance_rear  = vehicle_location.distance(self.rear_location)
            ego_distance       = vehicle_location.distance(self.ego_vehicle.get_location())

            if ego_distance_front < ego_distance_rear:
                self.front_vehicles.append(vehicle)
                self.front_distances.append(ego_distance)
            else:
                self.rear_vehicles.append(vehicle)
                self.rear_distances.append(ego_distance)

        if len(self.front_distances) is not 0:
            front_min_index = np.argmin(self.front_distances)
            self.closest_front_vehicle = self.front_vehicles[front_min_index]
            self.closest_distance_from_front_vehicle = self.front_distances[front_min_index]

        if len(self.rear_distances) is not 0:
            rear_min_index = np.argmin(self.rear_distances)
            self.closest_rear_vehicle = self.rear_vehicles[rear_min_index]
            self.closest_distance_from_rear_vehicle = self.rear_distances[rear_min_index]
        
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
        client = Client()                                       
        client.connect()                                        # connect the client 
        [blueprint, world, map]= client.get_simulation()
        
        #spawn(self.vehicle_list)
        '''
        start_point = carla.Transform(carla.Location(x=60.551256, y=-195.809540, z=1), carla.Rotation(pitch=360.000, yaw=1.439560, roll=0.0))
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
            thr += 0.001
            control_signal1 = carla.VehicleControl(throttle=thr)
            vehicle_actor1.apply_control(control_signal1)
            '''
            try:
                if self.index == len(self.waypoints) - 1:
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
                 
                # check for lane change
                self.turn = self.turn_sub.get_turn()

                success = self.overtake()

                if not success:
                    self.manual_lane_change()         

                self.check_obstacles()

                self.check_side_obstacles()
                
                # check if rear obstacle is closely
                #self.check_tailgating()
                
                if self.closest_distance_from_front_vehicle < 15:
                    self.car_follow()
                
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

                elif self.closest_distance_from_front_vehicle < 15 and False:

                    # set False in order to check if obstacle detector has triggered again
                    set_front_obstacle(False)

                    obstacle_detected = self.closest_front_vehicle.id
                  

                    if previous_obstacle_detected != obstacle_detected and "vehicle" in self.closest_front_vehicle.type_id:
                        
                        print("New obstacle")
                          
                        velocity_vec = self.closest_front_vehicle.get_velocity()
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
