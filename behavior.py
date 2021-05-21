import carla
import numpy as np
from agents.navigation.controller import VehiclePIDController
from traffic import Traffic
from vehicle_move import spawn
from communicationMQTT import VehiclePublisherMQTT, \
    VehicleSubscriberStartStopMQTT, \
    VehicleSubscriberVelocityMQTT, \
    VehicleSubscriberLeftRightMQTT, \
    VehicleSubscriberPositionMQTT, \
    VehicleSubscriberBehaviorMQTT, \
    VehicleSubscriberAggressiveMQTT, \
    VehicleSubscriberCautiousMQTT


class Behavior(object):
    def __init__(self, vehicle_actor, waypoints, trajectory, map, world):
        
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

    def change_lane(self, turn, i, num):

        if not self.trajectory.change and i != len(self.waypoints):
        
            if self.current_state == "INIT":

                if turn is not None:
                    prev = self.waypoints[i + 1]
                    w = self.trajectory.change_waypoint(waypoint=i + 1, direction=turn)

                    if w is not None:
                        self.waypoints[i + 1] = w
                        if w == prev:
                            self.current_state = "INIT"
                        else:
                            self.current_state = turn
                            self.pub_notify.publish({'value': self.current_state})
                            self.slow_down(desired_velocity=8)
                            
                    else:
                        self.current_state = "INIT"
                    turn = self.turn_sub.set_turn(None)

                else:
                    self.current_state = "INIT"
                    turn = self.turn_sub.set_turn(None)

            elif self.current_state == "LEFT" or self.current_state == "RIGHT":
        
                if turn == self.current_state or turn == None:   
                    prev = self.waypoints[i + 1]
                    if self.current_state == "LEFT":
                        w = self.trajectory.change_waypoint(waypoint=i + 1, direction="LEFT")
                    elif self.current_state == "RIGHT":
                        w = self.trajectory.change_waypoint(waypoint=i + 1, direction="RIGHT")

                    if w != None:
                        self.waypoints[i + 1] = w
                        if w == prev:
                            self.current_state = "INIT"
                            self.pub_notify.publish({'value': self.current_state})
                            self.slow_down(desired_velocity=8)
                    else:
                        self.current_state = "INIT"
                        self.pub_notify.publish({'value': self.current_state})
                        self.slow_down(desired_velocity=8)
                    turn = self.turn_sub.set_turn(None)

                elif turn != self.current_state:  
                    self.current_state = "INIT"
                    self.pub_notify.publish({'value': self.current_state})
                    turn = self.turn_sub.set_turn(None)
                    self.slow_down(desired_velocity=8)
        
    def cautious(self):
        self.velocity -= self.sub_caut.get_cautious()

    def neutral(self):
        pass

    def aggressive(self):
        self.velocity += self.sub_agg.get_aggressive()
        
    def follow_trajectory(self, world, vehicle_actor, spectator, get_front_obstacle, set_front_obstacle, get_other_actor, velocity):
        i = 0
        self.current_state = "INIT"
        
        turn = None
        turn_obstacle = None
        
        vel = {'velocity': velocity}
        self.pub_vel.publish(vel)

        previous_velocity = 0
        previous_obstacle_detected = None
        
        spawn()

        while True:
            try:

                if i == len(self.waypoints):
                    control_signal = self.custom_controller.run_step(0, self.waypoints[i - 1])
                    vehicle_actor.apply_control(control_signal)
                    break

                # spectator()
                '''
                p1 = [self.waypoints[i].transform.location.x, self.waypoints[i].transform.location.y, self.waypoints[i].transform.location.z]
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
                traffic_sign = traffic.check_signs(self.waypoints[i])
                traffic_light_state = traffic.check_traffic_lights(vehicle_actor)
                stop = self.sub.get_stop()
                self.velocity = self.speed_sub.get_velocity()
                '''
                left = self.waypoints[i].get_left_lane()
                right = self.waypoints[i].get_right_lane()
                if left != None:
                    world.debug.draw_string(left.transform.location, '{}'.format(0), draw_shadow=False, color=carla.Color(r=0, g=0, b=255), life_time=1000, persistent_lines=True)
                if right != None:
                    world.debug.draw_string(right.transform.location, '{}'.format(1), draw_shadow=False, color=carla.Color(r=255, g=0, b=0), life_time=1000, persistent_lines=True)
                '''
                
                # check for lane change
                turn = self.turn_sub.get_turn()
                
                if turn_obstacle != None:
                    print(turn_obstacle, 1)
                    self.trajectory.change = False
                    self.change_lane(turn_obstacle, i, 1)
                    turn_obstacle = None
                    
                else:
                    self.change_lane(turn, i, 2)
                

                # change goal --need-change-topic 
                behavior = self.sub_behavior.get_behavior()
                if behavior == True:
                    behavior = self.sub_behavior.set_behavior(False)
                    self.emergency_stop()

                    control_signal = self.custom_controller.run_step(self.velocity, self.waypoints[i])
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
                    if not self.waypoints[i].is_junction:
                        self.emergency_stop()
                    else:
                        control_signal = self.custom_controller.run_step(self.velocity, self.waypoints[i])

                elif get_front_obstacle():

                    # set False in order to check if obstacle detector has triggered again
                    set_front_obstacle(False)

                    obstacle_detected = get_other_actor().id

                    if previous_obstacle_detected != obstacle_detected and "vehicle" in get_other_actor().type_id:
                        
                        print("New obstacle")
                        
                        current_waypoint = self.map.get_waypoint(self.vehicle_actor.get_location(), project_to_road=True)

                        if self.waypoints[i].get_left_lane() != None and "Solid" not in str(self.waypoints[i].left_lane_marking.type):
                            turn_obstacle = "LEFT"
                                                    
                        elif self.waypoints[i].get_right_lane() != None and "Solid" not in str(self.waypoints[i].right_lane_marking.type):
                            turn_obstacle = "RIGHT"

                        previous_obstacle_detected = obstacle_detected

                    else:
                        pass

                elif stop:
                    self.emergency_stop()
                
                
                control_signal = self.custom_controller.run_step(self.velocity, self.waypoints[i])
                #if abs(control_signal.steer) > 0.6:
                #    self.slow_down(desired_velocity=10)
                #    control_signal = self.custom_controller.run_step(self.velocity, self.waypoints[i])

                
                if isinstance(self.waypoints[i], carla.libcarla.Waypoint):
                    p1 = carla.Location(self.waypoints[i].transform.location.x, self.waypoints[i].transform.location.y,
                                        self.waypoints[i].transform.location.z)
                    
                elif isinstance(self.waypoints[i], carla.libcarla.Transform):
                    p1 = carla.Location(self.waypoints[i].location.x, self.waypoints[i].location.y,
                                        self.waypoints[i].location.z)

                p2 = carla.Location(vehicle_actor.get_location().x, vehicle_actor.get_location().y,
                                    vehicle_actor.get_location().z)
                dist = p1.distance(p2)
                #print('Distance from waypoint {}'.format(i), dist) 

                if dist < 2:
                    i += 1
                    self.trajectory.change = False

                previous_velocity = self.velocity

                vehicle_actor.apply_control(control_signal)
                world.tick()

            except KeyboardInterrupt:
                break
