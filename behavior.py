import carla
import numpy as np
from agents.navigation.controller import VehiclePIDController
from utilities import draw_vehicle_box
from traffic import Traffic
from vehicle_move import spawn
from communicationMQTT import   VehiclePublisherMQTT, \
                                VehicleSubscriberStartStopMQTT, \
                                VehicleSubscriberVelocityMQTT, \
                                VehicleSubscriberLeftRightMQTT, \
                                VehicleSubscriberPositionMQTT

class Behavior(object):
    def __init__(self, vehicle_actor, waypoints, trajectory):
        # controller 
        self.custom_controller = VehiclePIDController(vehicle_actor, args_lateral = {'K_P': 1, 'K_D': 0, 'K_I': 0}, args_longitudinal = {'K_P': 1, 'K_D': 0, 'K_I': 0})    

        # communication attributes 
        self.pub_vel = VehiclePublisherMQTT(topic='speed_configure')
        self.sub = VehicleSubscriberStartStopMQTT(topic='start_stop_topic')
        self.speed_sub = VehicleSubscriberVelocityMQTT(topic='speed_configure')
        self.turn_sub = VehicleSubscriberLeftRightMQTT(topic='turn')
        self.sub_pos = VehicleSubscriberPositionMQTT(topic='position')
        self.pub = VehiclePublisherMQTT(topic='speed_topic')
        self.pub_notify = VehiclePublisherMQTT(topic='turn notification')
       
        self.waypoints = waypoints
        self.velocity = 0
        self.trajectory = trajectory

    def slow_down(self, current_velocity, desired_velocity, limit):
        if current_velocity > limit:
            current_velocity = desired_velocity
            vel = {'velocity': current_velocity}  
            self.pub_vel.publish(vel)
        return current_velocity

    def speed_up(self, current_velocity, desired_velocity):
        if current_velocity < desired_velocity:
            current_velocity = desired_velocity
            vel = {'velocity': current_velocity}  
            self.pub_vel.publish(vel)
        return current_velocity
    
    def emergency_stop(self):
        control = carla.VehicleControl()
        control.brake = 1.0
        control.steer = 0.0
        control.throttle = 0.0
        return control 

    def change_lane(self, turn, i):
        if self.trajectory.change == False and i != len(self.waypoints):
            if self.current_state == "INIT":
                if turn != None:
                    prev = self.waypoints[i+1]
                    w = self.trajectory.change_waypoint(waypoint=i+1, direction=turn)
                    if w != None:
                        self.waypoints[i+1] = w
                        if w == prev:
                            self.current_state = "INIT"
                        else:
                            self.current_state = turn
                            self.pub_notify.publish({'value': self.current_state})
                            self.velocity = self.slow_down(current_velocity=self.velocity, desired_velocity=10, limit=10)
                    else:
                        self.current_state = "INIT"
                    turn = self.turn_sub.set_turn(None)
                
                else:
                    self.current_state = "INIT"
                    turn = self.turn_sub.set_turn(None)

            elif self.current_state == "LEFT" or self.current_state == "RIGHT":
                if turn == self.current_state or turn == None:
                    prev = self.waypoints[i+1]
                    if self.current_state == "LEFT":
                        w = self.trajectory.change_waypoint(waypoint=i+1, direction="LEFT")
                    elif self.current_state == "RIGHT":
                        w = self.trajectory.change_waypoint(waypoint=i+1, direction="RIGHT")

                    if w != None:
                        self.waypoints[i+1] = w
                        if w == prev:
                            self.current_state = "INIT"
                            self.pub_notify.publish({'value': self.current_state})
                            self.velocity = self.slow_down(current_velocity=self.velocity, desired_velocity=10, limit=10)
                    else:
                        self.current_state = "INIT"
                        self.pub_notify.publish({'value': self.current_state})
                        self.velocity = self.slow_down(current_velocity=self.velocity, desired_velocity=10, limit=10)
                    turn = self.turn_sub.set_turn(None)
                    
                elif turn != self.current_state:
                    self.current_state = "INIT"
                    self.pub_notify.publish({'value': self.current_state})
                    turn = self.turn_sub.set_turn(None)
                    self.velocity = self.slow_down(current_velocity=self.velocity, desired_velocity=10, limit=10)

    def follow_trajectory(self, world, vehicle_actor, spectator, front_obstacle, set_front_obstacle):
    
        i = 0
        self.current_state = "INIT"
        #spawn()
        
        vel = {'velocity': 0}  
        self.pub_vel.publish(vel)
 
        while True:
            try:
                spectator()
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

                #print('Distance from waypoint {}'.format(i), dist)
                #draw_vehicle_box(world, vehicle_actor, vehicle_actor.get_transform().location, vehicle_actor.get_transform().rotation, 0.05)

                traffic = Traffic(world)
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
                turn = self.turn_sub.get_turn()
                
                # check for lane change
                self.change_lane(turn, i)

                # check for red lights, front obstacles and stop button 
                if traffic_light_state == "RED" or front_obstacle() == True or stop == True:
                    set_front_obstacle(False)                                               # set False in order to check if obstacle detector has triggered again  
                    control_signal = self.emergency_stop()
                else:
                    control_signal = self.custom_controller.run_step(self.velocity, self.waypoints[i])
                    
                    if abs(control_signal.steer) > 0.6:
                        self.velocity = self.slow_down(current_velocity=self.velocity, desired_velocity=10, limit=10)
                        control_signal = self.custom_controller.run_step(self.velocity, self.waypoints[i])

                p1 = carla.Location(self.waypoints[i].transform.location.x, self.waypoints[i].transform.location.y, self.waypoints[i].transform.location.z)
                p2 = carla.Location(vehicle_actor.get_location().x, vehicle_actor.get_location().y, vehicle_actor.get_location().z)
                dist = p1.distance(p2)

                if dist < 2:
                    i += 1
                    self.trajectory.change = False

                if i == len(self.waypoints):
                    control_signal = self.custom_controller.run_step(0, self.waypoints[i - 1])
                    break


                vehicle_actor.apply_control(control_signal)
                world.tick()      

            except KeyboardInterrupt:
                break    