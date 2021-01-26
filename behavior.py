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
    def __init__(self):
        self.pub_vel = VehiclePublisherMQTT(topic='speed_configure')
 

    def slow_down(self, current_velocity, desired_velocity, limit):
        if current_velocity > limit:
            current_velocity = desired_velocity
            vel = {'velocity': current_velocity}  
            self.pub_vel.publish(vel)
        return current_velocity

    def follow_random_trajectory(self, world, vehicle_actor, spectator, waypoints, front_obstacle, set_front_obstacle, trajectory):
    
        custom_controller = VehiclePIDController(vehicle_actor, args_lateral = {'K_P': 1, 'K_D': 0, 'K_I': 0}, args_longitudinal = {'K_P': 1, 'K_D': 0, 'K_I': 0})    
        print("Running...")
    
        i = 0
        sub = VehicleSubscriberStartStopMQTT(topic='start_stop_topic')
        speed_sub = VehicleSubscriberVelocityMQTT(topic='speed_configure')
        turn_sub = VehicleSubscriberLeftRightMQTT(topic='turn')

        sub_pos = VehicleSubscriberPositionMQTT(topic='position')

        pub = VehiclePublisherMQTT(topic='speed_topic')
        pub_notify = VehiclePublisherMQTT(topic='turn notification')
        current_state = "INIT"
        #spawn()

        vel = {'velocity': 0}  
        self.pub_vel.publish(vel)
 
        while True:
            try:
                spectator()
                print(sub_pos.get_pos())
                '''
                p1 = [waypoints[i].transform.location.x, waypoints[i].transform.location.y, waypoints[i].transform.location.z]
                p2 = [vehicle_actor.get_location().x, vehicle_actor.get_location().y, vehicle_actor.get_location().z]
                dist = np.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2 + (p2[2] - p1[2])**2)
                '''

                # velocity's norm in km/h
                velocity_vector = vehicle_actor.get_velocity()
                velocity_array = [velocity_vector.x, velocity_vector.y, velocity_vector.z] 
                velocity_norm = np.linalg.norm(velocity_array)
                vel = {'velocity': round(3.6 * velocity_norm, 1)}  
                pub.publish(vel)
                

                #print('Distance from waypoint {}'.format(i), dist)
                #draw_vehicle_box(world, vehicle_actor, vehicle_actor.get_transform().location, vehicle_actor.get_transform().rotation, 0.05)

                traffic = Traffic(world)
                traffic_sign = traffic.check_signs(waypoints[i])
                traffic_light_state = traffic.check_traffic_lights(vehicle_actor)
                stop = sub.get_stop()
                velocity = speed_sub.get_velocity()
                '''
                left = waypoints[i].get_left_lane()
                right = waypoints[i].get_right_lane()
                if left != None:
                    world.debug.draw_string(left.transform.location, '{}'.format(0), draw_shadow=False, color=carla.Color(r=0, g=0, b=255), life_time=1000, persistent_lines=True)
                if right != None:
                    world.debug.draw_string(right.transform.location, '{}'.format(1), draw_shadow=False, color=carla.Color(r=255, g=0, b=0), life_time=1000, persistent_lines=True)
                '''
                turn = turn_sub.get_turn()
                #print("turn:", turn)
                #print("current_state:", current_state)

                # check for lane change
                if trajectory.change == False and i != len(waypoints):
                    if current_state == "INIT":
                        if turn != None:
                            prev = waypoints[i+1]
                            w = trajectory.change_waypoint(waypoint=i+1, direction=turn)
                            if w != None:
                                waypoints[i+1] = w
                                if w == prev:
                                    current_state = "INIT"
                                else:
                                    current_state = turn
                                    pub_notify.publish({'value': current_state})
                                    velocity = self.slow_down(current_velocity=velocity, desired_velocity=10, limit=10)
                            else:
                                current_state = "INIT"
                            turn = turn_sub.set_turn(None)
                        
                        else:
                            current_state = "INIT"
                            turn = turn_sub.set_turn(None)

                    elif current_state == "LEFT" or current_state == "RIGHT":
                        if turn == current_state or turn == None:
                            prev = waypoints[i+1]
                            if current_state == "LEFT":
                                w = trajectory.change_waypoint(waypoint=i+1, direction="LEFT")
                            elif current_state == "RIGHT":
                                w = trajectory.change_waypoint(waypoint=i+1, direction="RIGHT")

                            if w != None:
                                waypoints[i+1] = w
                                if w == prev:
                                    current_state = "INIT"
                                    pub_notify.publish({'value': current_state})
                                    velocity = self.slow_down(current_velocity=velocity, desired_velocity=10, limit=10)
                            else:
                                current_state = "INIT"
                                pub_notify.publish({'value': current_state})
                                velocity = self.slow_down(current_velocity=velocity, desired_velocity=10, limit=10)
                            turn = turn_sub.set_turn(None)
                            
                        elif turn != current_state:
                            current_state = "INIT"
                            pub_notify.publish({'value': current_state})
                            turn = turn_sub.set_turn(None)
                            velocity = self.slow_down(current_velocity=velocity, desired_velocity=10, limit=10)

                # check for red lights, front obstacles and stop button 
                if traffic_light_state == "RED" or front_obstacle() == True or stop == True:
                    set_front_obstacle(False)                                               # set False in order to check if obstacle detector has triggered again  
                    control_signal = custom_controller.run_step(0, waypoints[i])

                else:
                    control_signal = custom_controller.run_step(velocity, waypoints[i])
                
                p1 = carla.Location(waypoints[i].transform.location.x, waypoints[i].transform.location.y, waypoints[i].transform.location.z)
                p2 = carla.Location(vehicle_actor.get_location().x, vehicle_actor.get_location().y, vehicle_actor.get_location().z)
                dist = p1.distance(p2)

                if dist < 2:
                    i += 1
                    trajectory.change = False
                


                if i == len(waypoints):
                    control_signal = custom_controller.run_step(0, waypoints[i - 1])
                    break
                #print(control_signal.steer)                
                vehicle_actor.apply_control(control_signal)
                world.tick()      

            except KeyboardInterrupt:
                break    