import carla
import numpy as np
from agents.navigation.controller import VehiclePIDController
from utilities import draw_vehicle_box
from traffic import Traffic
from comm_vehicle_sub import VehicleSubscriberMQTT


def follow_random_trajectory(world, vehicle_actor, spectator, waypoints, velocity, front_obstacle, set_front_obstacle):
    
    custom_controller = VehiclePIDController(vehicle_actor, args_lateral = {'K_P': 1, 'K_D': 0, 'K_I': 0}, args_longitudinal = {'K_P': 1, 'K_D': 0, 'K_I': 0})    
    
    print("Running...")
    
    i = 0
    sub = VehicleSubscriberMQTT(topic='start_stop_topic')
    sub.set_start("True")

    while True:
        try:
            #spectator()
            '''
            p1 = [waypoints[i].transform.location.x, waypoints[i].transform.location.y, waypoints[i].transform.location.z]
            p2 = [vehicle_actor.get_location().x, vehicle_actor.get_location().y, vehicle_actor.get_location().z]
            dist = np.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2 + (p2[2] - p1[2])**2)
            '''
            
            p1 = carla.Location(waypoints[i].transform.location.x, waypoints[i].transform.location.y, waypoints[i].transform.location.z)
            p2 = carla.Location(vehicle_actor.get_location().x, vehicle_actor.get_location().y, vehicle_actor.get_location().z)
            dist = p1.distance(p2)
            
            #print('Distance from waypoint {}'.format(i), dist)

            draw_vehicle_box(world, vehicle_actor, vehicle_actor.get_transform().location, vehicle_actor.get_transform().rotation, 0.05)

            traffic = Traffic(world)
            traffic_sign = traffic.check_signs(waypoints[i])
            traffic_light_state = traffic.check_traffic_lights(vehicle_actor)

            left = waypoints[i].get_left_lane()
            right = waypoints[i].get_right_lane()
            if left != None:
                world.debug.draw_string(left.transform.location, '{}'.format(0), draw_shadow=False, color=carla.Color(r=0, g=0, b=255), life_time=1000, persistent_lines=True)
            if right != None:
                world.debug.draw_string(right.transform.location, '{}'.format(1), draw_shadow=False, color=carla.Color(r=255, g=0, b=0), life_time=1000, persistent_lines=True)
            
            if traffic_light_state == "RED" or front_obstacle() == True:
                set_front_obstacle(False)                                               # set False in order to check if obstacle detector has triggered again  
                control_signal = custom_controller.run_step(0, waypoints[i])
            else:
                control_signal = custom_controller.run_step(velocity, waypoints[i])

            if dist < 2:
                i += 1
            
            if i == len(waypoints):
                control_signal = custom_controller.run_step(0, waypoints[i - 1])
                break

            # check if stop button have pressed 
            stop = sub.get_stop()
            if stop == True: 
                control_signal = custom_controller.run_step(0, waypoints[i])

            # check if start button have pressed 
            start = sub.get_start()
            if start == True:
                control_signal = custom_controller.run_step(velocity, waypoints[i])

            vehicle_actor.apply_control(control_signal)
            world.tick()      

            
        
        except KeyboardInterrupt:
            break    
    
