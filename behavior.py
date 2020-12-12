import carla
import numpy as np
from agents.navigation.controller import VehiclePIDController
from utilities import check_traffic_lights

def follow_random_trajectory(world, vehicle_actor, waypoints):
    custom_controller = VehiclePIDController(vehicle_actor, args_lateral = {'K_P': 1, 'K_D': 0, 'K_I': 0}, args_longitudinal = {'K_P': 1, 'K_D': 0, 'K_I': 0})    
    
    i = 0
    while True:
        try:
            traffic_light_state = check_traffic_lights(vehicle_actor)
            if traffic_light_state == "RED":
                control_signal = custom_controller.run_step(0, waypoints[i])
            else:
                control_signal = custom_controller.run_step(15, waypoints[i])

            p1 = [waypoints[i].transform.location.x, waypoints[i].transform.location.y, waypoints[i].transform.location.z]
            p2 = [vehicle_actor.get_location().x, vehicle_actor.get_location().y, vehicle_actor.get_location().z]
            dist = np.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2 + (p2[2] - p1[2])**2)
            #print('Distance from waypoint {}'.format(i), dist)
            
            if dist < 2:
                i += 1
            
            if i == len(waypoints):
                control_signal = custom_controller.run_step(0, waypoints[i - 1])
                break

            vehicle_actor.apply_control(control_signal)
            world.tick()      
        except KeyboardInterrupt:
            break    
    
