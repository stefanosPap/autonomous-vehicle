import carla
import numpy as np
from agents.navigation.controller import VehiclePIDController
from utilities import check_traffic_lights, draw_vehicle_box

def follow_random_trajectory(world, vehicle_actor, waypoints, velocity, front_obstacle, set_front_obstacle, start_point):
    custom_controller = VehiclePIDController(vehicle_actor, args_lateral = {'K_P': 1, 'K_D': 0, 'K_I': 0}, args_longitudinal = {'K_P': 1, 'K_D': 0, 'K_I': 0})    
    print("start")
    i = 0
    bb = vehicle_actor.bounding_box
    bbox = carla.BoundingBox(start_point.location, bb.extent)
    world.debug.draw_box(bbox, start_point.rotation, 0.1, carla.Color(255,0,0),1)
    while True:
        try:
            
            '''
            p1 = [waypoints[i].transform.location.x, waypoints[i].transform.location.y, waypoints[i].transform.location.z]
            p2 = [vehicle_actor.get_location().x, vehicle_actor.get_location().y, vehicle_actor.get_location().z]
            dist = np.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2 + (p2[2] - p1[2])**2)
            '''
            p1 = carla.Location(waypoints[i].transform.location.x, waypoints[i].transform.location.y, waypoints[i].transform.location.z)
            p2 = carla.Location(vehicle_actor.get_location().x, vehicle_actor.get_location().y, vehicle_actor.get_location().z)
            dist = p1.distance(p2)
            
            #print('Distance from waypoint {}'.format(i), dist)
            land = waypoints[i].get_landmarks(distance=3)
            if len(land) != 0:
                for j in range(len(land)):
                    world.debug.draw_box(carla.BoundingBox(land[j].transform.location, carla.Vector3D(0.5,0.5,2)), land[j].transform.rotation, 0.05, carla.Color(255,0,0,0),100)
                    print(land[j].name)
            
            draw_vehicle_box(world, vehicle_actor, vehicle_actor.get_transform().location, vehicle_actor.get_transform().rotation, 0.05)


            traffic_light_state = check_traffic_lights(vehicle_actor)
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

            vehicle_actor.apply_control(control_signal)
            world.tick()      
        except KeyboardInterrupt:
            break    
    
