#!/usr/bin/python3 
from vehicle import EgoVehicle
from sensor import Sensor, Lidar, CameraRGB, GNSS, IMU, ObstacleDetector, LaneInvasionDetector, Radar, CameraSemantic
from client import Client
from utilities import plot_axis
from trajectory import generate_random_trajectory
#from agents.navigation.roaming_agent import RoamingAgent
#from agents.navigation.behavior_agent import BehaviorAgent
#from agents.navigation.basic_agent import BasicAgent 
from agents.navigation.controller import VehiclePIDController
import numpy as np 
import carla
import time  
import random 

def main():

    #----create client----# 
    client = Client()                                       
    client.connect()                                        # connect the client 
    [blueprint, world, map]= client.get_simulation()        

    points = map.get_spawn_points()                         # returns a list of recommendations 
    start_waypoint = map.get_waypoint(points[0].location)         # return the waypoint of the spawn point 
    start_point = points[0]                                 # choose first point as spawn point
    
    #----create new ego vehicle----# 
    vehicle = EgoVehicle()                                  
    vehicle.choose_spawn_point(start_point)                 # spawn the vehicle 
    vehicle.choose_model('model3', blueprint, world)        # choose the model 
    vehicle_actor = vehicle.get_vehicle_actor()             # return actor object of the vehicle 
    vehicle_transform = vehicle.get_vehicle_transform()     # get vehicle's transform 
    
    #----plot axis----#
    origin = carla.Transform()                              # plot map's origin
    plot_axis(world, origin)

    origin = start_point                                    # plot vehicle's starting coordinate frame 
    plot_axis(world, origin)

    #world.debug.draw_string(map.get_waypoint(vehicle_actor.get_location()).transform.location, 'O', draw_shadow=False, color=carla.Color(r=255, g=200, b=0), life_time=50, persistent_lines=True)  
    #agent = BasicAgent(vehicle_actor)
    #agent.set_destination([-6.446170, -50.055023, 0.275307])
    #world.debug.draw_string( carla.Location(-6.446170, -50.055023, 0.275307), 'O', draw_shadow=False, color=carla.Color(r=0, g=0, b=255), life_time=100, persistent_lines=True)

    #waypoints = map.get_topology()
    #waypoints = map.generate_waypoints(3.0)
    #fileData = open("data.txt", 'w')
    #for i in waypoints:
    #    fileData.write(str(i[0].transform.location.x) + "  " + str(i[0].transform.location.y) + "  " + str(i[0].transform.location.z) + "    " + str(i[1].transform.location.x) + "  " + str(i[1].transform.location.y) + "  " + str(i[1].transform.location.z) + "\n")    
    #for waypoint in waypoints:
    #    if waypoint.lane_id > 0:
    #        world.debug.draw_string(waypoint.transform.location, '{}'.format(waypoint.lane_change), draw_shadow=False, color=carla.Color(r=0, g=255, b=0), life_time=150, persistent_lines=True)
    #    else:
    #        world.debug.draw_string(waypoint.transform.location, '{}'.format(waypoint.lane_change), draw_shadow=False, color=carla.Color(r=255, g=0, b=0), life_time=150, persistent_lines=True)
    #        world.debug.draw_string(waypoint[1].transform.location, 'O', draw_shadow=False, color=carla.Color(r=0, g=255, b=0), life_time=150, persistent_lines=True)
    
    custom_controller = VehiclePIDController(vehicle_actor, args_lateral = {'K_P': 1, 'K_D': 0, 'K_I': 0}, args_longitudinal = {'K_P': 1, 'K_D': 0, 'K_I': 0})
    
    waypoints = generate_random_trajectory(world, start_waypoint)
    
    i = 0
    while True:
        try:
            control_signal = custom_controller.run_step(15, waypoints[i])
            p1 = [waypoints[i].transform.location.x, waypoints[i].transform.location.y, waypoints[i].transform.location.z]
            p2 = [vehicle_actor.get_location().x, vehicle_actor.get_location().y, vehicle_actor.get_location().z]
            dist = np.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2 + (p2[2] - p1[2])**2)
            print('Distance from waypoint {}'.format(i), dist)
            
            if dist < 2:
                i += 1
            
            if i == len(waypoints):
                control_signal = custom_controller.run_step(0, waypoints[i - 1])
                break

            vehicle_actor.apply_control(control_signal)
            world.tick()      
        except KeyboardInterrupt:
            break    
    
    '''
    lidar = Lidar()                  # create a sensor 
    lidar.set_vehicle(vehicle_actor)                        # give the vehicle that the sensor will be attached to  
    lidar.set_rotation(0,0,0)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           
    lidar.set_location(0,0,2)
    lidar.set_simulation(blueprint, world, map)
    lidar.set_sensor()
    
    camera_rgb = CameraRGB()                 # create a sensor 
    camera_rgb.set_vehicle(vehicle_actor)                        # give the vehicle that the sensor will be attached to  
    camera_rgb.set_rotation(0,0,0)
    camera_rgb.set_location(0,0,2)
    camera_rgb.set_simulation(blueprint, world, map)
    camera_rgb.set_sensor()

    camera_sem = CameraSemantic()                 # create a sensor 
    camera_sem.set_vehicle(vehicle_actor)                        # give the vehicle that the sensor will be attached to  
    camera_sem.set_rotation(0,0,0)
    camera_sem.set_location(0,0,3)
    camera_sem.set_simulation(blueprint, world, map)
    camera_sem.set_sensor()

    gnss = GNSS()                 # create a sensor 
    gnss.set_vehicle(vehicle_actor)                        # give the vehicle that the sensor will be attached to  
    gnss.set_rotation(0,0,0)
    gnss.set_location(0,0,0)
    gnss.set_simulation(blueprint, world, map)
    gnss.set_sensor()

    imu = IMU()                 # create a sensor 
    imu.set_vehicle(vehicle_actor)                        # give the vehicle that the sensor will be attached to  
    imu.set_rotation(0,0,0)
    imu.set_location(0,0,0)
    imu.set_simulation(blueprint, world, map)
    imu.set_sensor()

    obs = ObstacleDetector()                 # create a sensor 
    obs.set_vehicle(vehicle_actor)                        # give the vehicle that the sensor will be attached to  
    obs.set_rotation(0,0,0)
    obs.set_location(0,0,0)
    obs.set_simulation(blueprint, world, map)
    obs.set_sensor()

    lane = LaneInvasionDetector()                 # create a sensor 
    lane.set_vehicle(vehicle_actor)                        # give the vehicle that the sensor will be attached to  
    lane.set_rotation(0,0,0)
    lane.set_location(0,0,0)
    lane.set_simulation(blueprint, world, map)
    lane.set_sensor()

    radar = Radar()                 # create a sensor 
    radar.set_vehicle(vehicle_actor)                        # give the vehicle that the sensor will be attached to  
    radar.set_rotation(0,0,0)
    radar.set_location(0,0,0)
    radar.set_simulation(blueprint, world, map)
    radar.set_sensor()

    camera_sem.read()
    radar.read()
    obs.read()
    imu.read()
    gnss.read()
    lidar.read()
    camera_rgb.read()

    vehicle.add_sensor(lidar)
    vehicle.add_sensor(camera_rgb)
    
    
    vehicle.wander()
    '''
    for actor in world.get_actors():
        actor.destroy()
    print('done.')

    
if __name__ == "__main__":
    main()
