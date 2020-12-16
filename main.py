#!/usr/bin/python3 

from vehicle import Vehicle
from sensor import Sensor, Lidar, CameraRGB, GNSS, IMU, ObstacleDetector, LaneInvasionDetector, Radar, CameraSemantic
from client import Client
from utilities import plot_axis, draw_vehicle_box
from trajectory import generate_random_trajectory
from behavior import follow_random_trajectory
#from agents.navigation.roaming_agent import RoamingAgent
#from agents.navigation.behavior_agent import BehaviorAgent
#from agents.navigation.basic_agent import BasicAgent 
import numpy as np 
import carla
import time  
import random 
import sys
import glob
import os  
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
def main():

    
    ####-----create client----#### 
    client = Client()                                       
    client.connect()                                        # connect the client 
    [blueprint, world, map]= client.get_simulation()        

    points = map.get_spawn_points()                         # returns a list of recommendations 
    start_waypoint = map.get_waypoint(points[0].location)   # return the waypoint of the spawn point 
    start_point = points[0]                                 # choose first point as spawn point
    
    ####----create new ego vehicle----####
    vehicle = Vehicle()                                  
    vehicle.choose_spawn_point(start_point)                 # spawn the vehicle 
    vehicle.choose_model('model3', blueprint, world)        # choose the model 
    vehicle_actor = vehicle.get_vehicle_actor()             # return actor object of the vehicle 
    vehicle_transform = vehicle.get_vehicle_transform()     # get vehicle's transform 
    client.add_actor(vehicle_actor)

    ####------plot axis------####
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
    for k in range(1,10,5):
        spawn_point = carla.Transform()
        spawn_point.location = start_point.location + carla.Location(k,38,0)
        spawn_point.rotation = start_point.rotation
        vehicle2 = Vehicle()                                  
        vehicle2.choose_spawn_point(spawn_point)                 # spawn the vehicle 
        vehicle2.choose_model('model3', blueprint, world)
        vehicle_actor2 = vehicle2.get_vehicle_actor()

        draw_vehicle_box(world, vehicle_actor, spawn_point.location, spawn_point.rotation, 100)

    pedestrian_actor = world.get_blueprint_library().filter('walker.pedestrian.0001')
    #ped_actor = world.spawn_actor(pedestrian_actor[0], spawn_point)
    
    walker_controller_bp = world.get_blueprint_library().find('controller.ai.walker')
    #client.add_actor(ped_actor)
    
    waypoints = generate_random_trajectory(world, start_waypoint, map, number_of_waypoints = 200)
    
    '''
    lidar = Lidar()                                             # create a lidar sensor 
    lidar.set_vehicle(vehicle_actor)                            # give the vehicle that the sensor will be attached to  
    lidar.set_rotation(0,0,0)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           
    lidar.set_location(0,0,2)
    lidar.set_simulation(blueprint, world, map)
    lidar.set_sensor()

    
    camera_rgb = CameraRGB()                                    # create an RGB Camera sensor 
    camera_rgb.set_vehicle(vehicle_actor)                       # give the vehicle that the sensor will be attached to  
    camera_rgb.set_rotation(0,0,0)
    camera_rgb.set_location(0,0,2)
    camera_rgb.set_simulation(blueprint, world, map)
    camera_rgb.set_sensor()

    camera_sem = CameraSemantic()                               # create a Semantic Segmentation Camera sensor 
    camera_sem.set_vehicle(vehicle_actor)                       # give the vehicle that the sensor will be attached to  
    camera_sem.set_rotation(0,0,0)
    camera_sem.set_location(0,0,3)
    camera_sem.set_simulation(blueprint, world, map)
    camera_sem.set_sensor()

    gnss = GNSS()                                               # create a GNSS sensor 
    gnss.set_vehicle(vehicle_actor)                             # give the vehicle that the sensor will be attached to  
    gnss.set_rotation(0,0,0)
    gnss.set_location(0,0,0)
    gnss.set_simulation(blueprint, world, map)
    gnss.set_sensor()

    imu = IMU()                                                 # create an IMU sensor 
    imu.set_vehicle(vehicle_actor)                              # give the vehicle that the sensor will be attached to  
    imu.set_rotation(0,0,0)
    imu.set_location(0,0,0)
    imu.set_simulation(blueprint, world, map)
    imu.set_sensor()
    '''
    obs = ObstacleDetector()                                    # create an Obstacle detector sensor 
    obs.set_vehicle(vehicle_actor)                              # give the vehicle that the sensor will be attached to  
    obs.set_rotation(vehicle_transform.rotation.pitch, vehicle_transform.rotation.yaw, vehicle_transform.rotation.roll)
    obs.set_location(X=2,Y=0,Z=1)
    obs.set_simulation(blueprint, world, map)
    obs.set_sensor(hit_radius=10)

    '''
    lane = LaneInvasionDetector()                               # create a Lane Invasion detector sensor 
    lane.set_vehicle(vehicle_actor)                             # give the vehicle that the sensor will be attached to  
    lane.set_rotation(0,0,0)
    lane.set_location(0,0,0)
    lane.set_simulation(blueprint, world, map)
    lane.set_sensor()
    
    radar = Radar()                                             # create a sensor 
    radar.set_vehicle(vehicle_actor)                            # give the vehicle that the sensor will be attached to  
    radar.set_rotation(pitch=5,yaw=0,roll=0)
    radar.set_location(X=2,Y=0,Z=1)
    radar.set_simulation(blueprint, world, map)
    radar.set_sensor(horizontal_fov=35, vertical_fov=20)
    '''
    
    #camera_sem.read()                                           # read data  
    #radar.read()
    obs.read()
    #imu.read()
    #gnss.read()
    #lidar.read()
    #camera_rgb.read()
    
    '''
    vehicle.add_sensor(lidar)                                   # add sensor to vehicle's configuration
    vehicle.add_sensor(camera_rgb)
    vehicle.add_sensor(camera_sem)
    vehicle.add_sensor(obs)
    vehicle.add_sensor(imu)
    vehicle.add_sensor(gnss)
    vehicle.add_sensor(radar)
    '''
    #vehicle.wander()                                             # just wander in autopilot mode and collect data                     

    follow_random_trajectory(world, vehicle_actor, waypoints, 15, obs.get_front_obstacle, obs.set_front_obstacle, start_point)

    for actor in client.get_created_actors():
        actor.destroy()
    print('done.')
    world.tick()
    sys.exit()
    
if __name__ == "__main__":
    main()


