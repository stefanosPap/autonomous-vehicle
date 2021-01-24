#!/usr/bin/python3 

from vehicle import Vehicle
from client import Client
from utilities import plot_axis, draw_vehicle_box, configure_sensor, save_waypoints, load_waypoints
from trajectory import Trajectory
from behavior import Behavior
from communicationMQTT import VehicleSubscriberStartStopMQTT
from vehicle_move import spawn
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

    #################
    # create client #
    ################# 
    client = Client()                                       
    client.connect()                                        # connect the client 
    [blueprint, world, map]= client.get_simulation()        
    
    points = map.get_spawn_points()                         # returns a list of recommendations 
    start_point = points[0]                                 # choose first point as spawn point
    start_waypoint = map.get_waypoint(start_point.location)   # return the waypoint of the spawn point 

    ##########################
    # create new ego vehicle #
    ##########################
    vehicle = Vehicle()                                  
    vehicle.choose_spawn_point(start_point)                 # spawn the vehicle 
    vehicle.choose_model('model3', blueprint, world)        # choose the model 
    vehicle_actor = vehicle.get_vehicle_actor()             # return actor object of the vehicle 
    vehicle_transform = vehicle.get_vehicle_transform()     # get vehicle's transform 
    #vehicle.set_spectator()
    client.add_actor(vehicle_actor)

    origin = carla.Transform()                              # plot map's origin
    plot_axis(world, origin)

    origin = start_point                                    # plot vehicle's starting coordinate frame 
    plot_axis(world, origin)

    #world.debug.draw_string(map.get_waypoint(vehicle_actor.get_location()).transform.location, 'O', draw_shadow=False, color=carla.Color(r=255, g=200, b=0), life_time=50, persistent_lines=True)  
    #agent = BasicAgent(vehicle_actor)
    #agent.set_destination([-6.446170, -50.055023, 0.275307])
    #world.debug.draw_string( carla.Location(-6.446170, -50.055023, 0.275307), 'O', draw_shadow=False, color=carla.Color(r=0, g=0, b=255), life_time=100, persistent_lines=True)
    
    #spawn()
    waypoints = map.get_topology()
    #print(waypoints)
    #waypoints_map = map.generate_waypoints(3.0)
    #for waypoint in waypoints:
    #    if waypoint.lane_id > 0:
        #world.debug.draw_string(waypoint[0].transform.location, 's', draw_shadow=False, color=carla.Color(r=0, g=0, b=255), life_time=1000, persistent_lines=True)
    #    else:
        #world.debug.draw_string(waypoint[1].transform.location, 'e', draw_shadow=False, color=carla.Color(r=255, g=0, b=0), life_time=1000, persistent_lines=True)


    #pedestrian_actor = world.get_blueprint_library().filter('walker.pedestrian.0001')
    #ped_actor = world.spawn_actor(pedestrian_actor[0], spawn_point)
    #walker_controller_bp = world.get_blueprint_library().find('controller.ai.walker')
    #client.add_actor(ped_actor)
    
    # generate random trajectory for the vehicle 
    trajectory = Trajectory(world, map)
    #waypoints = trajectory.generate_random_trajectory(start_waypoint, number_of_waypoints = 100)

    #save_waypoints(waypoints)
    waypoints = load_waypoints(world, map)
    waypoints = trajectory.load_trajectory(waypoints)
    # configure sensors 
    sensors = configure_sensor(vehicle_actor, vehicle_transform, blueprint, world, map, "ObstacleDetector")
    
    # add sensor to vehicle's configuration
    vehicle.add_sensor(sensors['obs'])                            
    
    # just wander in autopilot mode and collect data
    #vehicle.wander()                                                                  
 
    # wait until start button is pushed     
    sub = VehicleSubscriberStartStopMQTT(topic='start_stop_topic')
    while True:
        world.tick()
        start = sub.get_start()
        if start == True:
            break 

    # follow random trajectory and stop to obstacles and traffic lights
    behavior = Behavior()
    behavior.follow_random_trajectory(world, vehicle_actor, vehicle.set_spectator, waypoints, sensors['obs'].get_front_obstacle, sensors['obs'].set_front_obstacle, trajectory)

    ###########################
    # Destroy actors and exit #
    ###########################
    for actor in client.get_created_actors():
        actor.destroy()
    print('done.')
    world.tick()
    sys.exit()
    
if __name__ == "__main__":
    main()


