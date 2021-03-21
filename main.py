#!/usr/bin/python3 

from vehicle import Vehicle
from client import Client

from utilities import plot_axis, draw_vehicle_box, configure_sensor, save_waypoints, load_waypoints, pruning, draw_waypoints
from trajectory import Trajectory
from behavior import Behavior
from communicationMQTT import VehicleSubscriberStartStopMQTT, \
                              VehicleSubscriberCoorMQTT, \
                              VehicleSubscriberEnterMQTT, \
                              VehicleSubscriberDoneMQTT, \
                              VehiclePublisherMQTT

from interface import Interface
from vehicle_move import spawn
#from agents.navigation.roaming_agent import RoamingAgent
#from agents.navigation.behavior_agent import BehaviorAgent
from agents.navigation.basic_agent import BasicAgent 
from agents.navigation.global_route_planner_dao import GlobalRoutePlannerDAO  
from agents.navigation.global_route_planner import GlobalRoutePlanner  

import numpy as np
import carla
import time  
import random 
import sys
import glob
import os  
import copy 
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
    
    #world = client.get_client().load_world('/Game/Carla/Maps/Town02')
    #world = client.get_client().reload_world()
    #print(client.get_client().get_available_maps())
    points = map.get_spawn_points()                         # returns a list of recommendations 
    #start_point = carla.Transform(carla.Location(x=-95.793716, y=-3.109917, z=0.275307), carla.Rotation(pitch=0.0, yaw=-179.705399, roll=0.0))
    while True:
        start_point = random.choice(points)                                              # choose first point as spawn point
        start_waypoint = map.get_waypoint(start_point.location, project_to_road=False)   # return the waypoint of the spawn point 
        if start_waypoint.get_junction() == None:                                        # spawn vehicles only on roads, not junctions
            break 


    #print(start_point)
    #print(start_waypoint)
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
    
    # configure sensors 
    sensors = configure_sensor(vehicle_actor, vehicle_transform, blueprint, world, map, "ObstacleDetector")

    
    # add sensor to vehicle's configuration
    vehicle.add_sensor(sensors['obs'])                            
    
    # just wander in autopilot mode and collect data
    # vehicle.wander() 
   
    #world.debug.draw_string(map.get_waypoint(vehicle_actor.get_location()).transform.location, 'O', draw_shadow=False, color=carla.Color(r=255, g=200, b=0), life_time=50, persistent_lines=True)  
    #agent = BasicAgent(vehicle_actor)
    #agent.set_destination([-6.446170, -50.055023, 0.275307])
    #world.debug.draw_string( carla.Location(-6.446170, -50.055023, 0.275307), 'O', draw_shadow=False, color=carla.Color(r=0, g=0, b=255), life_time=100, persistent_lines=True)
    ba = BasicAgent(vehicle_actor)

    #gp_dao = GlobalRoutePlannerDAO(map, 4)
    #gp = GlobalRoutePlanner(gp_dao)
    #top = gp.get_topology()
    #spawn()
    #waypoints = map.get_topology()
    #waypoints = map.generate_waypoints(3.0)
    #for waypoint in waypoints:
    #    for waypoint in top[i]['path']:
    #        world.debug.draw_string(waypoint.transform.location, '{}'.format(round(waypoint.transform.location.y)), draw_shadow=False, color=carla.Color(r=0, g=0, b=255), life_time=1000, persistent_lines=True)
    #        world.debug.draw_string(waypoint[1].transform.location, 'e', draw_shadow=False, color=carla.Color(r=255, g=0, b=0), life_time=1000, persistent_lines=True)
    #print(top)

    #pedestrian_actor = world.get_blueprint_library().filter('walker.pedestrian.0001')
    #ped_actor = world.spawn_actor(pedestrian_actor[0], spawn_point)
    #walker_controller_bp = world.get_blueprint_library().find('controller.ai.walker')
    #client.add_actor(ped_actor)
    
    # generate random trajectory for the vehicle 
    sub_coor = VehicleSubscriberCoorMQTT(topic='coordinates')
    sub_enter = VehicleSubscriberEnterMQTT(topic='enter')
    sub_done = VehicleSubscriberDoneMQTT(topic='done')
    pub = VehiclePublisherMQTT(topic='text')
    pub_vel = VehiclePublisherMQTT(topic='speed_topic')
    pub_vel_conf = VehiclePublisherMQTT(topic='speed_configure')
    pub_waypoint = VehiclePublisherMQTT(topic='waypoint_choose')
    pub_enter = VehiclePublisherMQTT(topic='enter_info')
    pub_done = VehiclePublisherMQTT(topic='done_info')
    pub_start = VehiclePublisherMQTT(topic='start notification')

    msg = {'value': 'Press ENTER for inserting new waypoint'}
    pub_enter.publish(msg)
    
    msg = {'value': 'Press DONE for finishing waypoint selection'}
    pub_done.publish(msg)
    #custom_point = carla.Transform(carla.Location(x=-125.793716, y=-4 , z=0.275307), carla.Rotation(pitch=0.0, yaw=-179.705399, roll=0.0))
    #custom_waypoint = map.get_waypoint(custom_point.location, project_to_road=False, lane_type=carla.LaneType.Any)
    #end_point = carla.Transform(carla.Location(x=-137.793716, y=-1.8, z=0.275307), carla.Rotation(pitch=0.0, yaw=-179.705399, roll=0.0))
    #end_waypoint = map.get_waypoint(end_point.location, project_to_road=False, lane_type=carla.LaneType.Any)
        
    #index = len(waypoints) - 1
    #location = [stop_point.location.x, stop_point.location.y, stop_point.location.z]
    #location = [waypoints[index].transform.location.x, waypoints[index].transform.location.y, waypoints[index].transform.location.z]
    #waypoints.append(start_point)
    #waypoints.append(custom_point)
    #waypoints.append(end_waypoint)
    '''
        while True:
            end_point = random.choice(points)
            end_waypoint = map.get_waypoint(end_point.location, project_to_road=False, lane_type=carla.LaneType.Any)
            if end_waypoint == None:
                    continue
            #distance = end_waypoint.transform.location.distance(start_waypoint.transform.location)
            if end_waypoint.get_junction() == None: 
            #and distance < 20 and end_waypoint != start_waypoint:
                break 
    '''
    
    interface = Interface(world, map)
    trajectory = Trajectory(world, map)

    while True:
        end_waypoints = interface.handle(start_waypoint)
        waypoints = []
        for k in range(len(end_waypoints) - 1): 
            route = ba._trace_route(end_waypoints[k], end_waypoints[k+1])
            world.debug.draw_string(end_waypoints[k + 1].transform.location, '{}'.format(end_waypoints[k + 1].transform.location.x), draw_shadow=False, color=carla.Color(r=0, g=0, b=0), life_time=1000)
            for waypoint in route:
                waypoints.append(waypoint[0])  

        #save_waypoints(waypoints)
        #waypoints = load_waypoints(world, map)

        waypoints = pruning(map, waypoints)
        '''

        waypoints = []                
        waypoint = start_waypoint
                                        

        while True:
            world.tick()
            if sub_enter.get_enter() == True:
                p = sub_coor.get_coordinates()
                try:
                    p = int(p)
                    
                    text = {'text': ''}
                    pub.publish(text)
                    way = {'value': 'Going forward {} meters'.format(p)}
                    pub_waypoint.publish(way)
                    waypoint = start_waypoint
                    
                    while p > 0:
                        current_waypoints = waypoint.next_until_lane_end(1.0)
                        p = p - len(current_waypoints)
                        if p > 0:
                            waypoints = waypoints + current_waypoints
                        else:
                            waypoints = waypoints + current_waypoints[0:p]
                        waypoint = waypoints[len(waypoints) - 1].next(1.0)

                        if len(waypoint) != 1:
                            for i in range(len(waypoint)):
                                final_waypoint = waypoint[i].next_until_lane_end(1.0)
                                if abs(waypoints[len(waypoints) - 1].transform.rotation.yaw - final_waypoint[len(final_waypoint) - 1].transform.rotation.yaw) < 3:
                                    break 
                            waypoint = waypoint[i]

                        else:
                            waypoint = waypoint[0]
                            
                    break

                except ValueError, TypeError:
                    way = {'value': 'Invalid Value! Try again!'}
                    pub_waypoint.publish(way)
                    text = {'text': ''}
                    pub.publish(text)
                sub_enter.set_enter(False)

                    

                    
            paths = waypoints[len(waypoints) - 1].next(1.0)
            for i in range(len(paths)):
                ways = paths[i].next_until_lane_end(1.0)
                if abs(ways[len(ways) - 1].transform.rotation.yaw - waypoints[len(waypoints) - 1].transform.rotation.yaw) < 3:
                    print("straight")
                elif ways[len(ways) - 1].transform.rotation.yaw > waypoints[len(waypoints) - 1].transform.rotation.yaw:
                    print("right")
                elif ways[len(ways) - 1].transform.rotation.yaw < waypoints[len(waypoints) - 1].transform.rotation.yaw:
                    print("left")
                for i in range(len(ways)):
                    waypoints.append(ways[i])
        '''
        waypoints = trajectory.load_trajectory(waypoints)
        draw_waypoints(world, waypoints, 100)

        # wait until start button is pushed     
        sub = VehicleSubscriberStartStopMQTT(topic='start_stop_topic')
        pub_start.publish({'value': 'Waypoint selection has completed! Press START to begin!'})
        while True:
            world.tick()
            start = sub.get_start()
            if start == True:
                sub.set_start(False)
                pub_start.publish({'value': 'Your car is on! Choose velocity to begin!'})
                break 

        # follow random trajectory and stop to obstacles and traffic lights
        behavior = Behavior(vehicle_actor, waypoints, trajectory, map)
        behavior.follow_trajectory(world, vehicle_actor, vehicle.set_spectator, sensors['obs'].get_front_obstacle, sensors['obs'].set_front_obstacle, 0)
        start_waypoint = map.get_waypoint(vehicle_actor.get_location(), project_to_road=True, lane_type=carla.LaneType.Any)

    col = 100
    while True:
        vel = behavior.get_velocity()
        start_waypoint = map.get_waypoint(vehicle_actor.get_location(), project_to_road=False, lane_type=carla.LaneType.Any)
        while True:
            end_point = random.choice(points)
            end_waypoint = map.get_waypoint(end_point.location, project_to_road=False, lane_type=carla.LaneType.Any)
            if end_waypoint == None:
                continue
            #distance = end_waypoint.transform.location.distance(start_waypoint.transform.location)
            if end_waypoint.get_junction() == None: 
            #and distance < 20 and end_waypoint != start_waypoint:
                break 

        route = ba._trace_route(start_waypoint, end_waypoint)
        waypoints = []
        for waypoint in route:
            waypoints.append(waypoint[0])  

        #save_waypoints(waypoints)
        #waypoints = load_waypoints(world, map)
        col += 10
        waypoints = pruning(map, waypoints)
        waypoints = trajectory.load_trajectory(waypoints)
        draw_waypoints(world, waypoints, 150)
        
        behavior = Behavior(vehicle_actor, waypoints, trajectory, map)
        behavior.follow_trajectory(world, vehicle_actor, vehicle.set_spectator, sensors['obs'].get_front_obstacle, sensors['obs'].set_front_obstacle, vel)

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