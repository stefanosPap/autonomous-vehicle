#!/usr/bin/python3 

from vehicle import Vehicle
from client import Client

from utilities import   plot_axis, \
                        draw_vehicle_box, \
                        configure_sensor, \
                        save_waypoints, \
                        load_waypoints, \
                        pruning, \
                        draw_waypoints, \
                        rotate
                        
from trajectory import Trajectory
from behavior import Behavior
from communicationMQTT import VehicleSubscriberStartStopMQTT, \
                              VehicleSubscriberCoorMQTT, \
                              VehicleSubscriberEnterMQTT, \
                              VehicleSubscriberDoneMQTT, \
                              VehicleSubscriberLogMQTT, \
                              VehiclePublisherMQTT, \
                              VehicleSubscriberTurnMQTT

from interface import Interface
from vehicle_move import spawn
#from agents.navigation.roaming_agent import RoamingAgent
#from agents.navigation.behavior_agent import BehaviorAgent
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
    
    
    #start_point = carla.Transform(carla.Location(x=-88.182701, y=66.842422, z=1), carla.Rotation(pitch=0.089919, yaw=89.843735, roll=0.0))
    start_point = carla.Transform(carla.Location(x=0, y=-73, z=0.275307), carla.Rotation(pitch=0.0, yaw=90.0, roll=0.0))

    start_waypoint = map.get_waypoint(start_point.location, project_to_road=True)
    #print(start_waypoint)
    #print(start_point)
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
    #sensors = configure_sensor(vehicle_actor, vehicle_transform, blueprint, world, map, "ObstacleDetector")

    
    # add sensor to vehicle's configuration
    #vehicle.add_sensor(sensors['obs'])                            
    
    # just wander in autopilot mode and collect data
    # vehicle.wander() 
   
    #world.debug.draw_string(map.get_waypoint(vehicle_actor.get_location()).transform.location, 'O', draw_shadow=False, color=carla.Color(r=255, g=200, b=0), life_time=50, persistent_lines=True)  
    #agent = BasicAgent(vehicle_actor)
    #agent.set_destination([-6.446170, -50.055023, 0.275307])
    #world.debug.draw_string( carla.Location(-6.446170, -50.055023, 0.275307), 'O', draw_shadow=False, color=carla.Color(r=0, g=0, b=255), life_time=100, persistent_lines=True)

    #gp_dao = GlobalRoutePlannerDAO(map, 4)
    #gp = GlobalRoutePlanner(gp_dao)
    #top = gp.get_topology()
    #spawn()
    waypoints = map.get_topology()
    #waypoints = map.generate_waypoints(3.0)
    for waypoint in waypoints:
    #    for waypoint in top[i]['path']:
    #        world.debug.draw_string(waypoint.transform.location, '{}'.format(round(waypoint.transform.rotation.yaw)), draw_shadow=False, color=carla.Color(r=0, g=0, b=255), life_time=1000, persistent_lines=True)
            world.debug.draw_string(waypoint[0].transform.location, 's', draw_shadow=False, color=carla.Color(r=255, g=0, b=0), life_time=1000, persistent_lines=True)
            world.debug.draw_string(waypoint[1].transform.location, 'e', draw_shadow=False, color=carla.Color(r=255, g=0, b=0), life_time=1000, persistent_lines=True)
    #print(top)

    #pedestrian_actor = world.get_blueprint_library().filter('walker.pedestrian.0001')
    #ped_actor = world.spawn_actor(pedestrian_actor[0], spawn_point)
    #walker_controller_bp = world.get_blueprint_library().find('controller.ai.walker')
    #client.add_actor(ped_actor)
    
    # generate random trajectory for the vehicle 
    sub_coor = VehicleSubscriberCoorMQTT(topic='coordinates')
    sub_enter = VehicleSubscriberEnterMQTT(topic='enter')
    sub_done = VehicleSubscriberDoneMQTT(topic='done')
    sub_log = VehicleSubscriberLogMQTT(topic='log')
    sub_turn = VehicleSubscriberTurnMQTT(topic='turn_junction')

    pub = VehiclePublisherMQTT(topic='clean')
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

    msg = {'value': ''}
    pub_waypoint.publish(msg)
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
        
    interface = Interface(world, map, vehicle_actor)
    trajectory = Trajectory(world, map, vehicle_actor)
    waypoints = []
    pub.publish({'value': " "})
    
    while True:
        while True:
            while True:
                world.tick()
                action = " "
                if sub_log.get_log() != " ":
                    action = sub_log.get_log()
                    sub_log.set_log(" ")

                if action == "Location":
                    end_waypoints = interface.handle(start_waypoint)
                    custom_waypoints = trajectory.trace_route(end_waypoints)
                    start_waypoint = custom_waypoints[len(custom_waypoints) - 1]
                    waypoints = waypoints + custom_waypoints
                    break

                elif action == "Direction":

                    interface.turn_info(start_waypoint)
                    while sub_turn.get_turn() == None:
                        world.tick()
                        
                    if sub_turn.get_turn() == "RIGHT":
                        custom_waypoints = interface.handle_turn(start_waypoint, "RIGHT")
                        sub_turn.set_turn(None)

                    elif sub_turn.get_turn() == "LEFT":
                        custom_waypoints = interface.handle_turn(start_waypoint, "LEFT")
                        sub_turn.set_turn(None)
                    
                    elif sub_turn.get_turn() == "STRAIGHT":
                        custom_waypoints = interface.handle_turn(start_waypoint, "STRAIGHT")
                        sub_turn.set_turn(None)
                    
                    elif sub_turn.get_turn() == "FORWARD":
                        custom_waypoints = interface.handle_forward(start_waypoint)
                        sub_turn.set_turn(None)
                    
                    waypoints = waypoints + custom_waypoints
                    if custom_waypoints != []:
                        start_waypoint = custom_waypoints[len(custom_waypoints) - 1]
                    break 
                break

            if sub_done.get_done() == True:
                sub_done.set_done(False)
                break


        waypoints = pruning(map, waypoints)
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