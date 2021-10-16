#!/usr/bin/python3 
from scipy.interpolate import interp1d
from vehicle import Vehicle
from client import Client
from vehicle_move import spawn
from interface import Interface
from experiments import Experiment 
import sys 
from town import Town

from utilities import   plot_axis, \
                        configure_sensor, \
                        pruning, \
                        draw_waypoints, \
                        Cancel

from trajectory import Trajectory
from behavior import Behavior
from communicationMQTT import   VehicleSubscriberStartStopMQTT, \
                                VehicleSubscriberCoorMQTT, \
                                VehicleSubscriberEnterMQTT, \
                                VehicleSubscriberDoneMQTT, \
                                VehicleSubscriberLogMQTT, \
                                VehiclePublisherMQTT, \
                                VehicleSubscriberTurnMQTT

import numpy as np
import carla
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
    """
    Description:
        Function main is the central function from which the whole process is starting
    """    

    #
    # Create client  
    #
    client = Client()
    client.connect()  # connect the client
    [blueprint, world, map] = client.get_simulation()
    
    vehicle_list = []
    walker_list  = []

    #
    # Specify Town in CARLA #
    #
    # world = client.get_client().load_world('Town02')
    # world = client.get_client().reload_world()
    # print(client.get_client().get_available_maps())
    
    #
    # case of spawning the ego vehicle at a random place     
    #
    points = map.get_spawn_points()                                                     # returns a list of recommendations
    while True:
        start_point = random.choice(points)                                             # choose first point as spawn point
        start_waypoint = map.get_waypoint(start_point.location, project_to_road=False)  # return the waypoint of the spawn point
        if start_waypoint.get_junction() == None:                                       # spawn vehicles only on roads, not junctions
            break

    #
    # case of spawning the ego vehicle at a specific place     
    #
    start_point = carla.Transform(carla.Location(0, 20,  1), carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0))
    start_waypoint = map.get_waypoint(start_point.location, project_to_road=True)
    
    #
    # create new ego vehicle 
    #
    vehicle = Vehicle()
    vehicle.choose_spawn_point(start_point)              # spawn the vehicle
    vehicle.choose_model('model3', blueprint, world)     # choose the model
    vehicle_actor = vehicle.get_vehicle_actor()          # return actor object of the vehicle
    vehicle_transform = vehicle.get_vehicle_transform()  # get vehicle's transform

    vehicle_list.append(vehicle_actor)                   # add the vehicle to vehicle list 

    #
    # define experiment parameters
    #
    if len(sys.argv) > 2:
        experiment_execution = int(sys.argv[1])
    else:
        experiment_execution = 0
    
    # experiment mode
    if 10 > experiment_execution > 0: 
        exp = Experiment()
        [aggressive, lawful, vehicles, pedestrians] = exp.execute_scenario(scenario=int(sys.argv[1]), dynamic_obstacles=int(sys.argv[2]))
        experiment = {'experiment': int(sys.argv[1]), 'aggresssive': aggressive, 'lawful': lawful, 'vehicles': vehicles, 'pedestrians': pedestrians}

        print("-------------------------------------------------------------------------------"                     )
        print("                Running an experiment with the following values                "                     )
        print("                                aggressive: {}                                 ".format(aggressive)  )
        print("                                lawful: {}                                     ".format(lawful)      )
        print("                                vehicles: {}                                   ".format(vehicles)    )
        print("                                pedestrians: {}                                ".format(pedestrians) )
        print("-------------------------------------------------------------------------------"                     )
    
    # normal mode 
    else:
        exp = Experiment()
        [aggressive, lawful, vehicles, pedestrians] = [0, 0, 0, 0]
        experiment = {'experiment': 0, 'aggresssive': aggressive, 'lawful': lawful, 'vehicles': vehicles, 'pedestrians': pedestrians}

        print("-------- Running on normal mode with zero initial values for the sliders --------\n")

    #
    # spawn the specified number of dynamic obstacles 
    #
    spawn(vehicle_list, walker_list, vehicles, pedestrians)

    #
    # plot vehicle's starting coordinate frame 
    #
    origin = start_point 
    plot_axis(world, origin)
    
    #
    # configure sensors 
    #
    sensors = configure_sensor(vehicle_actor, vehicle_transform, blueprint, world, map, "ObstacleDetector")

    #
    # add sensor to vehicle's configuration
    #
    vehicle.add_sensor(sensors['obs'])
    
    #
    # just wander in autopilot mode and collect data
    #
    # vehicle.wander() 

    #
    # initialize some publishers and subscribers  
    #
    sub_done     = VehicleSubscriberDoneMQTT  (topic='done'              )
    sub_log      = VehicleSubscriberLogMQTT   (topic='log'               )
    sub_turn     = VehicleSubscriberTurnMQTT  (topic='turn_junction'     )

    pub          = VehiclePublisherMQTT       (topic='clean'             )
    pub_waypoint = VehiclePublisherMQTT       (topic='waypoint_choose'   )
    pub_enter    = VehiclePublisherMQTT       (topic='enter_info'        )
    pub_done     = VehiclePublisherMQTT       (topic='done_info'         )
    pub_start    = VehiclePublisherMQTT       (topic='start notification')
    
    pub_enter.   publish({'value': 'Press ENTER for inserting new waypoint'     })
    pub_done.    publish({'value': 'Press DONE for finishing waypoint selection'})
    pub.         publish({'value': ""                                           })
    pub_waypoint.publish({'value': ''                                           })

    cancel = Cancel()

    trajectory = Trajectory(world, map, vehicle_actor)
    interface  = Interface(world, map, vehicle_actor)

    # start a loop where the user gives the choices about the directions and the locations that wants to follow 
    while True:

        waypoints = []
        custom_waypoints = []
        current_waypoint = start_waypoint

        while True:
            while True:

                # check if cancel button be pushed 
                if cancel.cancel_process():
                    cancel.cancel_now(False)

                    # initialize all the variables again in order to start again
                    waypoints = []
                    custom_waypoints = []
                    current_waypoint = start_waypoint

                    break

                # check if new destination goal has logged 
                world.tick()
                action = " "
                if sub_log.get_log() != " ":
                    action = sub_log.get_log()
                    sub_log.set_log(" ")

                # case of location chosen 
                if action == "Location":
                    end_waypoints = interface.handle(current_waypoint)

                    try:
                        custom_waypoints = trajectory.trace_route(end_waypoints)
                        #for j in range(len(custom_waypoints) - 2):
                        #    world.debug.draw_line(custom_waypoints[j].transform.location, custom_waypoints[j + 1].transform.location, thickness=1, color=carla.Color(r=200, g=0, b=0), life_time=1000, persistent_lines=True)        
         
                        current_waypoint = custom_waypoints[len(custom_waypoints) - 1]
                        waypoints = waypoints + custom_waypoints
                    except IndexError:
                        waypoints = []
                        custom_waypoints = []
                        current_waypoint = start_waypoint

                    break

                # case of direction chosen 
                elif action == "Direction":

                    interface.turn_info(current_waypoint)
                    while sub_turn.get_turn() is None:
                        world.tick()
                        if cancel.cancel_process():
                            cancel.cancel_now(False)
                            custom_waypoints = []
                            waypoints = []
                            current_waypoint = start_waypoint
                            break

                    # handle each direction
                    if sub_turn.get_turn() == "RIGHT":
                        custom_waypoints = interface.handle_turn(current_waypoint, "RIGHT")
                        sub_turn.set_turn(None)

                    elif sub_turn.get_turn() == "LEFT":
                        custom_waypoints = interface.handle_turn(current_waypoint, "LEFT")
                        sub_turn.set_turn(None)

                    elif sub_turn.get_turn() == "STRAIGHT":
                        custom_waypoints = interface.handle_turn(current_waypoint, "STRAIGHT")
                        sub_turn.set_turn(None)

                    elif sub_turn.get_turn() == "FORWARD":
                        custom_waypoints = interface.handle_forward(current_waypoint)
                        sub_turn.set_turn(None)

                    waypoints = waypoints + custom_waypoints
                    if custom_waypoints:
                        current_waypoint = custom_waypoints[len(custom_waypoints) - 1]
                    break
                break

            # check if DONE button has been pushed  
            if sub_done.get_done():
                sub_done.set_done(False)
                break

        try:
            # perform pruning in the waypoints array in order to throw outliers or double waypoints
            waypoints = pruning(map, waypoints)
            waypoints = trajectory.load_trajectory(waypoints)
            draw_waypoints(world, waypoints, 100)
        except IndexError:
            continue

        # wait until START button is pushed     
        sub = VehicleSubscriberStartStopMQTT(topic='start_stop_topic')
        pub_start.publish({'value': 'Path selection has been completed! Press START to activate the vehicle!'})

        while True:

            world.tick()
            if cancel.cancel_process():
                cancel.cancel_now(False)

                # initialize all the variables again in order to start again
                waypoints = []
                custom_waypoints = []
                current_waypoint = start_waypoint
                break

            start = sub.get_start()
            if start:
                sub.set_start(False)
                pub_start.publish({'value': 'Your car is on! Choose velocity to begin!'})
                break

        # follow trajectory and stop to obstacles and traffic lights
        try:
            behavior = Behavior(vehicle_actor, waypoints, trajectory, map, world, blueprint, vehicle_list, walker_list, experiment, exp)
            behavior.follow_trajectory(world, vehicle_actor, vehicle.set_spectator, 0)
            
            vel = vehicle_actor.get_velocity()
            vector = [vel.x, vel.y, vel.z]
            
            # wait until vehicle stops 
            while np.linalg.norm(vector) > 0.0001:
                vel = vehicle_actor.get_velocity()
                vector = [vel.x, vel.y, vel.z]
                world.tick()

            start_waypoint = map.get_waypoint(vehicle_actor.get_location(), project_to_road=False, lane_type=carla.LaneType.Any)
            pub_waypoint.publish({'value': 'You have reached your destination! Define a new route to continue!'})

        except IndexError:
            continue

    #
    # Destroy actors and exit 
    #
    for actor in client.get_created_actors():
        actor.destroy()
    print('Done!')
    world.tick()
    sys.exit()


if __name__ == "__main__":
    main()
