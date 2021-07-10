#!/usr/bin/python3 
from scipy.interpolate import interp1d
from vehicle import Vehicle
from client import Client
from vehicle_move import spawn

from utilities import plot_axis, \
    configure_sensor, \
    pruning, \
    draw_waypoints, \
    Cancel

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
# from agents.navigation.roaming_agent import RoamingAgent
# from agents.navigation.behavior_agent import BehaviorAgent

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
    #################
    # create client #
    ################# 
    client = Client()
    client.connect()  # connect the client
    [blueprint, world, map] = client.get_simulation()
    
    vehicle_list = []
    walker_list = []
    # world = client.get_client().load_world('Town02')
    # world = client.get_client().reload_world()
    # print(client.get_client().get_available_maps())
    points = map.get_spawn_points()  # returns a list of recommendations
    # start_point = carla.Transform(carla.Location(x=-95.793716, y=-3.109917, z=0.275307), carla.Rotation(pitch=0.0, yaw=-179.705399, roll=0.0))
    while True:
        start_point = random.choice(points)  # choose first point as spawn point
        start_waypoint = map.get_waypoint(start_point.location,
                                          project_to_road=False)  # return the waypoint of the spawn point
        if start_waypoint.get_junction() == None:  # spawn vehicles only on roads, not junctions
            break

    start_point = carla.Transform(carla.Location(x=40.551256, y=-197.809540, z=1),
                                  carla.Rotation(pitch=360.000, yaw=1.439560, roll=0.0))
    # start_point = carla.Transform(carla.Location(x=0, y=-73, z=0.275307), carla.Rotation(pitch=0.0, yaw=90.0, roll=0.0))
    start_point = carla.Transform(carla.Location(x=7.116180, y=-197.809540, z=1), carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0))
    
    start_waypoint = map.get_waypoint(start_point.location, project_to_road=True)
    
    ##########################
    # create new ego vehicle #
    ##########################
    vehicle = Vehicle()
    vehicle.choose_spawn_point(start_point)  # spawn the vehicle
    vehicle.choose_model('model3', blueprint, world)  # choose the model
    vehicle_actor = vehicle.get_vehicle_actor()  # return actor object of the vehicle
    vehicle_transform = vehicle.get_vehicle_transform()  # get vehicle's transform

    vehicle_list.append(vehicle_actor)

    spawn(vehicle_list, walker_list, 5, 5)
    print(1, len(vehicle_list), len(walker_list))

    trajectory = Trajectory(world, map, vehicle_actor)

    '''
    last_waypoint = map.get_waypoint(start_point.location, project_to_road=True)

    right_waypoint = last_waypoint.get_right_lane()
    waypoint_2 = last_waypoint.previous(5.0)
    waypoint_2 = waypoint_2[0]

    waypoint_1 = last_waypoint.previous(7.0)
    waypoint_1 = waypoint_1[0]

    waypoint_3 = right_waypoint.previous(3.0)
    waypoint_3 = waypoint_3[0]

    waypoint_4 = right_waypoint.next(3.0)
    waypoint_4 = waypoint_4[0]

    waypoint_5 = last_waypoint.next(7.0)
    waypoint_5 = waypoint_5[0]


    end_waypoints = [start_waypoint, waypoint_2]
    cust = trajectory.trace_route(end_waypoints)
    cust = []

    x=np.array([start_waypoint.transform.location.x, waypoint_1.transform.location.x, waypoint_2.transform.location.x,  waypoint_3.transform.location.x, right_waypoint.transform.location.x,   waypoint_4.transform.location.x, waypoint_5.transform.location.x])
    y=np.array([start_waypoint.transform.location.y, waypoint_1.transform.location.y, waypoint_2.transform.location.y,  waypoint_3.transform.location.y, right_waypoint.transform.location.y,  waypoint_4.transform.location.y,  waypoint_5.transform.location.y,])

    x_new = np.linspace(x.min(), x.max(),100)

    f = interp1d(x, y, kind='quadratic')
    y_smooth=f(x_new)
    '''

    # vehicle.set_spectator()
    client.add_actor(vehicle_actor)

    origin = carla.Transform()  # plot map's origin
    plot_axis(world, origin)

    origin = start_point  # plot vehicle's starting coordinate frame
    plot_axis(world, origin)

    # configure sensors 
    sensors = configure_sensor(vehicle_actor, vehicle_transform, blueprint, world, map, "ObstacleDetector")

    # add sensor to vehicle's configuration
    vehicle.add_sensor(sensors['obs'])

    # just wander in autopilot mode and collect data
    # vehicle.wander() 

    # world.debug.draw_string(map.get_waypoint(vehicle_actor.get_location()).transform.location, 'O', draw_shadow=False, color=carla.Color(r=255, g=200, b=0), life_time=50, persistent_lines=True)
    # agent = BasicAgent(vehicle_actor)
    # agent.set_destination([-6.446170, -50.055023, 0.275307])
    # world.debug.draw_string( carla.Location(-6.446170, -50.055023, 0.275307), 'O', draw_shadow=False, color=carla.Color(r=0, g=0, b=255), life_time=100, persistent_lines=True)

    # gp_dao = GlobalRoutePlannerDAO(map, 4)
    # gp = GlobalRoutePlanner(gp_dao)
    # top = gp.get_topology()
    # spawn()
    #waypoints = map.get_topology()
    #waypoints = map.generate_waypoints(1.0)
    #for waypoint in waypoints:
    #    for waypoint in top[i]['path']:
    #    if waypoint[0].is_junction:
    #        world.debug.draw_string(waypoint[0].transform.location, "X", draw_shadow=False, color=carla.Color(r=0, g=0, b=255), life_time=1000, persistent_lines=True)
    #    if waypoint[1].is_junction:
    #        world.debug.draw_string(waypoint[1].transform.location, "X", draw_shadow=False, color=carla.Color(r=0, g=0, b=255), life_time=1000, persistent_lines=True)
    #    else:
    #    if waypoint.lane_id == 6:
    #        world.debug.draw_string(waypoint.transform.location, "{}".format(waypoint.lane_id), draw_shadow=False, color=carla.Color(r=0, g=250, b=0), life_time=1000, persistent_lines=True)

    #        world.debug.draw_string(waypoint[0].transform.location, 's', draw_shadow=False, color=carla.Color(r=255, g=0, b=0), life_time=1000, persistent_lines=True)
    #        world.debug.draw_string(waypoint[1].transform.location, 'e', draw_shadow=False, color=carla.Color(r=255, g=0, b=0), life_time=1000, persistent_lines=True)
    # print(top)

    # pedestrian_actor = world.get_blueprint_library().filter('walker.pedestrian.0001')
    # ped_actor = world.spawn_actor(pedestrian_actor[0], spawn_point)
    # walker_controller_bp = world.get_blueprint_library().find('controller.ai.walker')
    # client.add_actor(ped_actor)

    # generate random trajectory for the vehicle 
    sub_coor     = VehicleSubscriberCoorMQTT  (topic='coordinates'       )
    sub_enter    = VehicleSubscriberEnterMQTT (topic='enter'             )  
    sub_done     = VehicleSubscriberDoneMQTT  (topic='done'              )
    sub_log      = VehicleSubscriberLogMQTT   (topic='log'               )
    sub_turn     = VehicleSubscriberTurnMQTT  (topic='turn_junction'     )

    pub          = VehiclePublisherMQTT       (topic='clean'             )
    pub_vel      = VehiclePublisherMQTT       (topic='speed_topic'       )
    pub_vel_conf = VehiclePublisherMQTT       (topic='speed_configure'   )
    pub_waypoint = VehiclePublisherMQTT       (topic='waypoint_choose'   )
    pub_enter    = VehiclePublisherMQTT       (topic='enter_info'        )
    pub_done     = VehiclePublisherMQTT       (topic='done_info'         )
    pub_start    = VehiclePublisherMQTT       (topic='start notification')
    
    pub_enter.publish({'value': 'Press ENTER for inserting new waypoint'     })
    pub_done. publish({'value': 'Press DONE for finishing waypoint selection'})

    # custom_point = carla.Transform(carla.Location(x=-125.793716, y=-4 , z=0.275307), carla.Rotation(pitch=0.0, yaw=-179.705399, roll=0.0))
    # custom_waypoint = map.get_waypoint(custom_point.location, project_to_road=False, lane_type=carla.LaneType.Any)
    # end_point = carla.Transform(carla.Location(x=-137.793716, y=-1.8, z=0.275307), carla.Rotation(pitch=0.0, yaw=-179.705399, roll=0.0))
    # end_waypoint = map.get_waypoint(end_point.location, project_to_road=False, lane_type=carla.LaneType.Any)

    # index = len(waypoints) - 1
    # location = [stop_point.location.x, stop_point.location.y, stop_point.location.z]
    # location = [waypoints[index].transform.location.x, waypoints[index].transform.location.y, waypoints[index].transform.location.z]
    # waypoints.append(start_point)
    # waypoints.append(custom_point)
    # waypoints.append(end_waypoint)
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
    cancel = Cancel()
    interface = Interface(world, map, vehicle_actor)
    
    pub.publish({'value': " "})

    
    pub_waypoint.publish({'value': ''})

    msg = {'velocity': 0}
    #pub_vel_conf.publish(msg)
    #pub_vel.publish(msg)

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
            #waypoints = cust
            waypoints = pruning(map, waypoints)
            #for i in range(len(x_new)):
            #    point = carla.Location(x_new[i], y_smooth[i], 0)
            #    trans = carla.Transform(point)
            #    waypoints.append(trans)

            waypoints = trajectory.load_trajectory(waypoints)
            draw_waypoints(world, waypoints, 100)
        except IndexError:
            continue

        # wait until START button is pushed     
        sub = VehicleSubscriberStartStopMQTT(topic='start_stop_topic')
        pub_start.publish({'value': 'Waypoint selection has completed! Press START to begin!'})

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
            behavior = Behavior(vehicle_actor, waypoints, trajectory, map, world, blueprint, vehicle_list, walker_list)
            
            behavior.follow_trajectory(world, vehicle_actor, vehicle.set_spectator, sensors['obs'].get_front_obstacle,
                                       sensors['obs'].set_front_obstacle, sensors['obs'].get_other_actor, 0)
            vel = vehicle_actor.get_velocity()
            vector = [vel.x, vel.y, vel.z]
            
            while np.linalg.norm(vector) > 0.0001:
                vel = vehicle_actor.get_velocity()
                vector = [vel.x, vel.y, vel.z]
                world.tick()

            start_waypoint = map.get_waypoint(vehicle_actor.get_location(), project_to_road=False,
                                              lane_type=carla.LaneType.Any)
            pub_waypoint.publish({'value': 'You have reached your destination! Define a new route to continue!'})
            pub_vel_conf.publish(msg)
            pub_vel.publish(msg)

        except IndexError:
            continue

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
