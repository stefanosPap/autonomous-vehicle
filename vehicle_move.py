##################################################
# Script for generating other vehicles' movement #
##################################################
  
import carla 
import math 
import numpy as np
from utilities import draw_vehicle_box
from vehicle import Vehicle
from client import Client
from utilities import plot_axis, rotate, scale, expanded_bounding, rectangle_bounding
from communicationMQTT import VehiclePublisherMQTT
import random

def spawn(vehicle_list, walkers_list, number_of_vehicles, number_of_walkers):
    client = Client()                                       
    client.connect()                                        # connect the client 
    client_actor = client.get_client()
    [blueprint, world, map]= client.get_simulation()

    

    
    '''
    yaw = 90
    start_point = carla.Transform(carla.Location(x=-6.5, y=-90, z=0.275307), carla.Rotation(pitch=0.000000, yaw=yaw, roll=0.000000))
    start_point = carla.Transform(carla.Location(x=30.551256, y=-197.809540, z=1), carla.Rotation(pitch=360.000, yaw=1.439560, roll=0.0))
    
    start_point = carla.Transform(carla.Location(x=40.551256, y=-195.809540, z=1),
                                  carla.Rotation(pitch=360.000, yaw=1.439560, roll=0.0))

    vehicle = Vehicle()                                  
    vehicle.choose_spawn_point(start_point)                 # spawn the vehicle 
    vehicle.choose_model('model3', blueprint, world)
    control_signal = carla.VehicleControl(throttle=0.2)
    vehicle_actor = vehicle.get_vehicle_actor()
    vehicle_actor.apply_control(control_signal)
    #vehicle_actor.set_autopilot(True)
    vehicle_list.append(vehicle_actor)

    start_point = carla.Transform(carla.Location(x=90.551256, y=-191.809540, z=1),
                                  carla.Rotation(pitch=360.000, yaw=1.439560, roll=0.0))

    start_point = carla.Transform(carla.Location(x=80.551256, y=-195, z=1), carla.Rotation(pitch=360.000, yaw=1.439560, roll=0.0))

    
    vehicle = Vehicle()                                  
    vehicle.choose_spawn_point(start_point)                 # spawn the vehicle 
    vehicle.choose_model('model3', blueprint, world)
    control_signal = carla.VehicleControl(throttle=0.2)
    vehicle_actor = vehicle.get_vehicle_actor()
    vehicle_actor.apply_control(control_signal)
    #vehicle_actor.set_autopilot(True)
    vehicle_list.append(vehicle_actor)
    
    start_point = carla.Transform(carla.Location(x=54.551256, y=-191.809540, z=1),
                                  carla.Rotation(pitch=360.000, yaw=1.439560, roll=0.0))
    
    #vehicle = Vehicle()
    #vehicle.choose_spawn_point(start_point)  # spawn the vehicle
    #vehicle.choose_model('model3', blueprint, world)  # choose the model
    #vehicle_actor = vehicle.get_vehicle_actor()
    #control_signal = carla.VehicleControl(throttle=0.19)
    #vehicle_actor.apply_control(control_signal)
    # pub = VehiclePublisherMQTT(topic='position')
    '''

    '''
    for k in range(1,10,10):
        spawn_point = carla.Transform()
        spawn_point.location = start_point.location + carla.Location(k,38,0)
        spawn_point.rotation = start_point.rotation 
        
        vehicle = Vehicle()                                  
        vehicle.choose_spawn_point(spawn_point)                 # spawn the vehicle 
        vehicle.choose_model('model3', blueprint, world)
        vehicle_actor = vehicle.get_vehicle_actor()
        for i in range(20):
            world.tick()
        #vehicle_actor.set_autopilot(True) 
        bb = draw_vehicle_box(world, vehicle_actor, spawn_point.location, spawn_point.rotation, 100)
        plot_axis(world, carla.Transform(bb.location, bb.rotation))
        world.debug.draw_string(bb.location, 'C', draw_shadow=False, color=carla.Color(r=0, g=0, b=255), life_time=100, persistent_lines=True)
        
        pub.publish({'position': [bb.location.x, bb.location.y, bb.location.z]})
        
        locationA = bb.location + carla.Location(bb.extent.x, bb.extent.y, 0)
        pointA = rotate(bb=bb, degrees=yaw, location=locationA)

        locationB = bb.location + carla.Location(bb.extent.x, -bb.extent.y, 0)
        pointB = rotate(bb=bb, degrees=yaw, location=locationB)

        locationC = bb.location - carla.Location(bb.extent.x, bb.extent.y, 0)
        pointC = rotate(bb=bb, degrees=yaw, location=locationC)

        locationD = bb.location + carla.Location(-bb.extent.x, bb.extent.y, 0)
        pointD = rotate(bb=bb, degrees=yaw, location=locationD)

        world.debug.draw_string(pointA, 'A', draw_shadow=False, color=carla.Color(r=0, g=0, b=255), life_time=100, persistent_lines=True)
        world.debug.draw_string(pointB, 'B', draw_shadow=False, color=carla.Color(r=0, g=0, b=255), life_time=100, persistent_lines=True)
        world.debug.draw_string(pointC, 'C', draw_shadow=False, color=carla.Color(r=0, g=0, b=255), life_time=100, persistent_lines=True)
        world.debug.draw_string(pointD, 'D', draw_shadow=False, color=carla.Color(r=0, g=0, b=255), life_time=100, persistent_lines=True)
        
        expanded_bounding(world, bb, pointA, pointB, pointC, pointD)
        #bounding(world, [pointA.x, pointB.x, pointC.x, pointD.x], [pointA.y, pointB.y, pointC.y, pointD.y])
    '''
    #while True:
    world.tick()
    world.tick()
    world.tick()
    world.tick()
    world.tick()
    #return vehicle_list
    if True:
        '''
        port = 8000
        while True:
            world.tick()
            world.tick()
            try:
                traffic_manager = client_actor.get_trafficmanager(port)
                break
            except RuntimeError:
                print(port)
                port += 1
                if port == 8010:
                    break
        traffic_manager.set_global_distance_to_leading_vehicle(1.0)


        traffic_manager.set_synchronous_mode(True)
        '''
        blueprints = world.get_blueprint_library().filter('vehicle.*')
        blueprintsWalkers = world.get_blueprint_library().filter('walker.pedestrian.*')

        blueprints = [x for x in blueprints if int(x.get_attribute('number_of_wheels')) == 4]
        blueprints = [x for x in blueprints if not x.id.endswith('isetta')]
        blueprints = [x for x in blueprints if not x.id.endswith('carlacola')]
        blueprints = [x for x in blueprints if not x.id.endswith('cybertruck')]
        blueprints = [x for x in blueprints if not x.id.endswith('t2')]

        blueprints = sorted(blueprints, key=lambda bp: bp.id)

        spawn_points = world.get_map().get_spawn_points()
        number_of_spawn_points = len(spawn_points)

        if number_of_vehicles < number_of_spawn_points:
            random.shuffle(spawn_points)
        elif number_of_vehicles > number_of_spawn_points:
            number_of_vehicles = number_of_spawn_points

        all_id = []
        
        
        # --------------
        # Spawn vehicles
        # --------------
        
        for n, transform in enumerate(spawn_points):
            if n >= number_of_vehicles:
                break
            blueprint = random.choice(blueprints)
            if blueprint.has_attribute('color'):
                color = random.choice(blueprint.get_attribute('color').recommended_values)
                blueprint.set_attribute('color', color)
            if blueprint.has_attribute('driver_id'):
                driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
                blueprint.set_attribute('driver_id', driver_id)
            blueprint.set_attribute('role_name', 'autopilot')
           
            try: 
                vehicle_actor = world.spawn_actor(blueprint, transform)
            except:
                continue
            world.tick()
            world.tick()
            world.tick()
            world.tick()

            # spawn the cars and set their autopilot and light state all together
            vehicle_list.append(vehicle_actor)

        # -------------
        # Spawn Walkers
        # -------------
        # some settings
        percentagePedestriansRunning = 0.0      # how many pedestrians will run
        percentagePedestriansCrossing = 0.0     # how many pedestrians will walk through the road
        # 1. take all the random locations to spawn
        spawn_points = []
        for i in range(number_of_walkers):
            spawn_point = carla.Transform()
            loc = world.get_random_location_from_navigation()
            if (loc != None):
                spawn_point.location = loc
                spawn_points.append(spawn_point)

        # 2. we spawn the walker object
        batch = []
        walker_speed = []
        for spawn_point in spawn_points:
            walker_bp = random.choice(blueprintsWalkers)
            # set as not invincible
            if walker_bp.has_attribute('is_invincible'):
                walker_bp.set_attribute('is_invincible', 'false')
            # set the max speed
            if walker_bp.has_attribute('speed'):
                if (random.random() > percentagePedestriansRunning):
                    # walking
                    walker_speed.append(walker_bp.get_attribute('speed').recommended_values[1])
                else:
                    # running
                    walker_speed.append(walker_bp.get_attribute('speed').recommended_values[2])
            else:
                print("Walker has no speed")
                walker_speed.append(0.0)
            
            try:
                walker_actor = world.spawn_actor(walker_bp, spawn_point)
            except:
                while True:
                    spawn_point = carla.Transform()
                    loc = world.get_random_location_from_navigation()
                    if (loc != None):
                        spawn_point.location = loc
                    try:
                        walker_actor = world.spawn_actor(walker_bp, spawn_point)
                        break
                    except:
                        continue

            walkers_list.append(walker_actor)
        
        #results = client_actor.apply_batch_sync(batch, True)
        #walker_speed2 = []
        #for i in range(len(results)):
        #    walkers_list.append({"id": results[i].actor_id})
        #    walker_speed2.append(walker_speed[i])
        
        #walker_speed = walker_speed2
        # 3. we spawn the walker controller
        walker_ai_list = []
        walker_controller_bp = world.get_blueprint_library().find('controller.ai.walker')
        for i in range(len(walkers_list)):
            walker_actor_ai = world.spawn_actor(walker_controller_bp, carla.Transform(), walkers_list[i])
            walker_ai_list.append(walker_actor_ai)

        #results = client_actor.apply_batch_sync(batch, True)
        #for i in range(len(results)):
        #    walkers_list[i]["con"] = results[i].actor_id
        
        # 4. we put altogether the walkers and controllers id to get the objects from their id
        #for i in range(len(walkers_list)):
        #    all_id.append(walkers_list[i]["con"])
        #    all_id.append(walkers_list[i]["id"])
        #all_actors = world.get_actors(all_id)
        
        world.tick()
        world.tick()
        world.tick()
        world.tick()
        world.tick()

        # 5. initialize each controller and set target to walk to (list is [controler, actor, controller, actor ...])
        # set how many pedestrians can cross the road
        world.set_pedestrians_cross_factor(percentagePedestriansCrossing)
        for i in range(len(walkers_list)):
            # start walker
            walker_ai_list[i].start()
            # set walk to random point
            walker_ai_list[i].go_to_location(world.get_random_location_from_navigation())
            # max speed
            walker_ai_list[i].set_max_speed(float(walker_speed[int(i/2)]))
        for vehicle in vehicle_list[1:len(vehicle_list) - 2]:
            vehicle.set_autopilot(True)
            world.tick()
            world.tick()
            world.tick()
            world.tick()
        
        print('spawned %d vehicles and %d walkers, press Ctrl+C to exit.' % (len(vehicle_list), len(walkers_list)))

        # example of how to use parameters
        #traffic_manager.global_percentage_speed_difference(30.0)
    
