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
    """
    Description:
        Method spawn is used to create a specific number of dynamic obstacles in the map

    Args:
        vehicle_list        (list)    :    List with the overall vehicles in the map 
        walker_list         (list)    :    List with the overall walkers in the map 
        number_of_vehicles  (int)     :    The number of vehicles that will be spawned 
        number_of_walkers   (int)     :    The number of walkers that will be spawned 
    """    
    
    client = Client()                                       
    client.connect()                                        # connect the client 
    [blueprint, world, _]= client.get_simulation()

    world.tick()
    world.tick()
    world.tick()
    world.tick()
    world.tick()
    
    if True:
     
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
        
        # 3. we spawn the walker controller
        walker_ai_list = []
        walker_controller_bp = world.get_blueprint_library().find('controller.ai.walker')
        for i in range(len(walkers_list)):
            walker_actor_ai = world.spawn_actor(walker_controller_bp, carla.Transform(), walkers_list[i])
            walker_ai_list.append(walker_actor_ai)
        
        world.tick()
        world.tick()
        world.tick()
        world.tick()
        world.tick()

        # 4. initialize each controller and set target to walk to (list is [controler, actor, controller, actor ...])
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