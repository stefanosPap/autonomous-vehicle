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
#from scipy.spatial.transform import Rotation as R
def spawn():
    client = Client()                                       
    client.connect()                                        # connect the client 
    [blueprint, world, map]= client.get_simulation()
    yaw = 260
    start_point = carla.Transform(carla.Location(x=-6.5, y=-80, z=0.275307), carla.Rotation(pitch=0.000000, yaw=yaw, roll=0.000000))

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

    #while True:
        world.tick()
        world.tick()
        world.tick()
        world.tick()
        world.tick()

