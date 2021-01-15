##################################################
# Script for generating other vehicles' movement #
##################################################
  
import carla 
import math 
import numpy as np
from utilities import draw_vehicle_box
from vehicle import Vehicle
from client import Client
from utilities import plot_axis, rotate, scale  
#from scipy.spatial.transform import Rotation as R
def spawn():
    client = Client()                                       
    client.connect()                                        # connect the client 
    [blueprint, world, map]= client.get_simulation()
    yaw = 30
    start_point = carla.Transform(carla.Location(x=-10, y=-20, z=0.275307), carla.Rotation(pitch=0.000000, yaw=yaw, roll=0.000000))

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

        locationB = bb.location - carla.Location(bb.extent.x, bb.extent.y, 0)
        pointB = rotate(bb=bb, degrees=yaw, location=locationB)

        locationC = bb.location + carla.Location(bb.extent.x, -bb.extent.y, 0)
        pointC = rotate(bb=bb, degrees=yaw, location=locationC)

        locationD = bb.location + carla.Location(-bb.extent.x, bb.extent.y, 0)
        pointD = rotate(bb=bb, degrees=yaw, location=locationD)

        world.debug.draw_string(pointA, 'A', draw_shadow=False, color=carla.Color(r=0, g=0, b=255), life_time=100, persistent_lines=True)
        world.debug.draw_string(pointB, 'B', draw_shadow=False, color=carla.Color(r=0, g=0, b=255), life_time=100, persistent_lines=True)
        world.debug.draw_string(pointC, 'C', draw_shadow=False, color=carla.Color(r=0, g=0, b=255), life_time=100, persistent_lines=True)
        world.debug.draw_string(pointD, 'D', draw_shadow=False, color=carla.Color(r=0, g=0, b=255), life_time=100, persistent_lines=True)

        x_values = [pointA.x, pointB.x, pointC.x, pointD.x]
        y_values = [pointA.y, pointB.y, pointC.y, pointD.y]
        
        minValueX = min(x_values)
        minIndexX = x_values.index(minValueX)
        
        minValueY = min(y_values)
        minIndexY = y_values.index(minValueY)

        maxValueX = max(x_values)
        maxIndexX = x_values.index(maxValueX)

        maxValueY = max(y_values)
        maxIndexY = y_values.index(maxValueY)

        x = minValueX
        while x < maxValueX:
            point = carla.Location(x,minValueY,0)
            point = scale(bb=bb, valueX=1, valueY=1.1, location=point)
            point = rotate(bb=bb, degrees=yaw, location=point)
            world.debug.draw_string(point, 'X', draw_shadow=False, color=carla.Color(r=0, g=0, b=255), life_time=100, persistent_lines=True)
            x = x + 0.5
            if x > maxValueX:
                point = carla.Location(maxValueX,minValueY,0)
                point = scale(bb=bb, valueX=1, valueY=1.1, location=point)
                point = rotate(bb=bb, degrees=yaw, location=point)
                world.debug.draw_string(point, 'X', draw_shadow=False, color=carla.Color(r=0, g=0, b=255), life_time=100, persistent_lines=True)

        x = minValueX
        while x < maxValueX:
            point = carla.Location(x,maxValueY,0)
            point = scale(bb=bb, valueX=1.4, valueY=1.3, location=point)
            point = rotate(bb=bb, degrees=yaw, location=point)
            world.debug.draw_string(point, 'X', draw_shadow=False, color=carla.Color(r=0, g=255, b=0), life_time=100, persistent_lines=True)
            x = x + 0.5
            if x > maxValueX:
                point = carla.Location(maxValueX,maxValueY,0)
                point = scale(bb=bb, valueX=1.4, valueY=1.3, location=point)
                point = rotate(bb=bb, degrees=yaw, location=point)
                world.debug.draw_string(point, 'X', draw_shadow=False, color=carla.Color(r=0, g=255, b=0), life_time=100, persistent_lines=True)

        y = minValueY
        while y < maxValueY:
            point = carla.Location(minValueX,y,0)
            point = scale(bb=bb, valueX=1.1, valueY=1, location=point)
            point = rotate(bb=bb, degrees=yaw, location=point)
            world.debug.draw_string(point, 'Y', draw_shadow=False, color=carla.Color(r=255, g=0, b=0), life_time=100, persistent_lines=True)
            y = y + 0.5
            if y > maxValueY:
                point = carla.Location(minValueX,maxValueY,0)
                point = scale(bb=bb, valueX=1.1, valueY=1, location=point)
                point = rotate(bb=bb, degrees=yaw, location=point)
                world.debug.draw_string(point, 'Y', draw_shadow=False, color=carla.Color(r=255, g=0, b=0), life_time=100, persistent_lines=True)

        y = minValueY
        while y < maxValueY:
            point = carla.Location(maxValueX,y,0)
            point = scale(bb=bb, valueX=1.1, valueY=1, location=point)
            point = rotate(bb=bb, degrees=yaw, location=point)
            world.debug.draw_string(point, 'Y', draw_shadow=False, color=carla.Color(r=255, g=0, b=255), life_time=100, persistent_lines=True)
            y = y + 0.5
            if y > maxValueY:
                point = carla.Location(maxValueX,maxValueY,0)
                point = scale(bb=bb, valueX=1.1, valueY=1, location=point)
                point = rotate(bb=bb, degrees=yaw, location=point)
                world.debug.draw_string(point, 'Y', draw_shadow=False, color=carla.Color(r=255, g=0, b=255), life_time=100, persistent_lines=True)

    #while True:
        world.tick()
        world.tick()
        world.tick()
        world.tick()
        world.tick()

