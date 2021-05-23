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

def spawn(vehicle_list):
    client = Client()                                       
    client.connect()                                        # connect the client 
    [blueprint, world, map]= client.get_simulation()
    yaw = 90
    start_point = carla.Transform(carla.Location(x=-6.5, y=-90, z=0.275307), carla.Rotation(pitch=0.000000, yaw=yaw, roll=0.000000))
    start_point = carla.Transform(carla.Location(x=30.551256, y=-197.809540, z=1), carla.Rotation(pitch=360.000, yaw=1.439560, roll=0.0))
    
    start_point = carla.Transform(carla.Location(x=60.551256, y=-195.809540, z=1),
                                  carla.Rotation(pitch=360.000, yaw=1.439560, roll=0.0))

    vehicle = Vehicle()                                  
    vehicle.choose_spawn_point(start_point)                 # spawn the vehicle 
    vehicle.choose_model('model3', blueprint, world)
    control_signal = carla.VehicleControl(throttle=0.2)
    vehicle_actor = vehicle.get_vehicle_actor()
    vehicle_actor.apply_control(control_signal)

    vehicle_list.append(vehicle_actor)

    start_point = carla.Transform(carla.Location(x=90.551256, y=-191.809540, z=1),
                                  carla.Rotation(pitch=360.000, yaw=1.439560, roll=0.0))

    #vehicle = Vehicle()                                  
    #vehicle.choose_spawn_point(start_point)                 # spawn the vehicle 
    #vehicle.choose_model('model3', blueprint, world)
    #control_signal = carla.VehicleControl(throttle=0.2)
    #vehicle_actor = vehicle.get_vehicle_actor()
    #vehicle_actor.apply_control(control_signal)
    
    #vehicle_list.append(vehicle_actor)
    
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
if __name__ == "__main__":
    try:
        spawn()
    except KeyboardInterrupt:
        pass