#!/usr/bin/python3 
import glob
import os 
import sys

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla  

class Vehicle(object):
    
    def __init__(self):    
        self.vehicle_model = None
        self.vehicle_actor = None
        self.start_point = None 
        self.sensors = []

    def choose_model(self, model, bp, world):
        # set vehicle
        self.blueprint_library = bp
        self.world = world  
        self.vehicle_model = self.blueprint_library.filter(model)[0]                        # get vehicle (tesla model3 from library)
        self.vehicle_actor = self.world.spawn_actor(self.vehicle_model, self.start_point)   # spawn the vehicle to the first point    
    
    def choose_spawn_point(self, point):
        # set the spawn point 
        self.start_point = point

    def wander(self):
        while True:
            try:
                self.vehicle_actor.set_autopilot(True) 
                #self.set_spectator()
                self.world.tick()                                          # infinite loop in order not to terminate the programm  

            except KeyboardInterrupt:
                break
                
    def get_vehicle_actor(self):
        return self.vehicle_actor

    def get_vehicle_transform(self):
        return self.vehicle_actor.get_transform()
        
    def add_sensor(self, sensor):
        self.sensors.append(sensor)
        
    def set_spectator(self): 
        spectator = self.world.get_spectator()                              # set a spectator      
        transform = self.get_vehicle_transform()                            # get the coordinates of the car in order to attach the vehicle  
        transform.location.z += 50                                           # increase z by 2 in order to place it on top of the car ??? This maybe should change ???
        transform.rotation.roll = 0         
        transform.rotation.pitch = -90
        transform.rotation.yaw = 180
        spectator.set_transform(transform)