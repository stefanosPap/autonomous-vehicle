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

actor_list = []

class EgoVehicle(object):

    def __init__(self):    
        self.vehicle_model = None
        self.vehicle = None
        self.start_point = None 
        self.sensors = []

    def choose_model(self, model, bp, world):
        # set vehicle
        self.blueprint_library = bp
        self.world = world  
        self.vehicle_model = self.blueprint_library.filter(model)[0]                 # get vehicle (tesla model3 from library)
        self.vehicle = self.world.spawn_actor(self.vehicle_model, self.start_point)  # spawn the vehicle to the first point    
        actor_list.append(self.vehicle)
    
    def spawn(self, map):
        # get possible points for vehicle creation 
        self.map = map
        points = self.map.get_spawn_points()                                    # returns a list of recommendations 
        self.start_point = points[0]                                            # choose the first possible point 
        
    def wander(self):
                                               # set autopilot for testing data retrieval 
        while True:
            try:
                self.vehicle.set_autopilot(True) 
                spectator = self.world.get_spectator()                              # set a spectator      
                transform = self.vehicle.get_transform()                            # get the coordinates of the car in order to attach the vehicle  
                transform.location.z += 2                                           # increase z by 2 in order to place it on top of the car ??? This maybe should change ???
                transform.location.y -= 6 
                spectator.set_transform(transform) 
                self.world.tick()                                          # infinite loop in order not to terminate the programm  

            except KeyboardInterrupt:
                break
    
    def get_vehicle_actor(self):
        return self.vehicle

    def get_vehicle_transform(self):
        return self.vehicle.get_transform()
        
    def add_sensor(self, sensor):
        self.sensors.append(sensor)
        
