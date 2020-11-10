#!/usr/bin/python3 
import glob
import os
import signal 
import sys
import time
import random 
import math  

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla  


class EgoVehicle():

    def __init__(self):
        self.client = None
        self.start_point = None
        self.world = None
        self.map = None
        self.blueprint_library = None    
        self.vehicle_model = None
        self.vehicle = None
        self.start_point = None 

    def connect(self):  
        self.client = carla.Client('localhost', 2000)                # create a client 
        self.client.set_timeout(10.0)                                # sets in seconds the maximum time a network call is allowed before blocking it 

    def get_simulation(self):    
        self.world = self.client.get_world()                         # returns the world object currently active in the simulation 
        self.map = self.world.get_map()                              # returns the map that we are working on. The object returned is of type carla.Map 
        self.blueprint_library = self.world.get_blueprint_library()  # returns a list of actor blueprints available to ease the spawn of these into the world

    def choose_model(self,model):
        # set vehicle 
        self.vehicle_model = self.blueprint_library.filter(model)[0]                 # get vehicle (tesla model3 from library)
        self.vehicle = self.world.spawn_actor(self.vehicle_model, self.start_point)  # spawn the vehicle to the first point   
        self.vehicle.set_autopilot(True)                                             # set autopilot for testing data retrieval 

    def spawn(self):
        # get possible points for vehicle creation 
        points = self.map.get_spawn_points()                                    # returns a list of recommendations 
        self.start_point = points[0]                                            # choose the first possible point 
    
    def wander(self):
        while True:    
            spectator = self.world.get_spectator()                              # set a spectator      
            transform = self.vehicle.get_transform()                            # get the coordinates of the car in order to attach the vehicle  
            transform.location.z += 2                                           # increase z by 2 in order to place it on top of the car ??? This maybe should change ???
            spectator.set_transform(transform)
            #world_snapshot = 
            self.world.wait_for_tick()                                          # infinite loop in order not to terminate the programm 

def main():
    actor_list = []
    try:
        vehicle = EgoVehicle()
        vehicle.connect()
        vehicle.get_simulation()
        vehicle.spawn()
        vehicle.choose_model('model3')
        vehicle.wander()
        actor_list.append(vehicle)
    finally:
        print('destroying actors')
        for actor in actor_list:
            actor.destroy()
        print('done.')

if __name__ == "__main__":
    main()
