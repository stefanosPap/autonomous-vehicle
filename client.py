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

class Client(object):
    def __init__(self):
        self.client = None
        self.world = None
        self.map = None
        self.blueprint_library = None

    def connect(self):  
        self.client = carla.Client('localhost', 2000)                # create a client 
        self.client.set_timeout(10.0)                                # sets in seconds the maximum time a network call is allowed before blocking it     
        self.world = self.client.get_world()                         # returns the world object currently active in the simulation 
        self.map = self.world.get_map()                              # returns the map that we are working on. The object returned is of type carla.Map 
        self.blueprint_library = self.world.get_blueprint_library()  # returns a list of actor blueprints available to ease the spawn of these into the world

    def get_simulation(self):
        return [self.blueprint_library, self.world, self.map]
