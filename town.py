import carla 

class Town(object):
    """
    Description:
        Class Town is used to specify the characteristics of each final location 

    """    
    def __init__(self, world):
        """
            Method __init__ is the Constructor of Class Town that initializes the coordinates of each location

        Args:
            world   (carla.World)    :  World object of CARLA API
        """
        self.destinations = {"Gas Station"          : [ -33,  176,  0], \
                             "Central Roundabout"   : [   0,   20,  0], \
                             "Circular Plaza"       : [  97,   57,  0], \
                             "Tunnel"               : [ 247,  -40,  0], \
                             "Railstation"          : [-149,  -26,  0], \
                             "Skyscraper"           : [ -43,   -3,  0], \
                             "Hotel"                : [  80,  -91,  8], \
                             "Square"               : [ 116,  -76,  8], \
                             "Highway"              : [  56,  193,  0], \
                             "Mall"                 : [ -89,  -70, -1], \
                             "Office"               : [ 151, -167,  2], \
                             "Neighborhood"         : [  56,  130,  0], \
                             "Cafeteria"            : [ -74,   99,  0], \
                             "Restaurant"           : [ -73, -170,  0]
                             }

        for key in self.destinations.keys():
            gs = carla.Location(self.destinations[key][0], self.destinations[key][1], self.destinations[key][2])
            world.debug.draw_string(gs, key, draw_shadow=False, color=carla.Color(r=0, g=0, b=0), life_time=1000, persistent_lines=True)


    def get_destinations(self):
        """
        Description:
            Method get_destinations is used to return the dictionary with the available destinations

        Returns:
            dictionary: A dictionary with keys the names of destinations and values the coordinates of the destinations
        """    

        return self.destinations