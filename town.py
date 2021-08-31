import carla 

class Town(object):
    
    def __init__(self, world):
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

        gs = carla.Location(self.destinations["Gas Station"][0], self.destinations["Gas Station"][1], self.destinations["Gas Station"][2])
        world.debug.draw_string(gs, "Gas Station", draw_shadow=False, color=carla.Color(r=0, g=0, b=0), life_time=1000, persistent_lines=True)

        gs = carla.Location(self.destinations["Central Roundabout"][0], self.destinations["Central Roundabout"][1], self.destinations["Central Roundabout"][2])
        world.debug.draw_string(gs, "Central Roundabout", draw_shadow=False, color=carla.Color(r=0, g=0, b=0), life_time=1000, persistent_lines=True)

        gs = carla.Location(self.destinations["Circular Plaza"][0], self.destinations["Circular Plaza"][1], self.destinations["Circular Plaza"][2])
        world.debug.draw_string(gs, "Circular Plaza", draw_shadow=False, color=carla.Color(r=0, g=0, b=0), life_time=1000, persistent_lines=True)

        gs = carla.Location(self.destinations["Tunnel"][0], self.destinations["Tunnel"][1], self.destinations["Tunnel"][2])
        world.debug.draw_string(gs, "Tunnel", draw_shadow=False, color=carla.Color(r=0, g=0, b=0), life_time=1000, persistent_lines=True)

        gs = carla.Location(self.destinations["Railstation"][0], self.destinations["Railstation"][1], self.destinations["Railstation"][2])
        world.debug.draw_string(gs, "Railstation", draw_shadow=False, color=carla.Color(r=0, g=0, b=0), life_time=1000, persistent_lines=True)

        gs = carla.Location(self.destinations["Skyscraper"][0], self.destinations["Skyscraper"][1], self.destinations["Skyscraper"][2])
        world.debug.draw_string(gs, "Skyscraper", draw_shadow=False, color=carla.Color(r=0, g=0, b=0), life_time=1000, persistent_lines=True)

        gs = carla.Location(self.destinations["Hotel"][0], self.destinations["Hotel"][1], self.destinations["Hotel"][2])
        world.debug.draw_string(gs, "Hotel", draw_shadow=False, color=carla.Color(r=0, g=0, b=0), life_time=1000, persistent_lines=True)

        gs = carla.Location(self.destinations["Square"][0], self.destinations["Square"][1], self.destinations["Square"][2])
        world.debug.draw_string(gs, "Square", draw_shadow=False, color=carla.Color(r=0, g=0, b=0), life_time=1000, persistent_lines=True)

        gs = carla.Location(self.destinations["Highway"][0], self.destinations["Highway"][1], self.destinations["Highway"][2])
        world.debug.draw_string(gs, "Highway", draw_shadow=False, color=carla.Color(r=0, g=0, b=0), life_time=1000, persistent_lines=True)

        gs = carla.Location(self.destinations["Mall"][0], self.destinations["Mall"][1], self.destinations["Mall"][2])
        world.debug.draw_string(gs, "Mall", draw_shadow=False, color=carla.Color(r=0, g=0, b=0), life_time=1000, persistent_lines=True)

        gs = carla.Location(self.destinations["Office"][0], self.destinations["Office"][1], self.destinations["Office"][2])
        world.debug.draw_string(gs, "Office", draw_shadow=False, color=carla.Color(r=0, g=0, b=0), life_time=1000, persistent_lines=True)

        gs = carla.Location(self.destinations["Neighborhood"][0], self.destinations["Neighborhood"][1], self.destinations["Neighborhood"][2])
        world.debug.draw_string(gs, "Neighborhood", draw_shadow=False, color=carla.Color(r=0, g=0, b=0), life_time=1000, persistent_lines=True)

        gs = carla.Location(self.destinations["Cafeteria"][0], self.destinations["Cafeteria"][1], self.destinations["Cafeteria"][2])
        #world.debug.draw_string(gs, "Cafeteria", draw_shadow=False, color=carla.Color(r=0, g=0, b=0), life_time=1000, persistent_lines=True)

        gs = carla.Location(self.destinations["Restaurant"][0], self.destinations["Restaurant"][1], self.destinations["Restaurant"][2])
        #world.debug.draw_string(gs, "Restaurant", draw_shadow=False, color=carla.Color(r=0, g=0, b=0), life_time=1000, persistent_lines=True)

    def get_destinations(self):
        return self.destinations