import carla 

class Town(object):
    
    def __init__(self):
        self.destinations = {"Gas Station": [-33, 176, 0], "Centre": [0, 20, 0]}

    def get_destinations(self):
        return self.destinations