import carla 
import random 
from agents.navigation.basic_agent import BasicAgent 

class Trajectory():
    """
    Description:
        Class Trajectory is used to create the trajectory that the vehicle will pass, by creating an array containing all the waypoints which the vehicle will cross
    """    

    def __init__(self, world, map, vehicle_actor):
        """
        Description:
            Method __init__ is the Constructor of Class Trajectory that initializes most of the used variables 

        Args:
            world           (carla.World)           :    World object of CARLA API
            map             (carla.Map)             :    Map object of CARLA API
            world           (carla.World)           :    World object of CARLA API
        """        

        self.world = world
        self.map = map
        self.waypoints = []
        self.change = False
        self.basic_agent = BasicAgent(vehicle_actor)


    def load_trajectory(self, waypoints):
        """
        Description:
            Method load_trajectory is used to load a previously created array of waypoints   

        Args:
            waypoints (list): A list with the waypoints that will be loaded, in order to be followed by the ego vehicle 

        Returns:
            list: The list with the waypoints that will be followed by the ego vehicle 
        """        

        self.waypoints = waypoints
        return self.waypoints


    def generate_random_trajectory(self, start_waypoint, number_of_waypoints):
        """
        Description:
            Method generate_random_trajectory is creating a random array of waypoints that could be followed by the vehicle 

        Args:
            start_waypoint      (carla.Waypoint)    :   The waypoint from which the path will be started 
            number_of_waypoints (int)               :   The number of waypoints that the random trajectory will contain 

        Returns:
            list:   The list of random waypoints   
        """     

        self.waypoints.append(start_waypoint)
        self.world.debug.draw_string(start_waypoint.transform.location, 'O', draw_shadow=False, color=carla.Color(r=255, g=0, b=0), life_time=0, persistent_lines=True)
        next_w = start_waypoint.next(3.0)
    
        for i in range(1, number_of_waypoints):
            rand = random.choice(next_w)    
            self.world.debug.draw_string(rand.transform.location, '{}'.format(i), draw_shadow=False, color=carla.Color(r=0, g=0, b=255), life_time=1000, persistent_lines=True)
            self.waypoints.append(rand)
            next_w = rand.next(3.0)
    
        return self.waypoints


    def change_waypoint(self, waypoint, direction):
        """
        Description:
            Method change_waypoint is used to take the right or the left waypoint of a specific waypoint 

        Args:
            waypoint    (carla.Waypoint)    :   The waypoint that will be changed  
            direction   (str)               :   The direction to which the new waypoint will be located relatively to the old one  

        Returns:
            carla.Waypoint:     The new calculated waypoint  
        """        

        prev_waypoint = self.waypoints[waypoint]
        if prev_waypoint != None:
            if direction == "LEFT":
                next_waypoint = prev_waypoint.get_left_lane()
            elif direction == "RIGHT":
                next_waypoint = prev_waypoint.get_right_lane()

            if next_waypoint != None:
                w = next_waypoint
                if w != None:
                    self.waypoints[waypoint] = w
                    self.change = True 
                    self.world.debug.draw_string(w.transform.location, '{}'.format(waypoint), draw_shadow=False, color=carla.Color(r=0, g=255, b=0), life_time=1000, persistent_lines=True)
                    return w
            else:
                w = prev_waypoint
            return w


    def trace_route(self, end_waypoints):
        """
        Description:
            Method trace_route is used to create the shortest path using A* algorithm between the locations of end_waypoints array 

        Args:
            end_waypoints (list): The locations among which the shorthest path will be created  

        Returns:
            list: The list with the waypoints of the overall shortest path 
        """        
        
        waypoints = []
        for k in range(len(end_waypoints) - 1): 
            route = self.basic_agent._trace_route(end_waypoints[k], end_waypoints[k+1])
            for waypoint in route:
                waypoints.append(waypoint[0]) 
        return waypoints