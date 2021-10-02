import carla 
import random 
from agents.navigation.basic_agent import BasicAgent 

class Trajectory():
    def __init__(self, world, map, vehicle_actor):
        self.world = world
        self.map = map
        self.waypoints = []
        self.change = False
        self.basic_agent = BasicAgent(vehicle_actor)

    def load_trajectory(self, waypoints):
        self.waypoints = waypoints
        return self.waypoints

    def generate_random_trajectory(self, start_waypoint, number_of_waypoints):
        
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

        waypoints = []
        for k in range(len(end_waypoints) - 1): 
            route = self.basic_agent._trace_route(end_waypoints[k], end_waypoints[k+1])
            for waypoint in route:
                waypoints.append(waypoint[0]) 
        return waypoints