import carla 
import random 

class Trajectory():
    def __init__(self, world, map):
        self.world = world
        self.map = map
        self.waypoints = []
        self.change = False

    def load_trajectory(self, waypoints):
        self.waypoints = waypoints
        return self.waypoints

    def generate_random_trajectory(self, start_waypoint, number_of_waypoints):
        
        self.waypoints.append(start_waypoint)
        self.world.debug.draw_string(start_waypoint.transform.location, 'O', draw_shadow=False, color=carla.Color(r=255, g=0, b=0), life_time=0, persistent_lines=True)
        next_w = start_waypoint.next(3.0)
    
        for i in range(1, number_of_waypoints):
            rand = random.choice(next_w)
            #if i > 5 and i < 10:
            #    left = rand.get_left_lane()
                #right = waypoints[i].get_right_lane()
            #    if left != None:
            #        location = carla.Location(left.transform.location.x, left.transform.location.y, left.transform.location.z)
            #        w = self.map.get_waypoint(location)
            #        self.world.debug.draw_string(w.transform.location, '{}'.format(i), draw_shadow=False, color=carla.Color(r=0, g=255, b=0), life_time=1000, persistent_lines=True)
            #        self.waypoints.append(w)                
                #    self.world.debug.draw_string(left.transform.location, '{}'.format(0), draw_shadow=False, color=carla.Color(r=0, g=0, b=255), life_time=1000, persistent_lines=True)
                #if right != None:
                #    self.world.debug.draw_string(right.transform.location, '{}'.format(1), draw_shadow=False, color=carla.Color(r=255, g=0, b=0), life_time=1000, persistent_lines=True)
            
                # random custom waypoints 
                #location = carla.Location(rand.transform.location.x + 2, rand.transform.location.y, rand.transform.location.z)

            #else:    
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
                location = carla.Location(next_waypoint.transform.location.x, next_waypoint.transform.location.y, next_waypoint.transform.location.z)
                w = self.map.get_waypoint(location, project_to_road=False)
                if w != None:
                    self.waypoints[waypoint] = w
                    self.change = True 
                    self.world.debug.draw_string(w.transform.location, '{}'.format(waypoint), draw_shadow=False, color=carla.Color(r=0, g=255, b=0), life_time=1000, persistent_lines=True)
                    return w
            else:
                w = prev_waypoint
            return w