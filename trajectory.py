import carla 
import random 

def generate_random_trajectory(world, start_waypoint, map, number_of_waypoints):
    
    waypoints = [start_waypoint]
    world.debug.draw_string(start_waypoint.transform.location, 'O', draw_shadow=False, color=carla.Color(r=255, g=0, b=0), life_time=0, persistent_lines=True)
    next_w = start_waypoint.next(5.0)
    
    for i in range(1, number_of_waypoints):
        rand = random.choice(next_w)
        if i > 5 and i < 12:
            # random custom waypoints 
            location = carla.Location(rand.transform.location.x + 2, rand.transform.location.y, rand.transform.location.z)
            w = map.get_waypoint(location)
            world.debug.draw_string(w.transform.location, '{}'.format(i), draw_shadow=False, color=carla.Color(r=0, g=255, b=0), life_time=1000, persistent_lines=True)
            waypoints.append(w)
        else:    
            world.debug.draw_string(rand.transform.location, '{}'.format(i), draw_shadow=False, color=carla.Color(r=0, g=0, b=255), life_time=1000, persistent_lines=True)
            waypoints.append(rand)
        next_w = rand.next(5.0)
    
    return waypoints
    