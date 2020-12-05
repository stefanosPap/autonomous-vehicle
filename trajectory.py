import carla 
import random 

def generate_random_trajectory(world, start_waypoint, number_of_waypoints):
    
    waypoints = [start_waypoint]
    world.debug.draw_string(start_waypoint.transform.location, 'O', draw_shadow=False, color=carla.Color(r=255, g=0, b=0), life_time=0, persistent_lines=True)
    next_w = start_waypoint.next(2.0)
    
    for i in range(number_of_waypoints):
        rand = random.choice(next_w)
        world.debug.draw_string(rand.transform.location, '{}'.format(i), draw_shadow=False, color=carla.Color(r=0, g=0, b=255), life_time=1000, persistent_lines=True)
        next_w = rand.next(10.0)
        waypoints.append(rand)
    
    return waypoints
    