#!/usr/bin/python3 

import numpy as np

def metrics(data_file):
    
    data_file = open(data_file, 'r')
    routes = data_file.readlines()[1:]
    penalties_dict = {  'pedestrian_collision': 0.50, \
                        'vehicle_collision': 0.60, \
                        'static_obstacle_collision': 0.65, \
                        'red_light_violation': 0.7, \
                        'stop_sign_violation': 0.8, \
                        'speed_limit_violation': 0.9}
    Ri = []
    Pi = []
    N  = 10
    for route in routes:
        
        first_part = route.replace(",", " ").split()[3:5]   # The first two elements contain route_completion and off_road_event_time accordingly
        second_part = route.replace(",", " ").split()[5:]   # The last elements are the defined penalties 

        route_info = list(map(float , first_part))          # convert to list with floats 
        violations = list(map(float , second_part))         # convert to list with floats

        violations_dict = dict(zip(penalties_dict.keys(), violations))  # create violations dictionary match violations with penalties dictionary 
        
        # calculate the metric Pi 
        off_road_event_time = route_info[1]                             
        mult_dict = {k: penalties_dict[k] ** violations_dict[k] for k in penalties_dict} 
        pi = list(mult_dict.values())
        product = np.prod(pi) * off_road_event_time

        # save the value of each route in two arrays 
        Pi.append(product)
        Ri.append(route_info[0])
    
    # calculate the metrics
    route_completion = sum(Pi) / N
    infraction_penalty = sum(Ri) / N
    driving_score = sum((np.array(Pi) * np.array(Ri))) / N
    
if __name__ == '__main__':
    metrics('data.txt')