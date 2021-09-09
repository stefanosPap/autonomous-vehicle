#!/usr/bin/python3 

import numpy as np

def metrics(data_file, obs):
    
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

    RT = []
    LT = []
    SP = []
    
    PEDESTRIANS = []
    VEHICLES = []
    STATIC = []
    LIGHTS = []
    STOPS = []
    SPEED_LIMITS = []
    IN_ROAD = []
    
    N  = 2
    k = 1
    index = 0
    results = [ [0,0,0],\
                [0,0,0],\
                [0,0,0],\
                [0,0,0],\
                [0,0,0],\
                [0,0,0],\
                [0,0,0],\
                [0,0,0],\
                [0,0,0],\
                [0,0,0]
                ]
    results_turns_speed = [ [0,0,0],\
                            [0,0,0],\
                            [0,0,0],\
                            [0,0,0],\
                            [0,0,0],\
                            [0,0,0],\
                            [0,0,0],\
                            [0,0,0],\
                            [0,0,0],\
                            [0,0,0]]

    results_violations = [  [0,0,0,0,0,0,0],\
                            [0,0,0,0,0,0,0],\
                            [0,0,0,0,0,0,0],\
                            [0,0,0,0,0,0,0],\
                            [0,0,0,0,0,0,0],\
                            [0,0,0,0,0,0,0],\
                            [0,0,0,0,0,0,0],\
                            [0,0,0,0,0,0,0],\
                            [0,0,0,0,0,0,0],\
                            [0,0,0,0,0,0,0]]


    for route in routes:
        if k == 3:
            # calculate the metrics
            route_completion = sum(Ri) / N
            infraction_penalty = sum(Pi) / N
            driving_score = sum((np.array(Pi) * np.array(Ri))) / N
            
            right_turns_num = sum(RT)
            left_turns_num = sum(LT)
            average_speed = sum(SP) / N 


            pedestrians = sum(PEDESTRIANS)
            vehicles = sum(VEHICLES)
            static = sum(STATIC)
            lights = sum(LIGHTS)
            stops = sum(STOPS)
            speed_limits = sum(SPEED_LIMITS)
            in_road = sum(IN_ROAD) / N 

            results[index][0] = driving_score
            results[index][1] = route_completion
            results[index][2] = infraction_penalty

            results_turns_speed[index][0] = right_turns_num
            results_turns_speed[index][1] = left_turns_num
            results_turns_speed[index][2] = average_speed

            results_violations[index][0] = pedestrians
            results_violations[index][1] = vehicles
            results_violations[index][2] = static
            results_violations[index][3] = lights
            results_violations[index][4] = stops
            results_violations[index][5] = speed_limits
            results_violations[index][6] = in_road

            Ri = []
            Pi = []
            RT = []
            LT = []
            SP = []
            PEDESTRIANS = []
            VEHICLES = []
            STATIC = []
            LIGHTS = []
            STOPS = []
            SPEED_LIMITS = []
            IN_ROAD = []
            N  = 2
            k  = 1
            #print(results[index])
            #print(results_turns_speed[index])

            index += 1

        if route == '\n':
            break

        first_part = route.replace(",", " ").split()[3:5]   # The first two elements contain route_completion and off_road_event_time accordingly
        second_part = route.replace(",", " ").split()[5:11] # The second part has the elements that are the defined penalties 
        third_part = route.replace(",", " ").split()[11:]  # The last elements are the right, left turns and the average speed 

        route_info = list(map(float , first_part))          # convert to list with floats 
        violations = list(map(float , second_part))         # convert to list with floats
        turns_speed = list(map(float , third_part))         # convert to list with floats

        violations_dict = dict(zip(penalties_dict.keys(), violations))  # create violations dictionary match violations with penalties dictionary 

        # calculate the metric Pi 
        off_road_event_time = route_info[1]                             
        mult_dict = {p: penalties_dict[p] ** violations_dict[p] for p in penalties_dict} 
        pi = list(mult_dict.values())
        product = np.prod(pi) * off_road_event_time

        # save the value of each route in two arrays 
        Pi.append(product)
        Ri.append(route_info[0])
        
        RT.append(turns_speed[0])
        LT.append(turns_speed[1])
        SP.append(turns_speed[2])

        PEDESTRIANS.append(violations[0])
        VEHICLES.append(violations[1])
        STATIC.append(violations[2])
        LIGHTS.append(violations[3])
        STOPS.append(violations[4])
        SPEED_LIMITS.append(violations[5])
        IN_ROAD.append(off_road_event_time)

        k += 1
    
    for j in range(len(results[0])):
        results[index][j] = sum([row[j] for row in results]) / (len(results) - 1)

    for j in range(len(results_turns_speed[0]) - 1):
        results_turns_speed[index][j] = sum([row[j] for row in results_turns_speed]) 
    results_turns_speed[index][2] = sum([row[2] for row in results_turns_speed]) / (len(results_turns_speed) - 1)

    for j in range(len(results_violations[0]) - 1):
        results_violations[index][j] = sum([row[j] for row in results_violations]) 
    results_violations[index][6] = sum([row[6] for row in results_violations]) / (len(results_violations) - 1)


    data_file_1 = open('data1.txt', 'a')
    data_file_2 = open('data2.txt', 'a')
    data_file_3 = open('data3.txt', 'a')

    print("\n")
    print("---------------------------------------------------------------------------")
    print("  Driving Score, Route Completion and Infraction Penalty for " + obs + " obstacles")
    print("---------------------------------------------------------------------------")
    for j in range(len(results)):
        print(results[j])
        data_file_1.write(str(results[j]).replace("[", "").replace("]", "") + '\n')
    data_file_1.write('\n')
    print("\n")

    print("---------------------------------------------------------------------------")
    print("     Right Turns, Left Turns and Average Speed for " + obs + " obstacles")
    print("---------------------------------------------------------------------------")
    for j in range(len(results_turns_speed)):
        print(results_turns_speed[j])
        data_file_2.write(str(results_turns_speed[j]).replace("[", "").replace("]", "") + '\n')
    data_file_2.write('\n')
    print("\n")

    print("-------------------------------------------------------------")
    print("     Violations Number for " + obs + " obstacles")
    print("-------------------------------------------------------------")
    for j in range(len(results_violations)):
        print(results_violations[j])
        data_file_3.write(str(results_violations[j]).replace("[", "").replace("]", "") + '\n')
    data_file_3.write('\n')
    print("\n")

def generate_diagramms(data_file):
    pass
    data_file = open(data_file, 'r')
    
if __name__ == '__main__':
    obs = str(10)
    metrics('data_' + obs + '.txt', obs)