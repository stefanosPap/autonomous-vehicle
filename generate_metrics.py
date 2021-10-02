#!/usr/bin/python3 

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import sys 

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
    
    PEDESTRIANS  = []
    VEHICLES     = []
    STATIC       = []
    LIGHTS       = []
    STOPS        = []
    SPEED_LIMITS = []
    IN_ROAD      = []
    
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

            results[index][0] = round(driving_score, 3)
            results[index][1] = round(route_completion, 3)
            results[index][2] = round(infraction_penalty, 3)

            results_turns_speed[index][0] = round(right_turns_num, 3)
            results_turns_speed[index][1] = round(left_turns_num, 3)
            results_turns_speed[index][2] = round(average_speed, 3)

            results_violations[index][0] = round(pedestrians, 3)
            results_violations[index][1] = round(vehicles, 3)
            results_violations[index][2] = round(static, 3)
            results_violations[index][3] = round(lights, 3)
            results_violations[index][4] = round(stops, 3)
            results_violations[index][5] = round(speed_limits, 3)
            results_violations[index][6] = round(in_road, 3)

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

            index += 1

        if route == '\n':
            break

        first_part = route.replace(",", " ").split()[3:5]   # The first two elements contain route_completion and off_road_event_time accordingly
        second_part = route.replace(",", " ").split()[5:11] # The second part has the elements that are the defined penalties 
        third_part = route.replace(",", " ").split()[11:]   # The last elements are the right, left turns and the average speed 

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
        results[index][j] = round(sum([row[j] for row in results]) / (len(results) - 1), 3)

    for j in range(len(results_turns_speed[0]) - 1):
        results_turns_speed[index][j] = round(sum([row[j] for row in results_turns_speed]), 3) 
    results_turns_speed[index][2] = round(sum([row[2] for row in results_turns_speed]) / (len(results_turns_speed) - 1), 3)

    for j in range(len(results_violations[0]) - 1):
        results_violations[index][j] = round(sum([row[j] for row in results_violations]) , 3)
    results_violations[index][6] = round(sum([row[6] for row in results_violations]) / (len(results_violations) - 1), 3)


    data_file_1 = open('data1.txt', 'a')
    data_file_2 = open('data2.txt', 'a')
    data_file_3 = open('data3.txt', 'a')
    
    print("\n")
    print("---------------------------------------------------------------------------")
    print("  Driving Score, Route Completion and Infraction Penalty for " + obs + " obstacles")
    print("---------------------------------------------------------------------------")
    a = 0
    l = 0
    for j in range(len(results)):
        if j == len(results) - 1:
            print("\\multicolumn{2}{|c|}{\\textbf{Σύνολο}}",\
            "&",\
            results[j][0],\
            "&",\
            results[j][1],\
            "&",\
            results[j][2],\
            "\\\\")

        
        else:
            print("\\textbf" "{" + str(a) + "}" + " & \\textbf" "{" + str(l) + "}",\
            "&",\
            results[j][0],\
            "&",\
            results[j][1],\
            "&",\
            results[j][2],\
            "\\\\"
            )
        print("\\hline")
        l += 5
        if l == 15:
            a += 5
            l = 0
        data_file_1.write(str(results[j]).replace("[", "").replace("]", "") + '\n')
    data_file_1.write('\n')
    print("\n")

    print("---------------------------------------------------------------------------")
    print("     Right Turns, Left Turns and Average Speed for " + obs + " obstacles")
    print("---------------------------------------------------------------------------")
    a = 0
    l = 0
    for j in range(len(results_turns_speed)):
        if j == len(results_turns_speed) - 1:
            print("\\multicolumn{2}{|c|}{\\textbf{Σύνολο}}",\
            "&",\
            results_turns_speed[j][0],\
            "&",\
            results_turns_speed[j][1],\
            "&",\
            results_turns_speed[j][2],\
            "\\\\")

        
        else:
            print("\\textbf" "{" + str(a) + "}" + " & \\textbf" "{" + str(l) + "}",\
            "&",\
            results_turns_speed[j][0],\
            "&",\
            results_turns_speed[j][1],\
            "&",\
            results_turns_speed[j][2],\
            "\\\\"
            )
        print("\\hline")
        l += 5
        if l == 15:
            a += 5
            l = 0
        data_file_2.write(str(results_turns_speed[j]).replace("[", "").replace("]", "") + '\n')
    data_file_2.write('\n')
    print("\n")
    
    print("-------------------------------------------------------------")
    print("     Violations Number for " + obs + " obstacles")
    print("-------------------------------------------------------------")
    a = 0
    l = 0
    for j in range(len(results_violations)):
        if j == len(results_violations) - 1:
            print("\\multicolumn{2}{|c|}{\\textbf{Σύνολο}}",\
            "&",\
            results_violations[j][0],\
            "&",\
            results_violations[j][1],\
            "&",\
            results_violations[j][2],\
            "&",\
            results_violations[j][3],\
            "&",\
            results_violations[j][4],\
            "&",\
            results_violations[j][5],\
            "&",\
            results_violations[j][6],\
            "\\\\")

        
        else:
            print("\\textbf" "{" + str(a) + "}" + " & \\textbf" "{" + str(l) + "}",\
            "&",\
            results_violations[j][0],\
            "&",\
            results_violations[j][1],\
            "&",\
            results_violations[j][2],\
            "&",\
            results_violations[j][3],\
            "&",\
            results_violations[j][4],\
            "&",\
            results_violations[j][5],\
            "&",\
            results_violations[j][6],\
            "\\\\"
            )
        print("\\hline")
        
        l += 5
        if l == 15:
            a += 5
            l = 0        
        data_file_3.write(str(results_violations[j]).replace("[", "").replace("]", "") + '\n')
    data_file_3.write('\n')
    print("\n")

def generate_diagramms():
    data_file1 = open('data1.txt', 'r')
    data_file2 = open('data2.txt', 'r')
    
    labels = [0, 10, 30, 60, 90]
    lines1 = data_file1.readlines()
    lines2 = data_file2.readlines()

    array_driving_score = [[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0]]
    array_route_completion = [[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0]]
    array_violations = [[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0]]

    array_right_turns = [[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0]]
    array_left_turns = [[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0]]
    array_average_speed = [[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0]]
    
    for i in range(10):
        for j in range(5):
            line = lines1[i + j * 11]
            line = line.replace(",", " ").split()
            line = list(map(float , line))          # convert to list with floats 
            array_driving_score[i][j] = line[0]
            array_route_completion[i][j] = line[1]
            array_violations[i][j] = line[2]

        
            line = lines2[i + j * 11]
            line = line.replace(",", " ").split()
            line = list(map(float , line))          # convert to list with floats 
            array_right_turns[i][j] = line[0]
            array_left_turns[i][j] = line[1]
            array_average_speed[i][j] = line[2]


    arrays = [array_driving_score, array_route_completion, array_violations, array_right_turns, array_left_turns, array_average_speed]
             
    width = 0.08
    plot = 1

    r1  = np.arange(len(labels))
    r2  = [x + width for x in r1]
    r3  = [x + width for x in r2]
    r4  = [x + width for x in r3]
    r5  = [x + width for x in r4]
    r6  = [x + width for x in r5]
    r7  = [x + width for x in r6]
    r8  = [x + width for x in r7]
    r9  = [x + width for x in r8]
    r10 = [x + width for x in r9]
    plt.style.use('bmh')

    for array in arrays:
        _, ax = plt.subplots(figsize =(16, 8))
        
        ax.bar(r1,  array[0], width=width, edgecolor='white')
        ax.bar(r2,  array[1], width=width, edgecolor='white')
        ax.bar(r3,  array[2], width=width, edgecolor='white')
        ax.bar(r4,  array[3], width=width, edgecolor='white')
        ax.bar(r5,  array[4], width=width, edgecolor='white')
        ax.bar(r6,  array[5], width=width, edgecolor='white')
        ax.bar(r7,  array[6], width=width, edgecolor='white')
        ax.bar(r8,  array[7], width=width, edgecolor='white')
        ax.bar(r9,  array[8], width=width, edgecolor='white')
        ax.bar(r10, array[9], width=width, edgecolor='white')
        
        ax.set_ylabel('Scores')
        ax.set_xlabel('Number of dynamic obstacles')
        ax.set_xticks([r + 4 * width for r in range(len(labels))])
        ax.set_xticklabels(labels)
        bars = ['(0,0)', '(0,5)', '(0,10)', '(5,0)', '(5,5)', '(5,10)', '(10,0)', '(10,5)', '(10,10)', 'Total']
        ax.legend(bars, title="(agg, law)", fancybox=True, bbox_to_anchor=(1.1, 1.05))
    
        if plot == 1:
            ax.set_title('Driving Score Metric')
            plt.savefig("driving_score.png") 
        elif plot == 2:
            ax.set_title('Route Completion Metric')
            plt.ylim([0.6, 1])
            plt.savefig("route_completion.png")
        elif plot == 3:    
            ax.set_title('Infraction Penalty Metric')
            plt.savefig("infraction_penalty.png") 
        elif plot == 4:
            ax.set_title('Right Turns Metric')
            plt.savefig("right_turns.png")
        elif plot == 5:
            ax.set_title('Left Turns Metric')
            plt.savefig("left_turns.png")
        elif plot == 6:
            ax.set_title('Average Speed Metric')
            plt.ylim([10, 40])
            plt.savefig("average_speed.png")

        plot += 1

def generate_trajectory_diagramm():

    data_file = open('data_coordinates.txt', 'r')
    lines = data_file.readlines()
    
    x_w = lines[0]
    y_w = lines[1]
    x_l = lines[2]
    y_l = lines[3]

    x_w = x_w.replace(",", " ").replace("]", " ").replace("[", " ").split()
    x_w = list(map(float , x_w))

    y_w = y_w.replace(",", " ").replace("]", " ").replace("[", " ").split()
    y_w = list(map(float , y_w))

    x_l = x_l.replace(",", " ").replace("]", " ").replace("[", " ").split()
    x_l = list(map(float , x_l))

    y_l = y_l.replace(",", " ").replace("]", " ").replace("[", " ").split()
    y_l = list(map(float , y_l))

    plt.plot(x_w, y_w, color="red", linewidth=4)
    plt.plot(x_l, y_l, color="blue", linewidth=2)
    
    plt.xlabel("$x$", fontsize=16)
    plt.ylabel("$y$", fontsize=16)

    line1 = Line2D([0], [0], label='Specified Trajectory', color='red')
    line2 = Line2D([0], [0], label='Vehicle\'s Trajectory', color='blue')
    
    handles = [line1, line2]
    plt.legend(handles=handles)
    
    resolution_value = 1500
    plt.savefig("plot.png", dpi=resolution_value)

if __name__ == '__main__':

    open('data1.txt', 'w').close()
    open('data2.txt', 'w').close()
    open('data3.txt', 'w').close()
    
    try:
        
        if sys.argv[1].isdigit():
            if int(sys.argv[1]) in [0, 10, 30, 60, 90]:
                obs = sys.argv[1]
                metrics('data_' + obs + '.txt', obs)
            else:
                print("Give me one of 0, 10, 30, 60, 90")

        if sys.argv[1] == "all":
            obs = str(0)
            metrics('data_' + obs + '.txt', obs)
            
            obs = str(10)
            metrics('data_' + obs + '.txt', obs)
            
            obs = str(30)
            metrics('data_' + obs + '.txt', obs)
            
            obs = str(60)
            metrics('data_' + obs + '.txt', obs)
            
            obs = str(90)
            metrics('data_' + obs + '.txt', obs)
            
            generate_diagramms()

        if sys.argv[1] == "traj":
            generate_trajectory_diagramm()
    
    except (ValueError, IndexError):
        print("Give me Integer value or 'all' or 'traj' attribute")
    
    
    