class Experiment(object):
    """
    Description:
        Class Experiment implements the speific experiments that conducted, in order to test the performance of the autonomous vehicle 
    """    

    def __init__(self):
        """
        Description:
            Method __init__ is the void Constructor of Class Experiment 
        """   
        
        pass 


    def execute_scenario(self, scenario, dynamic_obstacles): 
        """
        Description:
            Method execute_scenario is used to initialize the variables of the chosen experiment scenario 

        Args:
            scenario (int): The scenario number 
            dynamic_obstacles (int): The number of dynamic obstacles that the will be spawned in the map 

        Returns:
            tuple: A tuple with the value of aggressivene and lawful sliders and the number of vehicles and pedestrians 
        """        

        vehicles    = dynamic_obstacles
        pedestrians = dynamic_obstacles
        self.dynamic_obstacles = dynamic_obstacles

        if scenario == 1:
            self.aggressive  = 0
            self.lawful      = 0

        elif scenario == 2:
            self.aggressive  = 0
            self.lawful      = 5

        elif scenario == 3:
            self.aggressive  = 0
            self.lawful      = 10

        elif scenario == 4:
            self.aggressive  = 5
            self.lawful      = 0

        elif scenario == 5:
            self.aggressive  = 5
            self.lawful      = 5
        
        elif scenario == 6:
            self.aggressive  = 5
            self.lawful      = 10

        elif scenario == 7:
            self.aggressive  = 10
            self.lawful      = 0

        elif scenario == 8:
            self.aggressive  = 10
            self.lawful      = 5

        elif scenario == 9:
            self.aggressive  = 10
            self.lawful      = 10
        else:
            return

        return self.aggressive, self.lawful, vehicles, pedestrians
    

    def save_experiment_data(self, data):
        """
        Description:
            Method save_experiment_data is used to log the values of the metrics in a text file 

        Args:
            data (dictionary): A dictionary that contains the value of each metric 
        """            
        
        self.file = open('data_' + str(self.dynamic_obstacles) + '.txt', 'a')
        stop_sign_violations      = data['stop_sign_violations']
        red_light_violations      = data['red_light_violations']
        speed_limit_violations    = data['speed_limit_violations']
        pedestrian_collision      = data['pedestrian_collision']
        vehicle_collision         = data['vehicle_collision']
        static_obstacle_collision = data['static_obstacle_collision']
        off_road_event_time       = data['off_road_event_time']
        route_completion          = data['route_completion']
        right_turns               = data['right_turns']
        left_turns                = data['left_turns']
        average_speed             = data['average_speed']
        
        self.file.write("{}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}\n".format(  self.aggressive,\
                                                                                            self.lawful,\
                                                                                            self.dynamic_obstacles,\
                                                                                            route_completion,\
                                                                                            off_road_event_time,\
                                                                                            pedestrian_collision,\
                                                                                            vehicle_collision,\
                                                                                            static_obstacle_collision,\
                                                                                            red_light_violations,\
                                                                                            stop_sign_violations,\
                                                                                            speed_limit_violations,\
                                                                                            right_turns,\
                                                                                            left_turns,\
                                                                                            average_speed))
        self.file.close()
