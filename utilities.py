import carla
import numpy as np  
from sensor import Sensor, Lidar, CameraRGB, GNSS, IMU, ObstacleDetector, LaneInvasionDetector, Radar, CameraSemantic
from communicationMQTT import VehiclePublisherMQTT, VehicleSubscriberCancelMQTT


def calculate_angle(ways):
    """
    Description:
        Function calculate_angle is used for caclulating angle between the first and the last point of a path
    
    Args:
        ways (list): A trajectory with the waypoints from which the first and the last element will be used for calculating the angle  

    Returns:
        float: The calculated angle 
    """    
    
    vec1 = ways[0].transform.get_forward_vector()
    vec1 = [vec1.x, vec1.y]
    
    vec2 = ways[len(ways) - 1].transform.get_forward_vector()
    vec2 = [vec2.x, vec2.y]
    
    unit_vector_1 = vec1 / np.linalg.norm(vec1)
    unit_vector_2 = vec2 / np.linalg.norm(vec2)
    
    dot_product = np.dot(unit_vector_1, unit_vector_2)
    angle = np.arctan(dot_product)

    return angle


def change_coordinate_system(start_point, point):
    """
    Description:
        Function change_coordinate_system is used for changing a point's coordinate system 

    Args:
        start_point (carla.Transform)   :   The point that will be the new origin 
        point       (carla.Transform)   :   The point whose the coordinates will be changed 

    Returns:
        carla.Transform:    The calculated point in the new coordinate system  
    """ 

    degrees = start_point.rotation.yaw
    theta = np.radians(degrees)
    c, s = np.cos(theta), np.sin(theta)
    R = np.array(((c, s , 0), (-s, c, 0), (0, 0, 1)))
    point = point - start_point.location

    point =  np.matmul(R, np.array([point.x, point.y, point.z]))

    point = carla.Location(point[0], point[1], point[2])

    return point
    

def rotate(bb, degrees, location):
    """
    Description:
        Function rotate is used for rotating a point through bounding box's center 

    Args:
        bb          (carla.Blueprint)      :   Blueprint object of CARLA API
        degrees     (float)                :   The angle in degrees by which the point will be rotated 
        location    (carla.Location)       :   The point's location that will be rotated

    Returns:
        carla.Location: The rotated point
    """

    theta = np.radians(degrees)
    c, s = np.cos(theta), np.sin(theta)
    R = np.array(((c, -s , 0), (s, c, 0), (0, 0, 1)))

    point = location 
    point = point - carla.Location(bb.location.x, bb.location.y, bb.location.z)

    point =  np.matmul(R, np.array([point.x, point.y, point.z]))

    point = carla.Location(point[0], point[1], point[2])
    point = point + carla.Location(bb.location.x, bb.location.y, bb.location.z)

    return point 


def scale(bb, valueX, valueY, location):
    """
    Description:
        Function scale is used for scaling a point through bounding box's center

    Args:
        bb          (carla.Blueprint)       :   Blueprint object of CARLA API
        valueX      (float)                 :   The value of scale in X-axis
        valueY      (float)                 :   The value of scale in Y-axis
        location    (carla.Location)        :   The point's location that will be scaled

    Returns:
        carla.Location: The scaled point
    """    

    point = location 
    point = point - carla.Location(bb.location.x, bb.location.y, bb.location.z)

    point = carla.Location(point.x * valueX, point.y * valueY, point.z)
    point = point + carla.Location(bb.location.x, bb.location.y, bb.location.z)

    return point


def expanded_bounding(world, bb, pointA, pointB, pointC, pointD, rate=2):
    """
    Description:
        Function expanded_bounding  is used for finding expanded bounding box

    Args:
        world       (carla.World)           :   World object of CARLA API
        bb          (carla.Blueprint)       :   Blueprint object of CARLA API
        pointA      (carla.Location)        :   The first edge's point
        pointB      (carla.Location)        :   The second edge's point
        pointC      (carla.Location)        :   The third edge's point
        pointD      (carla.Location)        :   The fourth edge's point
        rate        (int, optional)         :   The rate the box will be expanded. Defaults to 2.
    """    

    a = [pointA.x, pointA.y]
    b = [pointB.x, pointB.y]
    points = np.linspace(a,b, num=10)
    for i in range(len(points)):
        point = carla.Location(points[i][0],points[i][1],0)
        point = scale(bb=bb, valueX=rate, valueY=rate, location=point)
        world.debug.draw_string(point, 'X', draw_shadow=False, color=carla.Color(r=0, g=0, b=255), life_time=100, persistent_lines=True)
        
    a = [pointA.x, pointA.y]
    d = [pointD.x, pointD.y]
    points = np.linspace(a,d, num=10)
    for i in range(len(points)):
        point = carla.Location(points[i][0],points[i][1],0)
        point = scale(bb=bb, valueX=rate, valueY=rate, location=point)
        world.debug.draw_string(point, 'X', draw_shadow=False, color=carla.Color(r=255, g=0, b=0), life_time=100, persistent_lines=True)
        
    c = [pointC.x, pointC.y]
    d = [pointD.x, pointD.y]
    points = np.linspace(c,d, num=10)
    for i in range(len(points)):
        point = carla.Location(points[i][0],points[i][1],0)
        point = scale(bb=bb, valueX=rate, valueY=rate, location=point)
        world.debug.draw_string(point, 'X', draw_shadow=False, color=carla.Color(r=0, g=255, b=0), life_time=100, persistent_lines=True)
        
    c = [pointC.x, pointC.y]
    b = [pointB.x, pointB.y]
    points = np.linspace(c,b, num=10)
    for i in range(len(points)):
        point = carla.Location(points[i][0],points[i][1],0)
        point = scale(bb=bb, valueX=rate, valueY=rate, location=point)
        world.debug.draw_string(point, 'X', draw_shadow=False, color=carla.Color(r=255, g=255, b=0), life_time=100, persistent_lines=True)


def rectangle_bounding(world, x_values, y_values):
    """
    Description:
        Function rectangle_bounding is used for finding rectangle bounding box

    Args:
        world       (carla.World)           :   World object of CARLA API
        x_values    (list)                  :   The x-coordinates of the bounding box's  
        y_values    (list)                  :   The y-coordinates of the bounding box's 
    """    

    minValueX = min(x_values)
    minIndexX = x_values.index(minValueX)
    
    minValueY = min(y_values)
    minIndexY = y_values.index(minValueY)

    maxValueX = max(x_values)
    maxIndexX = x_values.index(maxValueX)

    maxValueY = max(y_values)
    maxIndexY = y_values.index(maxValueY)

    x = minValueX
    while x < maxValueX:
        point = carla.Location(x,minValueY,0)
        world.debug.draw_string(point, 'O', draw_shadow=False, color=carla.Color(r=0, g=0, b=255), life_time=100, persistent_lines=True)
        x = x + 0.5
        if x > maxValueX:
            point = carla.Location(maxValueX,minValueY,0)
            world.debug.draw_string(point, 'O', draw_shadow=False, color=carla.Color(r=0, g=0, b=255), life_time=100, persistent_lines=True)

    x = minValueX
    while x < maxValueX:
        point = carla.Location(x,maxValueY,0)
        world.debug.draw_string(point, 'O', draw_shadow=False, color=carla.Color(r=0, g=255, b=0), life_time=100, persistent_lines=True)
        x = x + 0.5
        if x > maxValueX:
            point = carla.Location(maxValueX,maxValueY,0)
            world.debug.draw_string(point, 'O', draw_shadow=False, color=carla.Color(r=0, g=255, b=0), life_time=100, persistent_lines=True)

    y = minValueY
    while y < maxValueY:
        point = carla.Location(minValueX,y,0)
        world.debug.draw_string(point, 'O', draw_shadow=False, color=carla.Color(r=255, g=0, b=0), life_time=100, persistent_lines=True)
        y = y + 0.5
        if y > maxValueY:
            point = carla.Location(minValueX,maxValueY,0)
            world.debug.draw_string(point, 'O', draw_shadow=False, color=carla.Color(r=255, g=0, b=0), life_time=100, persistent_lines=True)

    y = minValueY
    while y < maxValueY:
        point = carla.Location(maxValueX,y,0)
        world.debug.draw_string(point, 'O', draw_shadow=False, color=carla.Color(r=255, g=0, b=255), life_time=100, persistent_lines=True)
        y = y + 0.5
        if y > maxValueY:
            point = carla.Location(maxValueX,maxValueY,0)
            world.debug.draw_string(point, 'O', draw_shadow=False, color=carla.Color(r=255, g=0, b=255), life_time=100, persistent_lines=True)
        

def plot_axis(world, origin):
    """
    Description:
        Function plot_axis is used for ploting axis

    Args:
        world           (carla.World)           :   World object of CARLA API
        origin          (carla.Location)        :   The origin if the axis that will be plotted 
    """    

    length = 20

    world.debug.draw_line(origin.location,  origin.location + carla.Location(length, 0, 0), thickness=0.1, color=carla.Color(255,0,0), life_time=1000)
    world.debug.draw_line(origin.location,  origin.location + carla.Location(0, length, 0), thickness=0.1, color=carla.Color(0,255,0), life_time=1000)
    world.debug.draw_line(origin.location,  origin.location + carla.Location(0, 0, length), thickness=0.1, color=carla.Color(0,0,255), life_time=1000)

    world.debug.draw_string(origin.location + carla.Location(length + 1, 0, 0), 'X', draw_shadow=False, color=carla.Color(r=255, g=0, b=0), life_time=1000, persistent_lines=True)
    world.debug.draw_string(origin.location + carla.Location(0, length + 1, 0), 'Y', draw_shadow=False, color=carla.Color(r=0, g=255, b=0), life_time=1000, persistent_lines=True)
    world.debug.draw_string(origin.location + carla.Location(0, 0, length + 1), 'Z', draw_shadow=False, color=carla.Color(r=0, g=0, b=255), life_time=1000, persistent_lines=True)


def draw_vehicle_box(world, actor, location, rotation, life_time):
    """
    Description:
        Function draw_vehicle_box is used for drawing bounding boxes to vehicles

    Args:
        world           (carla.World)           :   World object of CARLA API
        actor           (carla.Actor)           :   The actor whose bounding box will be designed  
        location        (carla.Location)        :   The actor's center location
        rotation        (carla.Rotation)        :   The actor's rotation
        life_time       (float)                 :   The time that the box will remain visible  

    Returns:
        carla.BoundingBox: The designed bounding box 
    """

    bb = actor.bounding_box
    bbox = carla.BoundingBox(location, bb.extent)
    world.debug.draw_box(bbox, rotation, 0.1, carla.Color(255,0,0), life_time)

    return bbox


def configure_sensor(vehicle_actor, vehicle_transform, blueprint, world, map, *args):
    """
    Description:
        Function configure_sensor is used for configuring the vehicle's sensors 

    Args:
        vehicle_actor       (carla.Vehicle)         :    The actor object of the autonomous vehicle that the sensors are attached to 
        vehicle_transform   (carla.Transform)       :    The vehicle's transform
        blueprint           (carla.Blueprint)       :    Blueprint object of CARLA API
        world               (carla.World)           :    World object of CARLA API
        map                 (carla.Map)             :    Map object of CARLA API

    Returns:
        dictionary: A dictionary with all the sensors that were attached to the vehicle
    """    
    sensors = {}
    for sensor in args:
        if sensor == "Lidar":
            lidar = Lidar()                                             # create a lidar sensor 
            lidar.set_vehicle(vehicle_actor)                            # give the vehicle that the sensor will be attached to  
            lidar.set_rotation(0,0,0)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           
            lidar.set_location(0,0,2)
            lidar.set_simulation(blueprint, world, map)
            lidar.set_sensor()
            lidar.read()
            sensors['lidar'] = lidar

        elif sensor == "Radar":
            radar = Radar()                                             # create a sensor 
            radar.set_vehicle(vehicle_actor)                            # give the vehicle that the sensor will be attached to  
            radar.set_rotation(pitch=5,yaw=0,roll=0)
            radar.set_location(X=2,Y=0,Z=1)
            radar.set_simulation(blueprint, world, map)
            radar.set_sensor(horizontal_fov=35, vertical_fov=20)
            radar.read()
            sensors['radar'] = radar            
        
        elif sensor == "LaneInvasion":
            lane = LaneInvasionDetector()                               # create a Lane Invasion detector sensor 
            lane.set_vehicle(vehicle_actor)                             # give the vehicle that the sensor will be attached to  
            lane.set_rotation(0,0,0)
            lane.set_location(0,0,0)
            lane.set_simulation(blueprint, world, map)
            lane.set_sensor()
            lane.read()
            sensors['lane'] = lane            
            
        elif sensor == "ObstacleDetector":
            obs = ObstacleDetector()                                    # create an Obstacle detector sensor 
            obs.set_vehicle(vehicle_actor)                              # give the vehicle that the sensor will be attached to  
            obs.set_rotation(vehicle_transform.rotation.pitch, vehicle_transform.rotation.yaw, vehicle_transform.rotation.roll)
            obs.set_location(X=2,Y=0,Z=1)
            obs.set_simulation(blueprint, world, map)
            obs.set_sensor(distance=5)
            obs.read()
            sensors['obs'] = obs            

        elif sensor == "IMU":
            imu = IMU()                                                 # create an IMU sensor 
            imu.set_vehicle(vehicle_actor)                              # give the vehicle that the sensor will be attached to  
            imu.set_rotation(0,0,0)
            imu.set_location(0,0,0)
            imu.set_simulation(blueprint, world, map)
            imu.set_sensor()
            imu.read()
            sensors['imu'] = imu    

        elif sensor == "GNSS":
            gnss = GNSS()                                               # create a GNSS sensor 
            gnss.set_vehicle(vehicle_actor)                             # give the vehicle that the sensor will be attached to  
            gnss.set_rotation(0,0,0)
            gnss.set_location(0,0,0)
            gnss.set_simulation(blueprint, world, map)
            gnss.set_sensor()        
            gnss.read()
            sensors['gnss'] = gnss    

        elif sensor == "Camera Semantic":
            camera_sem = CameraSemantic()                               # create a Semantic Segmentation Camera sensor 
            camera_sem.set_vehicle(vehicle_actor)                       # give the vehicle that the sensor will be attached to  
            camera_sem.set_rotation(0,0,0)
            camera_sem.set_location(0,0,3)
            camera_sem.set_simulation(blueprint, world, map)
            camera_sem.set_sensor()
            camera_sem.read()
            sensors['camera_sem'] = camera_sem    

        elif sensor == "Camera RGB":  
            camera_rgb = CameraRGB()                                    # create an RGB Camera sensor 
            camera_rgb.set_vehicle(vehicle_actor)                       # give the vehicle that the sensor will be attached to  
            camera_rgb.set_rotation(0,0,0)
            camera_rgb.set_location(0,0,2)
            camera_rgb.set_simulation(blueprint, world, map)
            camera_rgb.set_sensor()
            camera_rgb.read()
            sensors['camera_rgb'] = camera_rgb    

    return sensors


def save_waypoints(waypoints):
    """
    Description:
        Function save_waypoints is used for saving the waypoints in a file

    Args:
        waypoints (list): The waypoints that will be saved in the file 
    """    

    fileData = open("data.txt", 'w')
    for waypoint in waypoints:
        fileData.write(str(waypoint.transform.location.x) + "  " + str(waypoint.transform.location.y) + "  " + str(waypoint.transform.location.z) + "\n")    
    fileData.close() 


def load_waypoints(world, map): 
    """
    Description:
        Function load_waypoints is used for loading waypoints from a file 

    Args:
        world        (carla.World)     :    World object of CARLA API
        map          (carla.Map)       :    Map object of CARLA API

    Returns:
        list: A list with the loaded waypoints
    """    

    fileData = open("data.txt", 'r')
    Lines = fileData.readlines() 
    waypoints = []
    i = 0
    for line in Lines: 
        coordinates = line.split()
        location = carla.Location(float(coordinates[0]), float(coordinates[1]), float(coordinates[2]))
        waypoint = map.get_waypoint(location)
        waypoints.append(waypoint)
        world.debug.draw_string(waypoint.transform.location, '{}'.format(i), draw_shadow=False, color=carla.Color(r=0, g=0, b=255), life_time=1000, persistent_lines=True)
        i += 1

    return waypoints


def pruning(map, waypoints):
    """[summary]
    Description:
        Function pruning is used for eliminating double waypoints and filling gaps between distant waypoints in waypoints' list

    Args:
        map          (carla.Map)       :    Map object of CARLA API
        waypoints    (list)            :    A list with the waypoints that will be pruned 

    Returns:
        list: The pruned waypoints 
    """    

    # throw double waypoints
    waypoints_initial = waypoints
    waypoints = []
    i = 0
    while True: 

        p1 = carla.Location(waypoints_initial[i].transform.location.x, waypoints_initial[i].transform.location.y, waypoints_initial[i].transform.location.z)
        p2 = carla.Location(waypoints_initial[i + 1].transform.location.x, waypoints_initial[i + 1].transform.location.y, waypoints_initial[i + 1].transform.location.z)
        dist = p1.distance(p2)

        if dist < 0.5:
            i += 1 
        else:
            waypoints.append(waypoints_initial[i])
            i += 1

        if i == len(waypoints_initial) - 1:
            break
    
    # fill gaps between distant waypoints
    waypoints_initial = waypoints
    waypoints = []
    i = 0
    while True: 

        p1 = carla.Location(waypoints_initial[i].transform.location.x, waypoints_initial[i].transform.location.y, waypoints_initial[i].transform.location.z)
        p2 = carla.Location(waypoints_initial[i + 1].transform.location.x, waypoints_initial[i + 1].transform.location.y, waypoints_initial[i + 1].transform.location.z)
        dist = p1.distance(p2)

        if dist > 4:
            a = [waypoints_initial[i].transform.location.x, waypoints_initial[i].transform.location.y]
            b = [waypoints_initial[i + 1].transform.location.x, waypoints_initial[i + 1].transform.location.y]
            points = np.linspace(a,b, num=round(dist / 2))
            for j in range(len(points) - 1):
                point = carla.Location(points[j][0],points[j][1],0)
                w = map.get_waypoint(point, project_to_road=False, lane_type=carla.LaneType.Any)
                if w != None:
                    waypoints.append(w)            
            i += 1 
        
        else:
            waypoints.append(waypoints_initial[i])
            i += 1            
         
        if i == len(waypoints_initial) - 1:
            break
    
    return waypoints


def draw_waypoints(world, waypoints, col):
    """
    Description:
        Function draw_waypoints for drawing waypoints

    Args:
        world       (carla.World)     :     World object of CARLA API
        waypoints   (list)            :     The list with the waypoints that will be drawn
        col         (carla.Color)     :     The color of the waypoints 
    """    

    m = 0
    color = carla.Color(r=col, g=col, b=col)
    for waypoint in waypoints:

        if isinstance(waypoint, carla.libcarla.Waypoint):
            world.debug.draw_string(waypoint.transform.location, '{}'.format(m), draw_shadow=False, color=color, life_time=2000)
            pass
        elif isinstance(waypoint, carla.libcarla.Transform):
            world.debug.draw_string(waypoint.location, '{}'.format(m), draw_shadow=False, color=color, life_time=2000)
            pass
        m += 1 
    
    #for j in range(len(waypoints) - 2):
    #    self.world.debug.draw_line(waypoints[j].transform.location, waypoints[j + 1].transform.location, thickness=0.3, color=carla.Color(r=0, g=200, b=0), life_time=1000, persistent_lines=True)        


class Cancel(object):
    """
    Description:
        Class Cancel is used for cancelling a process in the interface and returning in the initial state of specifying trajectory
    
    """
    
    def __init__(self):
        """
        Description:
            Method __init__ is the Constructor of Class Cancel that initializes most of the used variables 
        """        

        self.pub_cancel = VehiclePublisherMQTT(topic='cancel command')
        self.pub = VehiclePublisherMQTT(topic='clean')
        self.sub_cancel = VehicleSubscriberCancelMQTT(topic='cancel')


    def cancel_process(self):
        """
        Description:
            Method cancel_process is used for cancelling a process in the interface 

        Returns:
            boolean: Returns a boolean value with the result of cancelation 
        """      

        if self.sub_cancel.get_cancel():
            self.sub_cancel.set_cancel(False)
            self.pub.publish({'value': " "})
            return True
        return False


    def cancel_now(self, value):
        """
        Description:
            Method cancel_now is used for forcing a process to cancel now 

        Args:
            value (str): The value that will be published to the right topic in order to cancle the process 
        """        

        self.pub_cancel.publish({'value': value})
