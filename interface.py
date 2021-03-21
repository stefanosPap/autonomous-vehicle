import carla
from communicationMQTT import VehicleSubscriberStartStopMQTT, \
                              VehicleSubscriberCoorMQTT, \
                              VehicleSubscriberEnterMQTT, \
                              VehicleSubscriberDoneMQTT, \
                              VehiclePublisherMQTT
class Interface(object):
    def __init__(self, world, map):
        self.world = world 
        self.map = map

    def setupCom(self):
        self.pub_waypoint = VehiclePublisherMQTT(topic='waypoint_choose')
        pub = VehiclePublisherMQTT(topic='text')
        pub_vel = VehiclePublisherMQTT(topic='speed_topic')
        pub_vel_conf = VehiclePublisherMQTT(topic='speed_configure')
        sub_enter = VehicleSubscriberEnterMQTT(topic='enter')
        sub_done = VehicleSubscriberDoneMQTT(topic='done')
        sub_coor = VehicleSubscriberCoorMQTT(topic='coordinates')
    def handle(self, start_waypoint):


        
        msg = {'value': ''}
        self.pub_waypoint.publish(msg)
            
        end_waypoints = [start_waypoint]
          
        text = {'text': ''}
        pub.publish(text)

        vel = {'velocity': 0}  
        pub_vel.publish(vel)
        pub_vel_conf.publish(vel)

        prev_point = []
        num = 1 
        
        while True:
            self.world.tick()
            if sub_enter.get_enter() == True:
                sub_enter.set_enter(False)
                pub.publish(text)
                coordinates = sub_coor.get_coordinates()
                
                if coordinates != []:
                    #point = [int(item) for item in coordinates.split()]
                    point = coordinates
                    if len(point) == 3 and point != prev_point: 
                        way = {'value': 'Location {} at: {}'.format(num, sub_coor.get_location())}
                        self.pub_waypoint.publish(way)
                        num += 1  

                        prev_point = point 
                        point = carla.Location(point[0],point[1],point[2])
                        waypoint = self.map.get_waypoint(point, project_to_road=True, lane_type=carla.LaneType.Any)
                        end_waypoints.append(waypoint)
            
            if sub_done.get_done() == True:
                sub_done.set_done(False)
                break
        return end_waypoints 
    
    def handle_forward(self, start_waypoint):
        waypoints = []                
        waypoint = start_waypoint

        while True:
            self.world.tick()
            if sub_enter.get_enter() == True:
                p = sub_coor.get_coordinates()
                try:
                    p = int(p)
                    
                    text = {'text': ''}
                    pub.publish(text)
                    way = {'value': 'Going forward {} meters'.format(p)}
                    self.pub_waypoint.publish(way)
                    waypoint = start_waypoint
                    
                    while p > 0:
                        current_waypoints = waypoint.next_until_lane_end(1.0)
                        p = p - len(current_waypoints)
                        if p > 0:
                            waypoints = waypoints + current_waypoints
                        else:
                            waypoints = waypoints + current_waypoints[0:p]
                        waypoint = waypoints[len(waypoints) - 1].next(1.0)

                        if len(waypoint) != 1:
                            for i in range(len(waypoint)):
                                final_waypoint = waypoint[i].next_until_lane_end(1.0)
                                if abs(waypoints[len(waypoints) - 1].transform.rotation.yaw - final_waypoint[len(final_waypoint) - 1].transform.rotation.yaw) < 3:
                                    break 
                            waypoint = waypoint[i]

                        else:
                            waypoint = waypoint[0]   
                    break
                except ValueError, TypeError:
                    way = {'value': 'Invalid Value! Try again!'}
                    self.pub_waypoint.publish(way)
                    text = {'text': ''}
                    pub.publish(text)
                sub_enter.set_enter(False)

            '''
            paths = waypoints[len(waypoints) - 1].next(1.0)
            for i in range(len(paths)):
                ways = paths[i].next_until_lane_end(1.0)
                if abs(ways[len(ways) - 1].transform.rotation.yaw - waypoints[len(waypoints) - 1].transform.rotation.yaw) < 3:
                    print("straight")
                elif ways[len(ways) - 1].transform.rotation.yaw > waypoints[len(waypoints) - 1].transform.rotation.yaw:
                    print("right")
                elif ways[len(ways) - 1].transform.rotation.yaw < waypoints[len(waypoints) - 1].transform.rotation.yaw:
                    print("left")
                for i in range(len(ways)):
                    waypoints.append(ways[i])
            '''
