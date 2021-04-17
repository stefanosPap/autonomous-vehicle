import carla
import numpy as np
from utilities import plot_axis, change_coordinate_system, calculate_angle, Cancel
from communicationMQTT import VehicleSubscriberStartStopMQTT, \
                              VehicleSubscriberCoorMQTT, \
                              VehicleSubscriberEnterMQTT, \
                              VehicleSubscriberDoneMQTT, \
                              VehiclePublisherMQTT, \
                              VehicleSubscriberForwardMQTT, \
                              VehicleSubscriberCoorForwardMQTT
from town import Town

class Interface(object):
    def __init__(self, world, map, vehicle_actor):
        self.world = world 
        self.map = map
        self.vehicle_actor = vehicle_actor
        self.setupCom()
        self.cancel = Cancel()

    ####################################
    # Setup publishers and subscribers #
    ####################################
    def setupCom(self):
        self.pub_waypoint = VehiclePublisherMQTT(topic='waypoint_choose')
        self.pub = VehiclePublisherMQTT(topic='clean')
        self.pub_vel = VehiclePublisherMQTT(topic='speed_topic')
        self.pub_vel_conf = VehiclePublisherMQTT(topic='speed_configure')
        self.pub_forward = VehiclePublisherMQTT(topic='forward meters')
        self.pub_turn = VehiclePublisherMQTT(topic='turn number')
        self.pub_turn_clean = VehiclePublisherMQTT(topic='turn clean')

        self.sub_enter = VehicleSubscriberEnterMQTT(topic='enter')
        self.sub_done = VehicleSubscriberDoneMQTT(topic='done')
        self.sub_coor = VehicleSubscriberCoorMQTT(topic='coordinates')
        self.sub_coor_forward = VehicleSubscriberCoorForwardMQTT(topic='coordinates_forward')
        self.sub_forward = VehicleSubscriberForwardMQTT(topic='forward')

    #############################
    # Handle location decisions #
    #############################
    def handle(self, start_waypoint):

        msg = {'value': ''}
        self.pub_waypoint.publish(msg)
            
        end_waypoints = [start_waypoint]
          
        text = {'value': ''}
        self.pub.publish(text)

        vel = {'velocity': 0}  
        self.pub_vel.publish(vel)
        self.pub_vel_conf.publish(vel)

        prev_point = []
        num_of_locations = 1 

        town = Town()
        print("DD")

        while True:
            self.world.tick()
            if self.cancel.cancel_process():
                self.cancel.cancel_now(False)
                print("V")
                break 
            
            # Waiting for ENTER button to be pressed
            if self.sub_enter.get_enter() == True:
                
                self.sub_enter.set_enter(False)
                self.pub.publish(text)
                
                # Take the location that the user has specified
                location = self.sub_coor.get_location()

                #try catch block in order to check if this location exists in the current map 
                try:
                    destinations = town.get_destinations()
                    point = destinations[location]
                except:
                    way = {'value': 'Specify another location!'}
                    self.pub_waypoint.publish(way)
                    continue

                # if the given location is different from thw previous then log this
                if point != prev_point: 
                    way = {'value': 'Location {} at: {}'.format(num_of_locations, self.sub_coor.get_location())}
                    self.pub_waypoint.publish(way)
                    self.pub.publish({'value': ''})

                    num_of_locations += 1  

                    prev_point = point 
                    point = carla.Location(point[0],point[1],point[2])
                    waypoint = self.map.get_waypoint(point, project_to_road=True, lane_type=carla.LaneType.Any)
                    end_waypoints.append(waypoint)
        
                break
        return end_waypoints 
    
    ###########################
    # Handle forward decision #
    ###########################
    def handle_forward(self, start_waypoint):
        waypoints = []                
        waypoint = start_waypoint
        self.pub_forward.publish({'value': "Specify the distance in meters you want to go forward!"})
        while True:
            self.world.tick()
            if self.cancel.cancel_process():
                self.cancel.cancel_now(False)
                break 
            
            if self.sub_enter.get_enter():
                self.sub_enter.set_enter(False)
                p = self.sub_coor_forward.get_coordinates()
                try:
                    p = int(p)

                except ValueError:
                    self.pub_waypoint.publish({'value': 'Invalid Value! Try again!'})
                    self.pub.publish({'value': ''})
                    continue
            
                self.pub.publish({'value': ''})
                self.pub_waypoint.publish({'value': 'Going forward {} meters'.format(p)})
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
                            if abs(waypoint[i].transform.rotation.yaw - final_waypoint[len(final_waypoint) - 1].transform.rotation.yaw) < 3:
                                break 
                        waypoint = waypoint[i]

                    else:
                        waypoint = waypoint[0]   
                break
                
        return waypoints

    ########################
    # Handle turn decision #
    ########################
    def handle_turn(self, start_waypoint, turn):
        #w = start_waypoint.previous(1.0)
        #w = w[0]
        #self.world.debug.draw_string(w.transform.location, 'F', draw_shadow=False, color=carla.Color(r=255, g=0, b=255), life_time=1000, persistent_lines=True)
        waypoints = [start_waypoint]
        #if not start_waypoint.is_junction:
        try:
            waypoints = start_waypoint.next_until_lane_end(1.0)
        except RuntimeError:
            pass 
        
        #waypoint = start_waypoint
        
        #while True:
        #    next = waypoint.next(1.0)
        #    for i in range(len(next)):
        #        if next[i].lane_id == waypoint.lane_id:
        #            waypoints.append(next[i])
        #            break
        #    if next[i].is_junction:
        #        break
        #    waypoint = next[i]
        
        #while True:
        #    if waypoints[len(waypoints) - 1].next(1.0)[0].is_junction:
        #        break
        #    waypoints += waypoints[len(waypoints) - 1].next_until_lane_end(1.0)
        
        #print(waypoints[len(waypoints) - 1].is_junction)
        #print(waypoints[0].is_junction)
        paths = waypoints[len(waypoints) - 1].next(1.0)

        for i in range(len(paths)):
            
            ways = paths[i].next_until_lane_end(1.0)

            angle = calculate_angle(ways)
            
            final_point = change_coordinate_system(paths[i].transform, ways[len(ways) - 1].transform.location)
            initial_point = change_coordinate_system(ways[0].transform, ways[0].transform.location)

            if round(angle, 1) == 0.8 and turn == "STRAIGHT":

                self.world.debug.draw_string(ways[len(ways) - 1].transform.location, "XXXXXXX", draw_shadow=False, color=carla.Color(r=0, g=250, b=0), life_time=1000, persistent_lines=True) 
                self.pub_waypoint.publish({'value': 'Going STRAIGHT at the next junction'})
                self.pub.publish({'value': ''})
                for i in range(len(ways)):
                    waypoints.append(ways[i])
                break
            
            elif ((final_point.x < 0 and final_point.y < 0) or (final_point.x > 0 and final_point.y > 0)) and turn == "RIGHT" and round(angle, 1) != 0.8:
                
                if self.right_counter > 1:
                    self.pub_turn.publish({'value': "Specify the number of turn you want to follow!"})
                    while True:
                        self.world.tick()
                        if self.sub_enter.get_enter():
                            self.sub_enter.set_enter(False)
                            choice = self.sub_coor_forward.get_coordinates()

                            try:
                                choice = int(choice)

                                if choice in list(range(1, len(self.right_turn_endings) + 1)):
                                    k = 0
                                    while self.right_turn_endings[choice - 1].transform.rotation.yaw != ways[len(ways) - 1].transform.rotation.yaw:
                                        ways = paths[k].next_until_lane_end(1.0)
                                        k += 1

                                    for j in range(len(ways)):
                                        waypoints.append(ways[j])
                                    break

                                else:
                                    self.pub_waypoint.publish({'value': 'Invalid Value! Try again!'})
                                    self.pub_turn_clean.publish({'value': ''})
                                    continue
                            
                            except ValueError:
                                self.pub_waypoint.publish({'value': 'Invalid Value! Try again!'})
                                self.pub_turn_clean.publish({'value': ''})
                                continue
                
                else:
                    for i in range(len(ways)):
                        waypoints.append(ways[i])

                self.world.debug.draw_string(ways[len(ways) - 1].transform.location, "XXXXXXX", draw_shadow=False, color=carla.Color(r=0, g=250, b=0), life_time=1000, persistent_lines=True)                    
                self.pub_waypoint.publish({'value': 'Turn RIGHT at the next junction'})
                self.pub.publish({'value': ''})
                break 
            
            elif ((final_point.x > 0 and final_point.y < 0) or (final_point.x < 0 and final_point.y > 0)) and turn == "LEFT" and round(angle, 1) != 0.8:
                
                if self.left_counter > 1:
                    self.pub_turn.publish({'value': "Specify the number of turn you want to follow!"})
                    while True:
                        self.world.tick()
                        if self.sub_enter.get_enter():
                            self.sub_enter.set_enter(False)
                            choice = self.sub_coor_forward.get_coordinates()

                            try:
                                choice = int(choice)

                                if choice in list(range(1, len(self.left_turn_endings) + 1)):
                                    k = 0
                                    while self.left_turn_endings[choice - 1].transform.rotation.yaw != ways[len(ways) - 1].transform.rotation.yaw:
                                        ways = paths[k].next_until_lane_end(1.0)
                                        k += 1

                                    for j in range(len(ways)):
                                        waypoints.append(ways[j])
                                    break

                                else:
                                    self.pub_waypoint.publish({'value': 'Invalid Value! Try again!'})
                                    self.pub_turn_clean.publish({'value': ''})
                                    continue

                            except ValueError:
                                self.pub_waypoint.publish({'value': 'Invalid Value! Try again!'})
                                self.pub_turn_clean.publish({'value': ''})
                                continue
                
                else:
                    for i in range(len(ways)):
                        waypoints.append(ways[i])

                self.world.debug.draw_string(ways[len(ways) - 1].transform.location, "XXXXXXX", draw_shadow=False, color=carla.Color(r=0, g=250, b=0), life_time=1000, persistent_lines=True)    
                self.pub_waypoint.publish({'value': 'Turn LEFT at the next junction'})
                self.pub.publish({'value': ''})
                break 
            
            elif i == len(paths) - 1:
                self.pub_waypoint.publish({'value': 'Unable to go {} at the next junction'.format(turn)})
                self.pub.publish({'value': ''})
                return []

        self.world.debug.draw_string(waypoints[len(waypoints) - 1].transform.location, 'X', draw_shadow=False, color=carla.Color(r=0, g=0, b=255), life_time=1000, persistent_lines=True)
        return waypoints
    
    ###################################################
    # Function for informing about possible direction #
    ###################################################
    def turn_info(self, start_waypoint):
        
        # This try except block is used in case of the start waypoint is at the end of the lane and it cannot produse new waypoints. 
        # Therefore it throws a RuntimeError and we use as waypoints the starting waypoint  
        waypoints = [start_waypoint]
        try:
            waypoints = start_waypoint.next_until_lane_end(1.0)
        except RuntimeError:
            pass 
            
        paths = waypoints[len(waypoints) - 1].next(1.0)
        
        turn = "Possible paths:"
        turn_right = ""
        turn_left = ""
        turn_draw_right = ""
        turn_draw_left = ""

        self.right_counter = 0
        self.left_counter = 0
        self.right_turn_endings = []
        self.left_turn_endings = []
        
        for i in range(len(paths)):

            ways = paths[i].next_until_lane_end(1.0)

            angle = calculate_angle(ways)

            final_point = change_coordinate_system(paths[i].transform, ways[len(ways) - 1].transform.location)
            initial_point = change_coordinate_system(ways[0].transform, ways[0].transform.location)
            
            if round(angle, 1) == 0.8: 
                turn += " STRAIGHT"            
                turn_draw = "STRAIGHT"

                self.world.debug.draw_string(ways[len(ways) - 1].transform.location, turn_draw, draw_shadow=False, color=carla.Color(r=0, g=0, b=250), life_time=1000, persistent_lines=True)

            elif ((final_point.x < 0 and final_point.y < 0) or (final_point.x > 0 and final_point.y > 0)):
                self.right_counter += 1 
                self.right_turn_endings.append(ways[len(ways) - 1])

                turn_right += " RIGHT {}".format(self.right_counter)
                turn_draw_right = "RIGHT {}".format(self.right_counter)

                self.world.debug.draw_string(ways[len(ways) - 1].transform.location, turn_draw_right, draw_shadow=False, color=carla.Color(r=0, g=0, b=250), life_time=1000, persistent_lines=True)

            elif ((final_point.x > 0 and final_point.y < 0) or (final_point.x < 0 and final_point.y > 0)):
                self.left_counter += 1
                self.left_turn_endings.append(ways[len(ways) - 1])
                
                turn_left += " LEFT {}".format(self.left_counter)                
                turn_draw_left = "LEFT {}".format(self.left_counter)

                self.world.debug.draw_string(ways[len(ways) - 1].transform.location, turn_draw_left, draw_shadow=False, color=carla.Color(r=0, g=0, b=250), life_time=1000, persistent_lines=True)

            
            #for j in range(1, len(ways)):
            #    self.world.debug.draw_string(ways[j].transform.location, '{}'.format(round(ways[j].transform.rotation.yaw)), draw_shadow=False, color=carla.Color(r=0, g=0, b=0), life_time=1000, persistent_lines=True)
            
        if self.right_counter == 1: 
            turn_right = ''.join([i for i in turn_right if not i.isdigit()])
            
        if self.left_counter == 1: 
            turn_left = ''.join([i for i in turn_left if not i.isdigit()])

        turn += turn_left + turn_right
        self.pub_waypoint.publish({'value': turn})
