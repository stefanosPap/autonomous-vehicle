from commlib.transports.mqtt import Subscriber, ConnectionParameters, Publisher


class VehiclePublisherMQTT:
    """
    Description:
        Class VehiclePublisherMQTT implements the process of publishing messages to a specific topic 
    """    

    def __init__(self, topic):
        """
        Description:
            Method __init__ is the Constructor of Class VehiclePublisherMQTT that initializes most of the used variables 
         
        Args:
            topic (str): The topic that the data will be published 
        """        

        self.topic = topic
        #self.connection_parameters = ConnectionParameters(host='test.mosquitto.org', port=1883)
        self.connection_parameters = ConnectionParameters(host='localhost', port=1883)
        self.publisher = Publisher(topic=self.topic, conn_params=self.connection_parameters)


    def publish(self, msg):
        """
        Description:
            Method publish is used to publish a message in the specified topic 

        Args:
            msg (dictionary): The message that is published 
        """        
        self.publisher.publish(msg)


class VehicleSubscriberMQTT:
    """
    Description:
        Class VehicleSubscriberMQTT implements the process of subscribing to a specific topic and is the parent class of each other subscriber  
    """    

    def __init__(self, topic):
        """
        Description:
            Method __init__ is the Constructor of Class VehicleSubscriberMQTT that initializes most of the used variables 
         
        Args:
            topic (str): The topic that the client will subscribe and get the data 
        """  

        self.topic = topic
        self.subscribe()


    def subscribe(self):
        """
        Description:
            Method subscribe is used to subscribe to the specified topic 
        """        

        #connection_parameters = ConnectionParameters(host='test.mosquitto.org', port=1883)
        connection_parameters = ConnectionParameters(host='localhost', port=1883)
        
        # initialize Subscriber object 
        sub = Subscriber(topic=self.topic, on_message=self.data_callback, conn_params=connection_parameters)
        sub.run()


"""
The following classes inherit the class VehicleSubscriberMQTT and each of them implements a sinle subscriber to a specific topic. 
The data_callback function of each class is used to manipulate the data when a new message has been arrived.        
"""

class VehicleSubscriberAggressiveMQTT(VehicleSubscriberMQTT):   
    def __init__(self, topic):
        super().__init__(topic)
        self.aggressive = 0
        self.change_aggressive = False

    def data_callback(self, msg):
        if self.topic == 'aggressive' and self.change_aggressive == False:
            self.aggressive = msg['aggressive']
            self.change_aggressive = True

    def get_aggressive(self):
        return self.aggressive

    def get_change_aggressive(self):
        return self.change_aggressive

    def set_change_aggressive(self, value):
        self.change_aggressive = value

class VehicleSubscriberLawfulMQTT(VehicleSubscriberMQTT):
    def __init__(self, topic):
        super().__init__(topic)
        self.lawful = 0
        self.change_lawful = False

    def data_callback(self, msg):
        if self.topic == 'lawful' and self.change_lawful == False:
            self.lawful = msg['lawful']
            self.change_lawful = True

    def get_lawful(self):
        return self.lawful

    def get_change_lawful(self):
        return self.change_lawful

    def set_change_lawful(self, value):
        self.change_lawful = value

class VehicleSubscriberLogMQTT(VehicleSubscriberMQTT):
    def __init__(self, topic):
        super().__init__(topic)
        self.log = " "

    def data_callback(self, msg):
        if self.topic == 'log':
            self.log = msg['log']

    def get_log(self):
        return self.log

    def set_log(self, log):
        self.log = log


class VehicleSubscriberForwardMQTT(VehicleSubscriberMQTT):
    def __init__(self, topic):
        super().__init__(topic)
        self.forward = False

    def data_callback(self, msg):
        if self.topic == 'forward':
            self.forward = msg['forward']

    def get_forward(self):
        return self.forward

    def set_forward(self, forward):
        self.forward = forward


class VehicleSubscriberTurnMQTT(VehicleSubscriberMQTT):
    def __init__(self, topic):
        super().__init__(topic)
        self.turn = None

    def data_callback(self, msg):
        if self.topic == 'turn_junction':
            self.turn = msg['turn_junction']

    def get_turn(self):
        return self.turn

    def set_turn(self, value):
        self.turn = value

class VehicleSubscriberCoorMQTT(VehicleSubscriberMQTT):
    def __init__(self, topic):
        super().__init__(topic)
        self.location = None

    def data_callback(self, msg):
        if self.topic == 'coordinates':
            self.location = msg['location']

    def get_location(self):
        return self.location


class VehicleSubscriberCoorForwardMQTT(VehicleSubscriberMQTT):
    def __init__(self, topic):
        super().__init__(topic)
        self.coordinates = ""
        self.location = None

    def data_callback(self, msg):
        if self.topic == 'coordinates_forward':
            self.coordinates = msg['coordinates_forward']
            self.location = msg['location']

    def get_coordinates(self):
        return self.coordinates

    def get_location(self):
        return self.location

    def set_coordinates(self, coordinates):
        self.coordinates = coordinates


class VehicleSubscriberEnterMQTT(VehicleSubscriberMQTT):
    def __init__(self, topic):
        super().__init__(topic)
        self.enter = False

    def data_callback(self, msg):
        if self.topic == 'enter':
            self.enter = msg['enter']

    def get_enter(self):
        return self.enter

    def set_enter(self, enter):
        self.enter = enter


class VehicleSubscriberCancelMQTT(VehicleSubscriberMQTT):
    def __init__(self, topic):
        super().__init__(topic)
        self.cancel = False

    def data_callback(self, msg):
        print(msg)
        if self.topic == 'cancel':
            self.cancel = msg['cancel']

    def get_cancel(self):
        return self.cancel

    def set_cancel(self, cancel):
        self.cancel = cancel


class VehicleSubscriberDoneMQTT(VehicleSubscriberMQTT):
    def __init__(self, topic):
        super().__init__(topic)
        self.done = False

    def data_callback(self, msg):
        if self.topic == 'done':
            self.done = msg['done']

    def get_done(self):
        return self.done

    def set_done(self, done):
        self.done = done


class VehicleSubscriberChangeGoalMQTT(VehicleSubscriberMQTT):
    def __init__(self, topic):
        super().__init__(topic)
        self.change_goal = False

    def data_callback(self, msg):
        if self.topic == 'change_goal':
            self.change_goal = msg['change_goal']

    def get_change_goal(self):
        return self.change_goal

    def set_change_goal(self, change_goal):
        self.change_goal = change_goal


class VehicleSubscriberPositionMQTT(VehicleSubscriberMQTT):
    def __init__(self, topic):
        super().__init__(topic)
        self.pos = None

    def data_callback(self, msg):
        if self.topic == 'position':
            self.pos = msg['position']

    def get_pos(self):
        return self.pos


class VehicleSubscriberStartStopMQTT(VehicleSubscriberMQTT):
    def __init__(self, topic):
        super().__init__(topic)
        self.start = False
        self.stop = False

    def data_callback(self, msg):
        if msg['trigger'] == "start":
            self.start = True
            self.stop = False
        if msg['trigger'] == "stop":
            self.stop = True
            self.start = False

    def get_start(self):
        return self.start

    def get_stop(self):
        return self.stop

    def set_start(self, start):
        self.start = start

    def set_stop(self, stop):
        self.stop = stop


class VehicleSubscriberVelocityMQTT(VehicleSubscriberMQTT):
    def __init__(self, topic):
        super().__init__(topic)
        self.velocity = 0

    def data_callback(self, msg):
        if self.topic == 'speed_configure':
            self.velocity = msg['velocity']

    def get_velocity(self):
        return self.velocity


class VehicleSubscriberLeftRightMQTT(VehicleSubscriberMQTT):
    def __init__(self, topic):
        super().__init__(topic)
        self.turn = None

    def data_callback(self, msg):
        if self.topic == 'turn':
            self.turn = msg['value']

    def get_turn(self):
        return self.turn

    def set_turn(self, value):
        self.turn = value
