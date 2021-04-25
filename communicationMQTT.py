from commlib.transports.mqtt import Subscriber, ConnectionParameters, Publisher


class VehiclePublisherMQTT:
    def __init__(self, topic):
        self.topic = topic
        self.connection_parameters = ConnectionParameters(host='test.mosquitto.org', port=1883)
        self.publisher = Publisher(topic=self.topic, conn_params=self.connection_parameters)

    def publish(self, msg):
        self.publisher.publish(msg)


class VehicleSubscriberMQTT:
    def __init__(self, topic):
        self.topic = topic
        self.subscribe()

    def subscribe(self):
        connection_parameters = ConnectionParameters(host='test.mosquitto.org', port=1883)
        sub = Subscriber(topic=self.topic, on_message=self.data_callback, conn_params=connection_parameters)
        sub.run()


class VehicleSubscriberAggressiveMQTT(VehicleSubscriberMQTT):
    def __init__(self, topic):
        super().__init__(topic)
        self.aggressive = 0

    def data_callback(self, msg):
        if self.topic == 'aggressive':
            self.aggressive = msg['aggressive']

    def get_aggressive(self):
        return self.aggressive

    def set_aggressive(self):
        self.aggressive = 0


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


class VehicleSubscriberCautiousMQTT(VehicleSubscriberMQTT):
    def __init__(self, topic):
        super().__init__(topic)
        self.cautious = 0

    def data_callback(self, msg):
        if self.topic == 'cautious':
            self.cautious = msg['cautious']

    def get_cautious(self):
        return self.cautious

    def set_cautious(self):
        self.cautious = 0


class VehicleSubscriberCoorMQTT(VehicleSubscriberMQTT):
    def __init__(self, topic):
        super().__init__(topic)
        self.location = None

    def data_callback(self, msg):
        if self.topic == "coordinates":
            self.location = msg['location']

    def get_location(self):
        return self.location


class VehicleSubscriberCoorForwardMQTT(VehicleSubscriberMQTT):
    def __init__(self, topic):
        super().__init__(topic)
        self.coordinates = ""
        self.location = None

    def data_callback(self, msg):
        if self.topic == "coordinates_forward":
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


class VehicleSubscriberBehaviorMQTT(VehicleSubscriberMQTT):
    def __init__(self, topic):
        super().__init__(topic)
        self.behavior = False

    def data_callback(self, msg):
        if self.topic == 'behavior':
            self.behavior = msg['behavior']

    def get_behavior(self):
        return self.behavior

    def set_behavior(self, behavior):
        self.behavior = behavior


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
