# Thesis Title
*Autonomous vehicle’s graphical interface development for driving
behavior parameterization and remote controlling.*

# Abstract
&nbsp;&nbsp;&nbsp;The transportation of people over the years was an integral part of their daily lives.
For this reason, the first efforts to manufacture a car began in the 18th century. Over the
years, the industries were constructing more and more modern cars to offer the people
easy and comfortable transportations. In recent years, the most known companies and
universities perform research aiming to create autonomous vehicles, which will change the
way traditional cars work. Some of the main problems that the technology of self-driving
cars will contribute drastically, are the saving of significant time in people’s daily lives, the
reduction of road accidents and consequently the safer transportations, the contribution
to fuel economy and the reduction of environment’s pollution.<br/>
&nbsp;&nbsp;&nbsp;To date, a fully autonomous vehicle has not yet been constructed, which operates
without any human intervention. When this technology becomes a reality, the vehicle
will have a full and precise perception of the external environment’s conditions, will
make the right decisions every time, and will be able to exchange information with
other vehicles to cooperate for better operation of the overall traffic. However, a lot
of effort and research is demanded in the sector of autonomous driving to create a vehicle
that successfully corresponds to numerous scenarios and conditions that occur inside
the traffic. The implementation of such a system requires solving the problems of the
external environment’s perception, the right behavior choice, and the safe and smooth
transition to the final destination by obeying the traffic rules and avoiding dynamic and
static obstacles. To solve the aforementioned problems, suitable equipment is needed that
includes state-of-the-art sensors which will take as input the environment’s measurements.
The measurements will be analyzed by a central processing unit and finally, the right
decision will be taken.<br/>
&nbsp;&nbsp;&nbsp;The specific thesis deals with the development of a user interface that could be used
by an autonomous vehicle. Concurrently, an operational self-driving car is developed in
which the user interface is adapted. The user interface can send remote commands to
the autonomous vehicle about its motion and can provide it with particular parameters
to adjust its behavior according to the user’s desires. The developed autonomous driving
system is an ego-only system and a variation of a modular architecture was used. The
system’s development is implemented by using Python programming language and the
experiments are conducted in CARLA Simulator. To operate, solutions for the problems
of perception, path planning, behavior selection, and kinematic control are provided to the
autonomous vehicle. The user interface is implemented in the flow-based programming
tool that is called NodeRED and communicates with the overall system by using the
MQTT protocol and Mosquitto message broker. This way indicates that the purpose of
the particular thesis is the ability to send remote commands to the autonomous vehicle,
defining the rate of aggressiveness and legality that the user wants to have the vehicle.
As a result, the autonomous vehicle can be included in the ascending technology of the
Internet of Things, being part of a network with devices that are connected between
them.<br/>
&nbsp;&nbsp;&nbsp;The system has been tested for its correct execution of the remote commands that
are sent by the interface, examining the successful communication between them and
the behavior’s adaptation when the values of aggressiveness and legality are sent. The
experiments were conducted in the environment of the CARLA simulator for different
values of the parameters and the effectiveness has been tested as to the road violations,
the route completion, and the safety the vehicle had during its motion.

# Depedencies
The project was implemented using Ubuntu 16.04. In order to execute the programm it is needed to install the following depedencies: </br>
* [Carla Simulator](https://carla.org/)
* [Python 3.7.x](https://www.python.org/downloads/release/python-370/)
* [commlib-py](https://github.com/robotics-4-all/commlib-py)
* [NumPy](https://numpy.org/)
* [SciPy](https://www.scipy.org/)
* [pandas](https://pandas.pydata.org/)
* [Mosquitto](https://mosquitto.org/download/)
* [NodeRED](https://nodered.org/docs/getting-started/local)
<p> After installing the above depedencies, the code can be cloned to your local machine and run properly. </p>

# Installing and Running the Graphical Interface in NodeRED

After installing NodeRED as mentioned in Depedencies section, you can use the created graphical interface by following the steps below:
* Open a terminal and run the command:
  <pre> $ node-red </pre>
* Open a browser at the address http://127.0.0.1:1880/ 
* Select from the up-right menu: Import -> select a file to import -> Choose file *flows.json* from your local machine 
* In order to use the interface open a new browser tab at the address http://127.0.0.1:1880/ui/ 

<p>The graphical interface looks like the image below:</p>

![alt text](https://github.com/stefanosPap/autonomous-vehicle/blob/master/images/interface.png)

# Starting the autonomomous car
* Step 1: Open the Graphical Interface as mentioned in the previous section.
* Step 2: Open a terminal in order to run the simulator by executing the commands below:
<pre> 
$ cd /path-to-carla-installation/ 
In my case: $ cd /opt/carla-simulator/bin/ 
$ ./CarlaUE.sh
</pre> 
* Step 3: When the simulator is on open a new terminal in order to start the autonomous vehicle and run:
<pre>
$ python3.7 main.py n
where n is the number of vehicles and pedestrians, for 20 vehicles and 20 pedestrians run:
$ python3.7 main.py 20
</pre>
* Step 4: In order to choose a destination for the vehicle go to the interface and follow the steps below:
<pre>
1. In tab "Specify next destination" select in "Log new" dropdown box the option "Location".
2. In tab "Location" select in dropdown box a destination. For navigating to the "Gas Station" select the option "Gas Station".
3. In tab "Specify next destination" press the button "ENTER" to log the destination and then press the button "DONE" to complete the procedure and create the optimal path to the destination.
</pre>
<pre>
Briefly:
1. "Specify next destination" tab -> "Log new" dropdown box -> Select "Location"
2. "Location" tab -> Select destination e.g "Gas Station"
3. "Specify next destination" tab -> press "ENTER" -> press "DONE"
</pre>

# Usage
After giving the choices below, the path that is created is the one depicted in the image below:
1. *Direction* -> **STRAIGHT** (blue segment)
2. *Direction* -> **RIGHT** (blue segment)
3. *Direction* -> **FORWARD** (green segment)
4. *Location* -> **Highway** (orange segmemt)
5. *Direction* -> **STRAIGHT** (blue segment)
6. *Location* -> **Gas Station** (orange segment)

<p align="center">
  <img src="https://github.com/stefanosPap/autonomous-vehicle/blob/master/images/trajectory.png" />
</p>

After the path has been created the vehicle is ready to start folloing the trajectory. In order to start the vehicle press the *START* button and whenever you want  to stop the vehicle press the *STOP* button. The car will navigate autonomously by taking decisions according to the values of *Aggressive* and *Lawful* that are given by the user through the two sliders in *Behavior* tab. By clicking *RIGHT LANE* or *LEFT LANE* the vehicle is changing its lane accordingly. The button *CHANGE GOAL* is used to change the destination and the overall trajectory. When clicking this button you should design your path from the beginning. The buttons that were decribed are refered at the images below: 

<p align="center">
  <img src="https://github.com/stefanosPap/autonomous-vehicle/blob/master/images/handle_vehicle.png" />
</p>
<p align="center">
  <img src="https://github.com/stefanosPap/autonomous-vehicle/blob/master/images/behavior.png" />
</p>
