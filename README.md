# ROS-Gazebo-Traversable-Garmin-Lidar-Lite-v3-Sensor


## Einleitung

Diese ROS/Gazebo Paket enthält die Umsetzung eines Garmin Lidar Lite v3 Sensors zur Entfernungsmessung, der über einen Aktuator um seine Z-Achse schwenkbar ist.
Die Sensordaten werden in Gazebo über ein Ray-Sensor Element erzeugt und im Sensor Node weiterverarbeitet.
Zur Steuerung der Bewegung wird ein JointPositionController Element über den Aktuator Node angesteuert.


## Installation

### Build aus dem Github Repository

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (Framework für Roboteranwendungen),
- [Gazebo] (https://gazebosim.org/home) (Simulationsumgebung)

#### Building

Um die aktuelle Version zu erhalten, Lade dieses Repository in den catkin workspace und compiliere das Paket

	cd catkin_workspace/src
	git clone https://github.com/FaustFN/ROS-Gazebo-Traversable-Gramin-Lidar-Lite-v3-Sensor.git
	cd ../
	rosdep install --from-paths src --ignore-src -r -y
	catkin_make

## Usage

Zum Starten der Testumgebung ist das beiliegende Launchfile auszuführen:

	roslaunch rotating_lidar_sensor_position-controller gazebo.launch

## Launch files

* **gazebo.launch:** launches everything(gazebo, Aktuator Node, Sensor Node, usw.)

	Winkel werden im Rechte-Hand-Koordinatensystem um die positive Z-Achse gegen den Uhrzeigersinn aufgetragen
	Die Null Grad Position liegt entlang der Positiven X-Achse
     - **`actuator_min_angle`** Position des Endanschlages für Rotation im Uhrzeigersinn in ° Default: `-40.0`.
	 - **`actuator_max_angle`** Position des Endanschlages für Rotation gegen den Uhrzeigersinn in ° Default: `40.0`.
	 - **`actuator_max_angle`** Rate mit der der Sensor neue Daten sendet in hz Default: `500`.

## Nodes

### sensor_actuator_node

Node zur Steuereung der Sensorposition.
Wartet auf Eingabe eines Sollwinkels und gleicht diesen mit den im Launch File parametrierten Werten für die Endanschläge ab.
Gibt die Sollwinkelvorgabe an den JointPositionController in gazebo weiter.

#### Subscribed Topics

* **`/sensor/actuator/angle`** ([sensor_msgs/Float64])

	Topic zur Übertragung des Sollwinkels in ° an den Aktuator Node.

#### Published Topics

* **`/actuator_position_controller/command`** ([sensor_msgs/Float64])

	Topic zur Übertragung der Sollwinkelvorgabe an den JointPositionController

#### Parameters

* **`actuator_min_angle`** (double, default: "-40.0")
	Position des Endanschlages für Rotation im Uhrzeigersinn in °
	
* **`actuator_max_angle`** (double, default: "40.0")
	Position des Endanschlages für Rotation gegen den Uhrzeigersinn in °

### lidar_sensor_node
Node zur Verarbeitung der Sensordaten.
Wartet auf Sensordaten aus der Gazebo-Simulation, belegt die gemessenen Entfernungswerte mit einem gaussschen Rauschen gemäß den Sensorspezifikationen 
und stellt diese zusammen mit der Winkelposition des Sensors auf einer neuen Topic zur Verfügung

#### Subscribed Topics

* **`/sensor/laser/scan`** ([sensor_msgs/LaserScan])
	Topic mit der Sensordaten des Ray Sensors aus Gazebo übertragen werden
	
#### Published Topics

* **`/sensor/data`** ([rotating_lidar_sensor_position_controller/lidar_sensor_data])
	Topic veröffentlicht die weiterverarbeiteten Entfernungswerte und die dazugehörige Winkelposition des Sensors
	
	Nachrichtenaufbau:
		Header header				(siehe: https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Header.html)
		std_msgs/Float32 range		(siehe: https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32.html)
		std_msgs/Float32 angle
	
	Für weitere Details zum Aufbau der Nachricht siehe msg/lidar_sensor_data.msg
	
## Einbindung des Sensors in weitere Projekte
