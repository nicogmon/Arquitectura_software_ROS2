[![Review Assignment Due Date](https://classroom.github.com/assets/deadline-readme-button-24ddc0f5d75046c5622901739e7c5dd533143b0c8e959d652212380cedb1ea36.svg)](https://classroom.github.com/a/5Ocj-OtY)
# Receptionist

En esta práctica, el robot debe realizar la prueba del recepcionista (página 59 del [Reglamento](https://athome.robocup.org/wp-content/uploads/2022_rulebook.pdf)), aunque algo descafeinado.

El robot tiene 10 minutos para lo siguiente:
1. El robot parte siempre de la posición inicial, que es justo detrás de la línea de al lado de la pizarra, como se indica en clase.
2. El robot va a la puerta, y cuando haya una persona (< 1.5 metros de la puerta), le pregunta su nombre, y le indica que le acompañe.
3. El robot va hacia la zona de la fiesta, le presenta, le pregunta qué quiere beber y le indica claramente (orientándose, por ejemplo) donde hay un hueco libre donde se puede sentar.
4. El robot va a la barra y le pide al barman la bebida. Considera que ya la lleva cuando el barman le dice "Aquí tienes", o algo similar.
5. El robot va a la persona y "le entrega" la bebida.
6. Goto 2

Puntuación:
* +5 Navegar hasta la puerta
* +5 Detecta persona correctamente
* +5 Navega a la zona de la fiesta
* +5 Le presenta con su nombre
* +5 Le indica correctamente donde sentarse
* +5 Pedir la bebida
* +5 Servir la bebebida

La nota total de la práctica será en función de los puntos: notas = puntos / 40.0
Habrá hasta dos puntos extra en la nota final por hacer un póster del equipo.

# Cero_Dumped

***

![distro](https://img.shields.io/badge/ROS2-Humble-blue) ![distro](https://img.shields.io/badge/Ubuntu%2022-Jammy%20Jellyfish-green) ![License](https://img.shields.io/badge/License-Apache%202.0-blue)
![Language](https://img.shields.io/badge/Language-C%2B%2B-orange)

##### Date: 10/05/2023
##### Group: Cero Dumped
##### Authors: [Ana Martínez Albendea](https://github.com/ana-martinezal2021), [Nicolás García Moncho](https://github.com/nicogmon), [Pablo Sánchez Fernandez](https://github.com/psanchezf2021) and [Iker Peral del Pino](https://github.com/iperal2021)
##### Goal: carry out the receptionist practise of the software architecture subject

## 0.Introduction
The practice is based on a party environment in which the robot acts as a doorman. In this way, when there is a person at the door, he asks his name and what he wants to drink, and takes him to the party area. Once there, he introduces you to the rest of the people and asks the waiter for his drink and then gives it to him and returns to the door to receive the next person.

## Behaviour Trees
A behavior tree (BT) is a mathematical model of plan execution used in computer science, robotics, control systems and video games. They describe switchings between a finite set of tasks in a modular fashion. Their strength comes from their ability to create very complex tasks composed of simple tasks, without worrying how the simple tasks are implemented. BTs present some similarities to hierarchical state machines with the key difference that the main building block of a behavior is a task rather than a state.

Behavior Trees use the concept of Tick, a signal that is sent in a fixed frequency, which starts from the root node and is propagated to its children. By doing this Behavior Trees can react in real-time to events that happen in the world.

The following image is the behaviour trees that we have designed:

![Captura desde 2023-05-09 22-09-17](https://github.com/nicogmon/cero_Dumped/assets/92941166/dac2e730-f3de-4c4a-8d9d-8ee865238d72)
 
## Navigation 2
Nav2 is the professionally supported spiritual successor of the ROS Navigation Stack. This project seeks to find a safe way to have a mobile robot move to complete complex tasks through many types of environments and classes of robot kinematics. Not only can it move from Point A to Point B, but it can have intermediary poses, and represent other types of tasks like object following and more. 

Nav2 uses behavior trees to create customized and intelligent navigation behavior via orchestrating many independent modular servers. A task server can be used to compute a path, control effort, recovery, or any other navigation related task.

It has tools to:

* Load, serve, and store maps (Map Server)
* Localize the robot on the map (AMCL)
* Plan a path from A to B around obstacles (Nav2 Planner)
* Control the robot as it follows the path (Nav2 Controller)
* Smooth path plans to be more continuous and feasible (Nav2 Smoother)
* Convert sensor data into a costmap representation of the world (Nav2 Costmap 2D)
* Build complicated robot behaviors using behavior trees (Nav2 Behavior Trees and BT Navigator)
* Compute recovery behaviors in case of failure (Nav2 Recoveries)
* Follow sequential waypoints (Nav2 Waypoint Follower)
* Manage the lifecycle and watchdog for the servers (Nav2 Lifecycle Manager)
* Plugins to enable your own custom algorithms and behaviors (Nav2 Core)
* Monitor raw sensor data for imminent collision or dangerous situation (Collision Monitor)
* Python3 API to interact with Nav2 in a pythonic manner (Simple Commander)
* A smoother on output velocities to guarantee dynamic feasibility of commands (Velocity Smoother)



## GB-Dialog
GB-dialog simplifies the task of developing actions or behaviors related to dialogue.

GB-dialog contains the library DialogInterface from which we will inherit to develop our dialogue actions. Each action would be specific to an intent (Dialogflow concepts). The library offers methods to do speech-to-text tasks through Google Speech sevice and methods to do Natural Language Processing tasks through Dialogflow, using the ROS package dialogflow_ros (official package, our custom dialogflow_ros). The library also offers a method to do text-to-speech through the package sound_play.

## Goals

- [X] **Navigate to the door**
- [X] **Detect person correctly**
- [X] **Navigate to the party area**
- [X] **Introduces them with their name**
- [ ] **Tells them correctly where to sit**
- [X] **Order the drink**
- [ ] **Serve the drink**

## Code
Dialog essential code:
```cpp
T::NodeStatus
DialogName::tick()
{

  if (status() == BT::NodeStatus::IDLE) {
    name_ = "";
    start_time_ = node_->now();
    dialog_.DialogInterface::speak("Hello, What is your name?"); 
  }
  auto elapsed_time = node_->now() - start_time_;

  if (elapsed_time > 3s){
    if(name_ != "" ){
        
        return BT::NodeStatus::SUCCESS;
    }
    // out_led.value = RED;
    // led_pub_->publish(out_led);

    dialog_.DialogInterface::listen();
    rclcpp::spin_some(dialog_.get_node_base_interface());
  }
  
  return BT::NodeStatus::RUNNING;
}
```

## Install

Needed packages:
- [seekandcapture](https://github.com/Docencia-fmrico/seekandcapture-cero_dumped) by cero-dumped
- [ir_robots](https://github.com/IntelligentRoboticsLabs/ir_robots) repository by Intelligent Robotics Lab

Steps to install and run the package:

1. Clone the git repository in a workspace
```bash
git clone https://github.com/Docencia-fmrico/recepcionist-cero_dumped.git

```
3. Compile the workspace
```bash
colcon build --symlink-install
```
```bash
colcon build --symlink-install --packages-select recepcionist

```

4. Run the package with this command:

```bash
ros2 launch recepcionist perception.launch.py

``` 
> The [ir_robots](https://github.com/IntelligentRoboticsLabs/ir_robots) repository is needed to launch the kobuki and the navigation.
