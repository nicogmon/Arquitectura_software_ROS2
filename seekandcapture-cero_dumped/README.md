# SeekAndCapture

En esta práctica el robot debe:

1. Buscar a las personas a su alrededor.
2. Cuando detecta nua, debe ir hacia ella, incluso si ésta se mueve.
3. Cuando esté cerca, debe hacer alguna señal (sonido, mensaje,..)
4. El robot busca a otra persona diferente

* El software debe tener un subsistema de percepción que detecte a la persona en la imagen, transforme esa detección a una detección 3D y genere un frame desde "odom" para la detección. Usa los nodos del paquete perception_asr para esto.
* Debe haber un subsistema de actuación que, a partir de los frames de las percepciones de personas, elija una y genere los comandos para seguirla. Debe pararse a ~1 metro y hacer la señal.

Puntuación:

* Sistema de percepción funcionando correctamente [+1]
* El robot se aproxima a una persona [+2]
* El robot usa PIDs para seguir a la persona [+1]
* El robot se para y avisa cuando ha "cogido" a una persona [+1]
* El robot usa Behavior Trees para organizar su comportamiento [+2]
* El robot busca y sigue a otra persona cuando ha alcanzado a una [+2]
* Todo se lanza con un solo launcher. Tiene un sistema de depuración efectivo [+1]

La práctica se entrega el Martes 28/03/2023

***

![distro](https://img.shields.io/badge/ROS2-Humble-blue) ![distro](https://img.shields.io/badge/Ubuntu%2022-Jammy%20Jellyfish-green) ![License](https://img.shields.io/badge/License-Apache%202.0-blue)
![Language](https://img.shields.io/badge/Language-C%2B%2B-orange)

##### Date: 28/03/2023
##### Group: Cero Dumped
##### Authors: [Ana Martínez Albendea](https://github.com/ana-martinezal2021), [Nicolás García Moncho](https://github.com/nicogmon), [Pablo Sánchez Fernandez](https://github.com/psanchezf2021) and [Iker Peral del Pino](https://github.com/iperal2021)
##### Goal: carry out the seek and capture practise of the software architecture subject
##### Particularities: regulate the PID parameters according to our kobuki, learn how to use the transforms and change the base_footprint to base_link due to problems with the update of the repository used to use the transforms, and change the base_footprint to base_link due to problems with the update of the repository used to use the transforms.

## 0.Introduction
Through this practice we learn how to use the transforms from the ROS tf packages, as well as how to create and organize our code with behaviour trees. In addition, we also make use of the PID controllers for a correct and fluid operation when performing movements in kobuki.

This is because this practice is based on finding people through the image obtained from the camera and then proceeding to try to reach that person, similar to the game seek and capture.

## Transforms
tf is a package that lets the user keep track of multiple coordinate frames over time. tf maintains the relationship between coordinate frames in a tree structure buffered in time, and lets the user transform points, vectors, etc between any two coordinate frames at any desired point in time.

A robotic system typically has many 3D coordinate frames that change over time, such as a world frame, base frame, gripper frame, head frame, etc. tf keeps track of all these frames over time, and allows you to ask questions like:

- Where was the head frame relative to the world frame, 5 seconds ago?
- What is the pose of theBehavior Trees use the concept of Tick, a signal that is sent in a fixed frequency, which starts from the root node and is propagated to its children. By doing this Behavior Trees can react in real-time to events that happen in the world. object in my gripper relative to my base?
- What is the current pose of the base frame in the map frame?

An example of what it is seen whith the transforms on the rviz app is this:

![Screenshot from 2023-03-27 18-23-41](https://user-images.githubusercontent.com/92941166/228033121-16a64121-a7ba-4714-81a8-00210c897faa.png)

## Behaviour Trees
A behavior tree (BT) is a mathematical model of plan execution used in computer science, robotics, control systems and video games. They describe switchings between a finite set of tasks in a modular fashion. Their strength comes from their ability to create very complex tasks composed of simple tasks, without worrying how the simple tasks are implemented. BTs present some similarities to hierarchical state machines with the key difference that the main building block of a behavior is a task rather than a state.

Behavior Trees use the concept of Tick, a signal that is sent in a fixed frequency, which starts from the root node and is propagated to its children. By doing this Behavior Trees can react in real-time to events that happen in the world.

The following image is the behaviour trees that we have designed:

![Captura desde 2023-03-27 10-38-51](https://user-images.githubusercontent.com/92941166/228030689-f5fff147-7fa4-46f4-9c4c-720e2b7e8a1b.png)

## Goals

- [X] **Detecting a person**
- [X] **Chase a person**
- [X] **Indicate that a person has been found**
- [X] **Colcon test passed**
- [ ] **Implement Behaviour Tree**
- [ ] **Detect more than one person**

## Code

The most important codes are the next ones, wich are used to do the transforms and to calculate the distance and the angle needed. It also has the PID values we use. This values are the general values which can work in almost every kobuki without to much oscillation.

* [TfDetectionNode.cpp](https://github.com/Docencia-fmrico/seekandcapture-cero_dumped/blob/main/src/seekandcapture/TfDetectionNode.cpp)

```cpp
float len = std::size(msg->detections);

  for (int i = 0; i < len; i++) {

    if (msg->detections[i].results[0].hypothesis.class_id == "person") {
    
      float x=msg->detections[i].bbox.center.position.x,
            y=msg->detections[i].bbox.center.position.y,
            z=msg->detections[i].bbox.center.position.z;

      camera2obj.setOrigin(tf2::Vector3(x, y, z));
      try {
        odom2camera_msg = tf_buffer_.lookupTransform(
          "odom", "camera_depth_optical_frame",
          tf2::timeFromSec(rclcpp::Time(msg->header.stamp).seconds()));
        tf2::fromMsg(odom2camera_msg, odom2camera);
      } catch (tf2::TransformException & ex) {
        RCLCPP_WARN(get_logger(), "Obstacle transform not found: %s", ex.what());
        return;
      }
      
      tf2::Transform odom2object = odom2camera * camera2obj;

```
* [MoveToTransformNode.cpp](https://github.com/Docencia-fmrico/seekandcapture-cero_dumped/blob/main/src/seekandcapture/MovefromTransformNode.cpp)

```cpp
  lin_pid_(0.0, 5.0, 0.0, 0.3),
  ang_pid_(0.0, M_PI/2, 0.5, 1.2),
  
  [...]
  
  lin_pid_.set_pid(0.6, 0.05, 0.35);
  ang_pid_.set_pid(0.6, 0.08, 0.32);

  [...]
   
  try {
        person_msg = tf_buffer_.lookupTransform(
          "base_link", "person",
          tf2::timeFromSec(rclcpp::Time(person_msg.header.stamp).seconds()));
          

        } catch (tf2::TransformException & ex) {
            RCLCPP_WARN(get_logger(), "Obstacle transform not found: %s", ex.what());
            return;
        }
  const auto & x = person_msg.transform.translation.x;
  const auto & y = person_msg.transform.translation.y;


  double angulo = atan2(y, x);
  double distancia = sqrt(pow(x,2) + pow(y,2));

  if (distancia <= 0.9) { 
      vel.linear.x = 0;
      vel.angular.z = 0;
      vel_pub_->publish(vel);
      out_sound.value = kobuki_ros_interfaces::msg::Sound::ON;
      sound_pub_->publish(out_sound);
      out_led.value = GREEN;
      led_pub_->publish(out_led);

      return;      
  [...]
  
```

## Install

Steps to install and run the package:

1. Clone the git repository in a workspace
2. Import the third-party repos of the package using the next comand in the src directory of the workspace:
```bash 
vcs import < seekandcapture-cero_dumped/third_parties.repos
```
3. Compile the workspace
```bash
colcon build --symlink-install --packacges-select seekandcapture
```
4. Run the package with this command:

```bash
ros2 launch seekandcapture perception.launch

``` 
> The [ir_robots](https://github.com/IntelligentRoboticsLabs/ir_robots) repository is needed to launch the kobuki.

## Package final demostration

https://user-images.githubusercontent.com/113594702/228205358-783f0345-514a-4040-8379-3bb2dd796b87.mp4

