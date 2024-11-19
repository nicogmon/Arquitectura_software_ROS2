# ASR-0-AvoidObstacle
Práctica 0 - Arquitecturas Software para Robots 2023

Crea un comportamiento autónomo de un robot usando una máquina de estado finito (FSM) para evitar obstáculos:
1. El robot empieza parado, y comienza su comportamiento cuando se pulsa un botón del robot.
2. El robot avanza hasta encontrar un obstáculo a menos de un metro enfrente de él.
3. Cuando encuentra un obstáculo, el robot gira 90 grados, y realiza un movimiento de arco para sobrepasarlo.
4. Si mientras está haciendo el arco, se encuentra un nuevo obstáculo, vuelve a hacer lo mismo del punto 3.

![asr_practica_0](https://user-images.githubusercontent.com/3810011/217230998-a162f2e1-cf50-4e26-9155-53ca73e99f86.png)

El robot debe funcionar en el robot real Kobuki.

Puntuación (sobre 10):

* +8 correcto funcionamiento en el robot real.
* +2 Readme.md bien documentado con videos.
* -3 Warnings o que no pase los tests.
* +1 Setup de CI/CD


***

##### Fecha: 24/02/2023
##### Grupo: Cero Dumped
##### Autores: [Ana Martínez Albendea](https://github.com/ana-martinezal2021), [Nicolás García Moncho](https://github.com/nicogmon), [Pablo Sánchez Fernandez](https://github.com/psanchezf2021) e [Iker Peral del Pino](https://github.com/iperal2021)
##### Objetivo: Realizar la práctica 0 de la asignatura de Arquitectura Software para Robots
##### Particularidades: Para ajustar tiempo y velocidad del robot para realizar los movimientos, hay que tener en cuenta el suelo y batería del robot.

## 0.Introducción

Para completar esta práctica se ha reutilizado y mejorado el código de muestra Bump and go, disponible en el directorio [*br2_fsm_bumpgo_cpp*](https://github.com/fmrico/book_ros2/tree/main/br2_fsm_bumpgo_cpp) del repositorio [*book_ros2*](https://github.com/fmrico/book_ros2) de [**fmrico**](https://github.com/fmrico). El objetivo a seguir es llevar a cabo los requisitos del enunciado del ejercicio para lo cual hemos diseñado una *Máquina de Estado Finito* la cual es explicada en el siguiente apartado.

Los objetivos a cumplir son los siguientes:

- [x] Se pulsa un boton para iniciar el movimiento.
- [x] Tras detectar un obstaculo gira **90º**.
- [x] Inicia un movimiento en forma de arco para esquivar el obstaculo.
- [x] Si detecta de nuevo un obstaculo vuelve a girar.
- [x] Tras concluir el arco el robot gira de nuevo **90º** y avanza hacia delante.

Además del *lidar*, necesario en la práctica, se han usado funciones propias del robot utilizado tales como botones y leds. Estos sensores y actuadores son usados para iniciar/detener el movimiento del robot y mostrar el estado en el que se encuentra el robot gracias a los distintos colores disponibles. El funcionamiento del robot puede verse en el siguiente conjunto de videos en los que el robot se enfrenta a un obstaculo y a dos, respectivamente:


1. Un solo obstaculo:


https://user-images.githubusercontent.com/113594702/220981257-a4c4b866-8387-4d1c-9a73-f908cab1f817.mp4


***

2. Dos obstaculos:


https://user-images.githubusercontent.com/113594702/220981287-c62e36ec-abae-4129-a6d5-19b0b025c034.mp4


***

3. Dos obstaculos con funcionamiento completo:


https://user-images.githubusercontent.com/113594702/221004171-a0fe95ec-37a3-4ffc-81af-2aa003f677f7.mp4


***
Para correr el paquete hay que introducir los siguientes comandos:

1. Primero lanzar el kobuki con el siguiente comando desde el directorio *launch* del repositorio [*ir_robots*](https://github.com/IntelligentRoboticsLabs/ir_robots) de [*IntelligentRoboticsLab](https://github.com/IntelligentRoboticsLabs):
```bash
ros2 launch kobuki.launch.py

```  

2. Despues en una nueva terminal introducir el siguiente comando:
```bash
ros2 launch avoid_cerodumped_cpp avoid_cerodumped.launch.py

``` 

## 1.Diagrama de Estados 

El diagrama de estados diseñado para la práctica consta de un total de 6 estados:

* **START**: Estado inicial del robot.
* **STOP**: Estado en el cual el robot se encuentra detenido.
* **FORWARD**: Estado de movimiento rectilineo.
* **TURN**: Giro de 90º antes de realizar el arco.
* **TURN2**: Giro de 90º después de realizar el arco.
* **ARCH**: Movimiento de arco usado para esquivar el obstäculo.

El diagrama utilizado tiene la siguiente forma: 

![img_sd](https://github.com/Docencia-fmrico/avoid-obstacle-cero_dumped/blob/main/DiagramaNuevo.jpg)

El funcionamiento del mismo se basa en que una vez pulsado el boton 0 del robot, este inicia su movimiento pasando del estado **START** a **FORWARD**. Una vez se detecta un obstaculo dentro del rango especificado, pasa al estado **TURN**, en el cual gira sobre si mismo 90º. Acto seguido pasa al estado **ARCH** iniciando un movimiento en forma de arco de manera que pueda esquivar el obstaculo y si durante este movimiento vuelve a detectar algún nuevo obstaculo regresa al estado **TURN**. Si no se vuelve a detectar nada, una vez que termina el recorrido del arco pasa a **TURN2**, realizando un nuevo giro de 90º sobre si mismo, y una vez terminado repite le ciclo al volver a **FORWARD**. 

Si en algún momento es necesario parar el movimiento del robot es posible hacerlo pulsando el Boton 1 del robot, el cual reinicia al estado **START**, siendo necesario volver a empezar pulsando el boton 0 en caso de querer reanudar el movimiento. Ademas de esta función, si pasado un tiempo establecido no se ha vuelto a detectar nada con el laser, se pasa al estado STOP, que para todo movimiento del robot.

## 2.Códigos característicos de la práctica

Las diferencias más clara con el código base son las referentes al uso de los ya mencionado botones y leds del robot. Para la implementación de los *publishers* y *subscribers* hemos tomado de ejemplo los ya existentes, siendo estos los referentes al laser y el uso de velocidades.

* Codigo para los botones:
  - AvoidNode.cpp

  ```cpp
  #include "kobuki_ros_interfaces/msg/button_event.hpp"
  
  button_sub_ = create_subscription<kobuki_ros_interfaces::msg::ButtonEvent>(
      "input_button", rclcpp::SensorDataQoS(),
      std::bind(&AvoidNode::button_callback, this, _1));

  void
  AvoidNode::button_callback(kobuki_ros_interfaces::msg::ButtonEvent::UniquePtr button_msg)
  {
    state_button_ = std::move(button_msg);

    if (state_button_->button == 0 && state_button_->state == 1) {
      button_pressed = true;
    }
    if (state_button_->button == 1 && state_button_->state == 1) {
      button_pressed = false;
    }
  }

  ```
  - AvoidNode.hpp
  

  ```cpp
  #include "kobuki_ros_interfaces/msg/button_event.hpp"
  
  void button_callback(kobuki_ros_interfaces::msg::ButtonEvent::UniquePtr button_msg);

  rclcpp::Subscription<kobuki_ros_interfaces::msg::ButtonEvent>::SharedPtr button_sub_;
  kobuki_ros_interfaces::msg::ButtonEvent::UniquePtr state_button_;

  ```
* Codigo para los leds:
  - AvoidNode.cpp
  
  ```cpp 
  led_pub_ = create_publisher<kobuki_ros_interfaces::msg::Led>("output_led", 0);
  
  kobuki_ros_interfaces::msg::Led out_led;
  
  out_led.value = RED;
  out_led.value = GREEN;
  out_led.value = ORANGE;
  out_led.value = BLACK;
  ```
  - AvoidNode.hpp
  ```cpp
  #include "kobuki_ros_interfaces/msg/led.hpp"
  
  rclcpp::Publisher<kobuki_ros_interfaces::msg::Led>::SharedPtr led_pub_;
  ```
