# pid_controller

- PID Controller control machine under Robot Operating System(ROS).

- OS: Linux Ubuntu.

- Software: Robot Operating Sysyem.

- Publisher:

  - /PIDController_output (PID Control calculate result.)

- Subscriber:

  - /setpoint (System Control command input)

  - /output (System Control output result)

``` bash
$ cd <catkin_ws>/src
```

``` bash
$ git clone https://github.com/qaz9517532846/pid_controller.git
```

``` bash
$ cd ..
```

``` bash
$ catkin_make
```
