# pid_controller

- PID Controller control machine under Robot Operating System(ROS).

- OS: Linux Ubuntu.

- Software: Robot Operating System.

- Publisher:

  - /PIDController_output (PID Control calculate result.)

- Subscriber:

  - /setpoint (System Control command input)

  - /output (System Control output result)

- Run the pid_controller.

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

![image](https://github.com/qaz9517532846/pid_controller/blob/main/image/result.png)

------

This repository is for your reference only. copying, patent application, academic journals are strictly prohibited.

Copyright © 2021 ZM Robotics Software Laboratory.
