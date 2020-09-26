# CYBORG ROBOT SIMULATION TASKS 🤖
![](./images/logo.jpg)

## 📌 Prerequisites
- Ubuntu 16.04 or newer (Ubuntu 18.04 recommended)
- [ROS Kinetic ](http://wiki.ros.org/kinetic/Installation/Ubuntu) (Ubuntu 16.04) or [ROS Melodic ](http://wiki.ros.org/melodic/Installation/Ubuntu) (Ubuntu 16.04)
- [Catkin Tools](https://catkin-tools.readthedocs.io/en/latest/installing.html)

## ⭐ Before Cloning

Create a ROS workspace and a src folder, then run catkin build
Note: use only small alphabets without spaces, use underscore instead.
```sh
$ mkdir <workspace_name>
$ cd <workspace_name>
$ mkdir src
$ catkin build
```
No need to source the workspace everytime, run this command to source the workspace whenever you open a new terminal
```sh
$ echo 'source ~/hector_ws/devel/setup.bash' >> ~/.bashrc
```
## 💻 Clone
Clone this repository inside src folder
```sh
$ cd src
$ git clone https://github.com/CYBORG-NIT-ROURKELA/robot_simulation.git
```

## 💥 Build
Build all the packages with
```sh
$ cd ..
$ catkin build
```
or build specific packages with
```sh
$ cd ..
$ catkin build [package_name]
```

## 🤝 Contributors 

Well done all the talented members for completing the task ✨

<!-- ALL-CONTRIBUTORS-LIST:START - Do not remove or modify this section -->
<!-- prettier-ignore-start -->
<!-- markdownlint-disable -->
<table>
  <tr>
    <td align="center"> 
      <a href="https://github.com/arnabGudu">
        <img src="https://avatars3.githubusercontent.com/u/42674892?s=460&u=c9e8c0d1fcfc439b0cf0fba6d82f4142573e12e4&v=4" width="100px;" alt=""/><br />
      </a>
      <sub><b>Arnab Paikaray</b></sub>
      <a href="https://github.com/CYBORG-NIT-ROURKELA/robot_simulation/pulls?q=is%3Apr+reviewed-by%3AarnabGudu" title="Reviewed Pull Requests">👀</a>
      <a href="https://github.com/CYBORG-NIT-ROURKELA/robot_simulation/commits?author=arnabGudu" title="Code">💻</a>
    </td>

  </tr>
</table>

## ❤️ Project Admin

|                                     <a href="https://github.com/arnabGudu"><img src="https://avatars3.githubusercontent.com/u/42674892?s=460&u=c9e8c0d1fcfc439b0cf0fba6d82f4142573e12e4&v=4" width=150px height=150px /></a>                                      |
| :-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------: |
|                                                                                      <b>Arnab Paikaray</b>


License
----

MIT

