# Jetracer_WS_github
This repo contains basic code to run the jetracers for the hackathon.
![4_jetracers_in_a_line_cropped](https://github.com/Lorenzo-Lyons/Jetracer_WS_github/assets/94372990/9bbfd133-cf4d-4b3f-9fe9-1517ebd933cb)



## Installation
Clone this repo 
```
git clone https://github.com/Lorenzo-Lyons/Jetracer_WS_github.git
```

### Create a new branch for your group 
First navigate the newly created Jetracer_WS_github folder and list available remote branches by typing:
```
git branch -r
```
Select the hacakthon branch by typing:
```
git branch hackathon_18_Jan_2024
```
Check the outcome by viewing all local branches by typing:
```
git branch
```
Finally create a new local branch with the initials of the group members, for example:
```
git checkout -b hackathon_18_Jan_2024_ABCD
```

### Build and source the catkin workspace

Re-build the catkin workspace by typinig:
```
catkin clean -y
catkin build
```
Now source the newly created ros workspace by adding the required line in the .bashrc file. Then close and reopen all terminals (also close and reopen VScode) in order for the changes to take effect. (in .bashrc)
```
source PATH_TO_GIT_REPO/Jetracer_WS_github/devel/setup.bash
```
If needed check out the ROS setup tutorial: http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment

## Setup information
Each vehicle is identified by a number set as an environment variable in the .bashrc file located in the home directory. Upon launching a new terminal the "car_number" will be displayed. Many scripts need this value to properly set up topic names. This is needed for running experiments with multiple robots.


## Kinematic bicycle Jetracer Model
The Jetracers are controlled by providing them with throttle and steering inputs, respectively:

```math
\begin{align*}\tau \in [-1,+1]\\\sigma \in [-1,+1]\end{align*}
```
The behaviour of the Jetracer can be modelled with a kinematic bicycle model, assuming that the velocity is below 1.5 m/s:
```math
\begin{align*}
    \begin{bmatrix}\dot{x}\\\dot{y}\\\dot{\eta}\\\dot{v}\end{bmatrix}&=\begin{bmatrix}v\cos{\eta}\\v\sin{\eta}\\\frac{v \tan(\delta(\sigma))}{l}\\f(\tau,v)\end{bmatrix}
\end{align*}
```
Where $x,y,\eta,v$ are respectively the x-y position,orientation and longitudinal velocity. $l$ is the length of the Jetracer. 

The mapping from the steering input $\sigma$ to the steering angle $\delta$ [rad] has been obtained experimentally and is specific for each Jetracer. For convenience the function *steer_angle_2_command(steer_angle,car_number)* in Jetracer_WS_github/src/lane_following_controller_pkg/src/functions_for_controllers.py maps the desired steering angle $\delta$ into the required steering input $\delta$ for each vehicle.

![steering_curve](https://github.com/Lorenzo-Lyons/Jetracer_WS_github/assets/94372990/22f4077f-2ee2-4653-ace0-1a6e59f49850)


The acceleration function $f(\tau,v)$ has also been estimated experimentally. It features a throttle dependent term and a velocity dependent friction term. It is detailed in the fuction *evaluate_Fx_2(vx, th)* in Jetracer_WS_github/src/lane_following_controller_pkg/src/functions_for_controllers.py

![acceleration_curve](https://github.com/Lorenzo-Lyons/Jetracer_WS_github/assets/94372990/b606e87b-93d3-41d6-b527-e71fcd877233)

## Available low level controllers

The package *lane_following_controller_pkg* cointains low level controllers that use the previously described kinematic bicycle model to control the robot. 

**longitudinal_controller.** This controller tracks a reference velocity. To do so simpy publish a reference velocity value to the topic *v_ref_<car_number>*.

The controller works by means of a feedforward-feedback controller defined as:

```math
\begin{align*}
\tau = - K(v-v_{ref}) + \tau^{ff}
\end{align*}
```
Where $K$ is a gain and $\tau^{ff}$ is defined as:
```math
\begin{align*}
\tau^{ff} =\tau\text{  s.t.  } f(\tau,v_{ref})=0
\end{align*}
```

The current vehicle velocity $v$ is provided by an encoder. To start the sensor publishing:

```
rosrun cytron_jetracer publish_sensors_and_inputs_universal.py
```
To start the longitudinal velocity tracking controller run:
```
rosrun lane_following_controller_pkg longitudinal_controller.py
```






