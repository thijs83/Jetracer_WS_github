# Jetracer_WS_github
This repo contains basic code to run the jetracers for the hackathon.

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
Where $x,y,\eta,v$ are respectively the x-y position,orientation and longitudinal velocity. $l$ is the length of the Jetracer. The mapping from the steering input $\sigma$ to the steering angle $\delta$ [rad] has been obtained experimentally and is specific for each Jetracer. For convenience the function steer_angle_2_command(steer_angle,car_number) in Jetracer_WS_github/src/lane_following_controller_pkg/src mappes the desired steering angle $\delta$ into the required steering input $\delta$ for each vehicle.



and the acceleration function $f(\tau,v)$ . For 





