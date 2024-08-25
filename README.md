### `path_planner_server`

#### Notes

1. In `DWBLocalPlanner`, "DWB" means **D**ynamic **W**indow **A**pproach. Here is the _abstract_ of the [original paper](https://www.ri.cmu.edu/pub_files/pub1/fox_dieter_1997_1/fox_dieter_1997_1.pdf):
> This paper describes the dynamic window approach to reactive collision avoidance
> for mobile robots equipped with synchro-drives. The approach is derived directly
> from the motion dynamics of the robot and is therefore particularly well-suited for
> robots operating at high speed. It differs from previous approaches in that the search
> for commands controlling the translational and rotational velocity of the robot is
> carried out directly in the space of velocities. The advantage of our approach is that
> it correctly and in an elegant way incorporates the dynamics of the robot. This is done
> by reducing the search space to the _dynamic window_, which consists of the velocities
> reachable within a short time interval. Within the dynamic window the approach only
> considers admissible velocities yielding a trajectory on which the robot is able to stop
> safely. Among these velocities the combination of translational and rotational velocity
> is chosen by maximizing an objective function. The objective function includes a
> measure of progress towards a goal location, the forward velocity of the robot, and
> the distance to the next obstacle on the trajectory. In extensive experiments the
> approach presented here has been found to safely control our mobile robot RHINO
> with speeds of up to 95 cm/sec, in populated and dynamic environments.
2. The reason `DWBLocalPlanner` is called that instead of `DWA...` are not clear and appear to have a historical component.
3. In `bt_navigator`, "bt" stands for **b**ehavior **t**ree.

#### Launching

1. `ros2 launch localization_server localization.launch.py`.
2. `ros2 launch path_planner_server pathplanner.launch.py`.
3. In Rviz2, pick **2D Pose Estimate**. Optionally, move around until localziation converges.
4. In Rviz2, pick **2D Goal Pose**.

#### Path to goal

| Path |
| --- |
| ![1](assets/1.png) |
| ![2](assets/2.png) |
| ![3](assets/3.png) |
| ![4](assets/4.png) |
| ![5](assets/5.png) |
| ![6](assets/6.png) |
| ![7](assets/7.png) |
