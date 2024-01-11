# Tbot2Canonical
A package that transforms the telemetry emitted by turtlesim_node to canonical form.

This node attempts to make the turtlesim_node compatible with nodes that
expect to interact with a canonical robot.

A canonical robot accepts movement commands in the form of Twist messages
on /cmd_vel, and reports its pose and velocity in the form of
Odometry messages on /odom. It also publishes the transform between the
odom frame and the base_link frame on /tf. The turtlebot simulator, turtlesim,
does not conform completely to that convention - it uses a simplified custom
message for odometry. This node transforms the simplified message into
the canonical Odometry message and a tf transform.

