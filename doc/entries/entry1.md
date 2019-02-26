#### Ultrasound for reactive obstacle avoidance
I added two ultrasound sensors looking off to the left and right 
on the front bumper. An arduino dumps these into a serial terminal at a minimum
of about 5Hz, usually faster), and a ROS node reads these and puts them into two
[`sensor_msgs`](http://wiki.ros.org/sensor_msgs)`.msg.Range` topics.

I implemented two behaviors--a back-up-and-turn, 
where we try to go in the direction where there's more space,
and a go-forward, where the steering fraction \\(\in(-1,1)\\)
is computed like
$$\tanh\left( (d_r - d_l) \cdot \lambda \right)$$
and this noisy output is passed through a 10-entry rolling mean filter
(using [`collections.deque`](https://docs.python.org/2/library/collections.html#collections.deque)!).

The result is an illusion of path planning! But it's still really just reactive.

![first video](1_small.gif)
![second video](2_small.gif)

#### In other news

I've tested the [`openni_*`](http://wiki.ros.org/openni_launch) ROS packages, and found that they produce a surfeit
of depth-camera topics from my first-generation Kinect sensor (the power cable
of which I lopped off and replaced with a barrel connector to my pre-ATX 12V rail).
No accelerometer data, though--it would be nice not to have to add a separate IMU,
and instead just use the one that's in the Kinect. 
I think the [`kinect_aux`](http://wiki.ros.org/kinect_aux) package will get this for me.

I hope I can fake "odometry" from this IMU data,
and so obviate the need for separate wheel encoders.
However, if I need them, my current plan is to glue half a dozen tiny magnets
regularly spaced around the inside of the back wheels,
and position a [Hall-effect sensor](https://www.sparkfun.com/products/14709)
nearby.
But I'd rather not have to make another mini-project out of getting that little
Arduino-project working properly, reading my poor-man's grey code.
Integrating an IMU in (non-embedded) software would be easier.

As for real planning, I still have some reading to do 
to figure out what's available already-written
for Ackermann-kinematics robots.
I've seen [TEB local planner](http://wiki.ros.org/teb_local_planner)
used by others; I'm not sure that this would work with the same global planners
used in the gmapping stack, since some maneuvers, like N-point turns,
are fundamentally different between Ackermann and differential-drive kinematics.
I'm not above writing my own planning software
(actually, writing a quick and dirty pair of MPC-local + tree-based-global
would be a worthwhile endeavor,
and maybe not *too* much harder than getting existing packages installed,
tuned, and working smoothly),
but first I need at least write a motor controller
that react to [`ackermann_msgs`](http://wiki.ros.org/ackermann_msgs)`.msgs.AckermannDrive` messages properly.

And, of course, getting SLAM working with the Kinect 
is a whole separate miniproject.

