I've done a fair bit of thinking on where I think I should go with this project, and so I'm gathering here my thoughts on what techniques I plan to use in each level of the stack.

#### Drivers

First, I need to be sure that I'm getting all the data I'll need for subsequent steps. This will be the easiest step, and is already mostly done. I want to be able to teleop the car using only these methods, and watching visualizations from these topics, before I proceed.

1. Respond to [`AckermannDrive` messages](http://docs.ros.org/api/ackermann_msgs/html/msg/AckermannDrive.html), minimally responding to the `steering_angle` and `speed` fields. Since the [Pololu Micro Maestro](https://www.pololu.com/product/1351) servo controller I'm using to communicate with the car's ESC actually supports ramps with specified speed, a possible stretch goal would be to do something intelligent with the `steering_angle_velocity`, `acceleration`, and `jerk` fields.
   For now, it will be enough to call Pololu's provided `UscCmd` program with `os.system`  in our ROS node to set the speed and angle, but their serial protocol is [pretty well documented](https://www.pololu.com/docs/0J40/5.c) (summarized for my use [here](https://github.com/tsbertalan/gudrun/commit/d26840303cfa8fac44f7768aadbfb18fda8f496b)) so I could avoid the repeated subprocess by sending commands by serial directly.
2. Use the [`openni_*`](http://wiki.ros.org/openni_camera) packages to publish RGBD information. (tested and working)
3. Use [`kinect_aux`](http://wiki.ros.org/kinect_aux) to publish the Kinect's IMU information as a `sensor_msgs/Imu` topic. (not yet tested)

#### Perception, sensor fusion, and mapping

When I have some confidence that I as a human can drive the car using the same sensor data and command topics that I'll be providing to higher-level packages, I can turn to replacing my intuition-based sensor fusion to something more concrete.

1. Use something to fake odometry from the Kinect's IMU data. I think that this may be possible via one of the nodes in [`robot_localization`](http://docs.ros.org/melodic/api/robot_localization/html/index.html#). This is the part I'm most uncertain about. However, since this is superficially a pretty simple task (some interesting notes [here](http://www.seattlerobotics.org/encoder/200610/Article3/IMU%20Odometry,%20by%20David%20Anderson.htm); it might be worthwhile to include our actual motor commands in this reckoning, but for that I'll have to sit down with pencil and paper and work out some (E/U)KF scheme), I might just write my own code to make a best-effort IMU odometer. I've seen gmapping fix some pretty egregious wheel-encoder odometry errors in the past, so I think this will be just *fiiiine*.
2. There seem to be many libraries available that will perform SLAM on RGBD or 3D point cloud data. However, after searching for a couple hours, I'm actually having difficulty finding one that will install on ROS Melodic. So, I think I'll take the simpler approach of throwing away most of the RGBD data and instead using one slice as a laserscan via the melodic-available [`depthimage_to_laserscan`](http://wiki.ros.org/depthimage_to_laserscan).
   So, I'll probably  just use this and gmapping. KISS.

#### Speed control

I have some doubts here. In [previous work](http://tomsb.net/Gunnar/), I had a tight control loop running on an Arduino to maintain commanded motor speeds. Here, I don't have wheel encoders, so the best I might be able to do is have a loose loop between my (likely very poor) odometry from the perception phase to my motor command interface in the driver phase.

The more I think about this, the more I think that adding some sort of wheel encoders would greatly simplify many other parts of the design. Maybe I should do that Hall-effect sensor side project after all.

#### Planning

Once I've can watch the map being generated as I teleop around, I can set up a medium-level planning stack.

1. [`teb_local_planner`](http://wiki.ros.org/teb_local_planner) seems to be the way to go for what they call a local planner. However, the videos there suggest that this planner is capable of fairly advanced maneuvering, with multi-point turns, and it seems that it does consider multiple topologically distinct local plans. 
2. However, TEB does require a `nav_msgs/Path` global plan, which can be create simply with [`global_planner`](http://wiki.ros.org/global_planner?distro=melodic#Published_Topics) (that is, A* or Djikstra, or some clever smoothed, interpolated combination) from the standard [navigation stack](http://wiki.ros.org/navigation).

#### Behaviors!

This is the more fun, conceptual part. As suggested by the TEB docs, I'll need to disable some of the low-leve behaviors that come with the navigation stack (like the spin and clear). Witness instead the simple [`Behavior` superclass](https://github.com/tsbertalan/gudrun/blob/a8cc65c6957c3498b119dbe4b59c7455c75a2977/src/gudrun_motor/ultrasound_bump_drive.py#L53) I made, without much thought, for the little bump-drive script I made last week. But, really, this section is more about high-level behaviors. I have a few ideas, some of which require more hardware/software additions than others:

* Wander around, and identify and catalog objects. Fairly easy:
  1.  Set random navigation objectives (perhaps with some [frontier exploration](http://wiki.ros.org/frontier_exploration) strategy, though that's pretty optional).
  2. Wander around, take photographs annotated with current poses (and therefore, ideally, the photographed objects position--the RGBD might help a lot with this).
  3. Pass them to some pre-trained object recognition neural network (I have a [Google Edge TPU](https://coral.withgoogle.com/) I want to try to use for this), and note any high-confidence hits.
  4. Assemble a database from these and make a nice frontend for querying it. Maybe a visual menu, and and for each item, there'd be a "take me there" button. The robot would drive to the remembered pose (including orientation), and then (stretch goal!), use a pair of servos to direct a laser pointer at the remembered position of the object (or even redetect it live and point to the object's updated location).
* Patrol. In addition to or in place of the previous goal, we'd explore as far as possible (e.g., a whole apartment), and then continuously revisit the areas we saw the least recently. A nice trick would be to do some sort of anomaly detection. Somehow featureize all the views of the apartment (probably just camera+pose) and then continuously do some unsupervised learning to detect when these features go away from the typical. Obviously, the border of the "typical" region in feature space will become better-characterized as we gather more data, and there will be lots of false positives at first (it's dark! this is unfamiliar! I'm afraid!).
* Recharge. In addition to either or both of the previous, it would be great if Gudrun could recharge herself. I have had some thoughts on this, but put it on hold as overcomplicated and not necessary for now. Basically, though, my thoughts are divided between
  1. loading onto the underdeck a pair of simple NiMH and Li-Ion (balancing) battery chargers designed for standalone use, and then using a complex arrangement of transistors, relays, and ADC voltage sensing to flip power over suddenly from battery to an external bumper; and
  2. wrapping the batteries in some [battery management system](https://hobbyking.com/en_us/6s-li-ion-10a-pcm.html), (preferably with balancing capability), and trying to avoid letting the batteries ever get so low that they need a proper "charger". Basically, float-charging them at below nominal voltage. I have a couple of ~5V solar panels that might contribute here, especially if I do ...
* Outdoor path following. Here, the more focused goals of above might be discarded in place of just being able to navigate reliably across campus. However, an issue with this is that the view distances are typically *much* longer than in indoor scenes, and so path planning is very different. However, in a way, there's more opportunity for creativity here, since it's less geometric and special-senor based, and more computer vision. Really, though, since this would involve significant changes to the above stack, it would almost be separate project. But, as the weather gets warmer, this might beckon more strongly...