# interim_bot

## i made a little robot over interim, and it needs code.

this is running on a Pi 4 with a little robot that I made. [CAD here](https://cad.onshape.com/documents/1572e870c9acf2bd19b0c4b7/w/9b006baadf794d47302299e4/e/9979f0684349711be933ad49?renderMode=0&uiState=66c6a00f7dc83b64f5144a4a0)

Right now the goal is to be able to drive it around with a bluetooth game controller.
Planning to use ROS2 for a _simple_ but _extensible and not dumb_ architecture.

## Progress
in-process notes for a working engineer

some notes:
- project-management wise, this has gone OK. Motors work, the architecture isn't terrible, and I had a driving machine in 2 iterations
- 25khz seems like the right PWM frequency, I get control of the motor with reasonable accuracy
- try higher frequencies, it seems to slow the motor down
- the motors have low torque at low speeds; the BLDC driver is not a servo driver by any stretch
- `pigpiod` library is far better for writing PWM tasks
  - `RPi.GPIO` is actually a software PWM that ties up the core
  - `pigpiod` needs to be built from source for Ubuntu, but that's fine honestly. took very little time

but there are some issues:
- ~need to add a big flyback across the 12V rail before connecting to a battery. currently the robot browns out during a huge PWM change, and takes 5 min to reconnect to wifi. oops~
  - I'm actually unsure if this is an overcurrent trip or a noise coupling thing. these are both bad with tradeoffs.
  - added the flyback, this has been mostly mitigated for now. it happened once on an 80% -> 10% jump, so maybe a bigger cap would help (at its own tradeoff risks). i'm just gonna fix it in software
- motors can move, but I have no motor class yet
  - that class needs to accepts `forward_rotation_dir`, `pwm_freq_hz`, pinouts, and probably encoder stuff too for each motor
  - probably want a closed-loop speed control to make sure it moves under load. just speed control, no position shenanigans.
- the two motors should be a `Node` with several services for different velocity types
  - simplest is just `speed_left, speed_right, duration` CLI from a computer
  - then we add a service that just modifies the speed with no duration. this assumes that when the controller is connected and the sticks are idle, the controller is sending zeros to the robot.
  - you _could_ mix a joystick input, accepting a magnitude and direction and computing the two wheel velocities required.
  - from a UX point-of-view, the best solution here is probably just passing in two joysticks' y-values (like a skidsteer).
