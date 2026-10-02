# Jackal UGV — ROS Melodic Setup & Control Scripts

Notes and scripts for controlling a Clearpath **Jackal UGV** over **ROS Melodic**, including
a workaround for running **Python 3** rospy nodes on Melodic (which ships targeting Python 2).

## Repository structure

```
Jackal_project-main/
└── src/
    ├── jackal_controll.py   # Jackal class: ROS node wrapping velocity control + odometry feedback
    ├── return_to_zero.py    # Example script: drives the robot back to (0, 0) using Jackal class
    └── tools.py             # Math helpers (pose error, trajectory generation) used by control scripts
```

There is no `catkin` package here (no `package.xml` / `CMakeLists.txt`) — these are plain Python
scripts that talk to the Jackal's ROS master over the network using `rospy`, so no local catkin
workspace/build is required to run them.

## 1. Prerequisites

- A Jackal running **Ubuntu 18.04 + ROS Melodic** (Clearpath's factory image, or a manual install).
- A workstation/laptop on the same network as the Jackal (or SSH'd into the Jackal directly).
- `python3` installed on whichever machine runs these scripts.

## 2. Installing ROS Melodic (Ubuntu 18.04)

Only needed on a machine that doesn't already have ROS Melodic (the Jackal's factory image
usually ships with it preinstalled — check with `rosversion -d` before redoing this).

### 2.1 Add the ROS apt repository and signing key

The signing key ROS's official install instructions used to reference (via `apt-key adv
--recv-key ...`) has since expired / `apt-key` itself is deprecated, so `apt update` will fail
with a `NO_PUBKEY` / `EXPKEYSIG` error if you follow older guides verbatim. Use the current
keyring-based method instead:

```bash
sudo apt-get install curl gnupg lsb-release

# Fetch the current ROS signing key into a dedicated keyring file
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

# Register the repo, pinned to that keyring (avoids deprecated apt-key)
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros/ubuntu $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/ros1-latest.list
```

If you have an old `/etc/apt/sources.list.d/ros-latest.list` from a previous `apt-key`-based
install, remove it first (`sudo rm /etc/apt/sources.list.d/ros-latest.list`) so the repo isn't
registered twice.

### 2.2 Install ROS Melodic + Jackal packages

```bash
sudo apt-get update
sudo apt-get install ros-melodic-desktop-full
sudo apt-get install ros-melodic-jackal-simulator ros-melodic-jackal-desktop ros-melodic-jackal-navigation
```

### 2.3 Environment + rosdep setup

```bash
sudo rosdep init
rosdep update

echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc

sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential
```

Verify with `roscore` (should start without errors) and `rosversion -d` (should print `melodic`).

## 3. Networking: connecting to the Jackal

If you're running scripts from your own laptop instead of on the robot itself, point ROS at the
Jackal's onboard master instead of your own:

```bash
export ROS_MASTER_URI=http://<jackal-hostname-or-ip>:11311
export ROS_HOSTNAME=<your-machine-ip>
```

Add these to `~/.bashrc` (or a small `setup_jackal.bash` you `source`) so every new terminal picks
them up automatically. Verify connectivity with `rostopic list` — you should see the Jackal's
topics (e.g. `/odometry/filtered`, `/jackal_velocity_controller/cmd_vel`).

## 4. Running Python 3 rospy nodes on ROS Melodic

Melodic's ROS packages are built and distributed for **Python 2**, so `python3 my_node.py` will
typically fail with `ModuleNotFoundError: No module named 'rospkg'` (or `catkin_pkg`, `yaml`, etc.)
even though `rospy` itself imports fine. `rospy` doesn't require a Python-3-specific ROS build for
plain publisher/subscriber nodes — it just needs a handful of supporting packages available under
Python 3:

```bash
sudo apt-get install python3-pip python3-yaml
sudo pip3 install rospkg catkin_pkg
```

After that, running any node in this repo with `python3` (instead of the default `python`/`python2`)
works normally, e.g.:

```bash
python3 src/return_to_zero.py
```

This is sufficient for scripts like the ones here that only use core `rospy` message types
(`geometry_msgs`, `nav_msgs`). It is **not** enough if you need Python 3 builds of packages that
ship compiled/generated code for `tf`, `cv_bridge`, etc. — those need to be rebuilt from source
against Python 3 in a catkin workspace, which is a separate, heavier process.

## 5. Code overview

### `jackal_controll.py` — `Jackal` class

Wraps the ROS boilerplate for controlling the robot:

- Initializes a `jackal_controller` node.
- Publishes `Twist` commands to `/jackal_velocity_controller/cmd_vel`.
- Subscribes to `/odometry/filtered` and tracks position (`getPosition`), heading (`getTheta`),
  and the rotation matrix (`getRotationMatrix`), including a simple angle-unwrapping scheme for
  continuous angular velocity across the ±π crossing.

Key methods:

| Method | Description |
|---|---|
| `Jackal(freq)` | Construct and block until the first odometry message arrives |
| `getPosition()` | Current `[x, y, 0]` in the inertial frame |
| `getTheta()` | Current heading (rad) |
| `getRotationMatrix()` | Rotation matrix of the robot frame relative to the inertial frame |
| `setLinearSpeed(v)` / `setAngularSpeed(w)` | Stage a velocity command |
| `setRobotSpeed()` | Publish the staged `Twist` command |
| `getRate()` | `rospy.Rate` object at the configured frequency |
| `calcLinearVel()` / `calcAngularVel()` | Finite-difference velocity estimates from odometry |

### `tools.py` — control math helpers

- `getError(p, pd, R)` — position error `pd - p`, expressed in the robot's frame.
- `getLinearError(e)` / `getAngularError(e)` — extract linear/angular components of the error
  for a simple proportional controller.
- `getFifthOrder(...)` — quintic (5th-order) trajectory interpolation (position/velocity/
  acceleration at time `t`) for smooth point-to-point motion.

### `return_to_zero.py` — example usage

A minimal proportional controller built on `Jackal` + `tools.py`: drives the robot from its
current pose back to `(0, 0)`, stopping once the position error drops below `0.001`.

Run it with:

```bash
python3 src/return_to_zero.py
```

## 6. Writing your own control script

```python
from jackal_controll import Jackal

robot = Jackal(10)          # 10 Hz control loop
rate = robot.getRate()

while not rospy.is_shutdown():
    robot.setLinearSpeed(0.2)
    robot.setAngularSpeed(0.0)
    robot.setRobotSpeed()
    rate.sleep()
```

## References

- [How to setup ROS with Python 3 (Medium)](https://medium.com/@beta_b0t/how-to-setup-ros-with-python-3-44a69ca36674)
- [Is a Python 3 Subscriber Node in ROS Melodic Simple to Make? (ROS Answers)](https://answers.ros.org/question/374758/is-a-python-3-subscriber-node-in-ros-melodic-simple-to-make)
- [Clearpath Jackal Tutorials](https://docs.clearpathrobotics.com/docs_robots/legacy/ros1_robots/outdoor_robots/jackal/tutorials_jackal/)
- [Setting Up Jackal's Network (Clearpath, Melodic)](https://www.clearpathrobotics.com/assets/guides/melodic/jackal/network.html)
- [How to fix ROS package repo signature verification error (VarHowto)](https://varhowto.com/fix-ros-package-repo-signature-verification-error/)
- [ROS GPG Key Expiration Incident (ROS Discourse)](https://discourse.openrobotics.org/t/ros-gpg-key-expiration-incident/20669)
