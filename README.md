# Perception AUV
Parent repository including launch files for Perception related packages.

External package dependencies are managed using a `dependencies.repos` file and the ROS-standard `vcs` tool.

## Launch
Main launch command to launch all Perception AUV related packages.
```bash
# launch with --show-args to print out all available launch arguments
ros2 launch perception_setup perception.launch.py
```
See [perception_setup/README.md](perception_setup/README.md) for more information.

## Isaac ROS Developer Environment
Set up your system using the official NVIDIA Isaac ROS Developer Environment guide: https://nvidia-isaac-ros.github.io/v/release-3.2/getting_started/dev_env_setup.html

### Expected workspace structure
Development is assumed to take place inside:

```bash
~/workspaces/isaac_ros-dev/src
```

## Dependencies
This project requires additional repositories, which are listed in the .repos files.

### Install dependencies

Install `vcstool` if it is not already available:

```bash
sudo apt update
sudo apt install -y python3-vcstool
```
Run `vcs import` from the workspace `src/` directory. This assumes the
`perception-auv` repository has already been cloned into `src/`.

Using **HTTPS**:
```bash
vcs import < perception-auv/dependencies.https.repos
```

Using **SSH**:
```bash
vcs import < perception-auv/dependencies.ssh.repos
```

### Development Environment (Docker)
**Prerequisite:** All repository dependencies must be installed (see [Dependencies](#dependencies)).

From the workspace root:
```bash
./src/isaac_ros_common/scripts/run_dev_rosdep.sh
```
This starts the Isaac ROS Docker dev environment.

If you are offline or want to skip rebuilding the Docker image:
```bash
./src/isaac_ros_common/scripts/run_dev_rosdep.sh --skip_image_build
```

### Host SDK Access (Docker)

Host-installed SDKs (e.g. ZED, Spinnaker) are made available to the Docker
development container via Docker arguments defined in the
[isaac_ros_common](https://github.com/vortexntnu/isaac_ros_common) repository:

`isaac_ros_common/scripts/.isaac_ros_dev-dockerargs`

This file specifies required volume mounts, device access, and environment
variables. Update it as needed to expose additional SDKs or hardware before
starting the Docker environment.

See the Isaac ROS documentation for more details:
https://nvidia-isaac-ros.github.io/v/release-3.2/concepts/docker_devenv/index.html#development-environment

### Camera configuration
##### Realsense D555
- **Camera IP:** `10.0.0.67`
- **Subnet Mask:** `255.255.255.0`

Ensure your computer is on the same subnet (e.g., `10.0.0.x`, excluding `10.0.0.67`).
