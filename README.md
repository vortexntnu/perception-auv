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

## Dependencies
This project requires additional repositories, which are listed in the [dependencies.repos](dependencies.repos) file.

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
