# barracuda-thrusters

Dockerized `barracuda_thrusters` ROS 2 Humble node. This package handles the hardware interface for the vehicle's thrusters, converting desired force values from the control module into PWM signals via I2C.

### Core Functionality

- **Subscribes to `/barracuda/cmd_thrust`:** Listens for a `sensor_msgs/JointState` message containing the effort values (in Newtons) for all 8 thrusters in a single array.
- **I2C Communication:** Sends these force values to two Teensy microcontrollers via I2C.
  - 4 thrusters are connected to the PWM pins on the first Teensy (I2C address `0x2d`), utilizing registers 0, 4, 8, and 12.
  - 4 thrusters are connected to the second Teensy (I2C address `0x2e`), utilizing the same registers.
- **Hardware Killswitch:** Enables and disables the Teensys based on a physical GPIO killswitch input.
  - When the latch closes (signal LOW), the node sets the "killed" register (16) on the Teensys to `0`.
  - When the latch opens (signal HIGH), the "killed" register is set to `1`.
- **Dynamic Hardware Mocking:** Built-in fallback architecture (`mock_gpio.py` and `mock_smbus.py`) allows the node to run natively on non-Jetson hardware (like a Mac or Windows PC) without crashing, enabling seamless local development.

### Automated Testing

ROS 2 diagnostic node to verify hardware mapping and I2C communication.

To run the automated thruster test:

1. Exec into the running Docker container: `docker exec -it barracuda-thrusters bash`
2. Run the diagnostic node: `ros2 run barracuda_thrusters test_thrusters`

This node will automatically cycle through thrusters 1-8, sending a temporary test effort value to a single thruster every 2 seconds while keeping the others at 0.0.

### Jetson AGX Setup Instructions (JetPack 6.2)

JetPack 6.2 runs Ubuntu 22.04 and comes with the NVIDIA Container Toolkit natively integrated, making Docker setup straightforward.

1. Ensure your Jetson is connected to a network accessible by your development machine.
2. [Add your user to the Docker group](https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user) so you can run containers without `sudo`: `sudo usermod -aG docker $USER` (requires a logout/login to take effect).
3. Ensure your user has permissions to access the I2C bus: `sudo usermod -aG i2c $USER`.
4. Clone this repository onto the Jetson.

### Running the Containers

This repository uses a dual-file Docker Compose strategy to safely separate local development from hardware deployment.

- **Local Development (Mac/Windows/x86 Linux):**
  - Start the dev environment: `docker compose --profile dev up -d`
  - Access the container: `docker exec -it barracuda-thrusters-dev bash`
  - Stop the dev environment `docker compose --profile dev down --rmi all --remove-orphans`
- **Hardware Deployment (Jetson AGX):**
  - Start the production environment using the NVIDIA runtime override: `docker compose -f docker-compose.yaml -f docker-compose.jetson.yaml up -d`
  - Access the container: `docker exec -it barracuda-thrusters bash`
  - Stop the environment `docker compose down --rmi all --remove-orphans`

### Native ROS 2 Workspace Integration (No Docker)

If you prefer to run this package natively on a Linux machine (without Docker) or integrate it into a larger vehicle control workspace, you can pull it in directly.

#### Option A: Direct Clone

Use this method if you are setting up a standalone ROS 2 workspace just for testing the thrusters natively.

1. Create a new ROS 2 workspace and source directory: `mkdir -p ~/ros2_ws/src`
2. Navigate to the source directory: `cd ~/ros2_ws/src`
3. Clone the repository: `git clone https://github.com/usc-robosub/barracuda-thrusters.git`
4. Navigate back to the workspace root: `cd ~/ros2_ws`
5. Install necessary Python and ROS dependencies: `rosdep install --from-paths src --ignore-src -y`
6. Build the package: `colcon build --symlink-install`
7. Source the newly built workspace: `source install/setup.bash`

#### Option B: As a Git Submodule

Use this method if you already have a primary autonomous vehicle workspace (e.g., `auv_ws`) and want to include the thrusters as a tracked module.

1. Navigate to your existing workspace's source directory: `cd ~/auv_ws/src`
2. Add this repository as a submodule: `git submodule add https://github.com/usc-robosub/barracuda-thrusters.git`
3. Commit the new submodule tracking file: `git commit -m "Added barracuda-thrusters submodule"`
4. Navigate back to your workspace root: `cd ~/auv_ws`
5. Install dependencies and build as usual:
   - `rosdep install --from-paths src --ignore-src -y`
   - `colcon build --symlink-install`

- **Note for other developers cloning the parent repo:** When they pull the main vehicle repository, they will need to run `git submodule update --init --recursive` to fetch the contents of this thruster package.

### Developing Directly on the Jetson (VS Code)

1. Install the following VS Code extensions on your local machine:

- [Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)
- [Remote - SSH](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-ssh)

1. Connect the current window to the Jetson in VS Code using the Remote Explorer (SSH).
2. Run `git config --global user.name "yourname"` and `git config --global user.email "youremail"`.
3. Run `ssh-keygen` to create an SSH key pair, then [add the public key to your GitHub account](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account).
4. While in the remote window connected to the Jetson, open the Remote Explorer extension and select "Dev Containers" from the dropdown menu. Attach to the running `barracuda-thrusters` container.
5. You can now edit files, commit, push changes, and interact with the ROS 2 workspace directly through the integrated terminal.
