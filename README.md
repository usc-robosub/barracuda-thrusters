# barracuda-thruster-output-controller

### Setup Instructions
1. Clone the repo to an RPi (tested on RPi running 64-bit Bookworm). See [RPi Setup Instructions](#rpi-setup-instructions) to make sure it is set up properly.
2. Build the image and start the container in the background with ```docker compose up -d``` 
3. Enter the container in shell mode with ```docker exec -it thruster-controller bash```
4. From the catkin_ws dir, run ```catkin_make``` to build the barracuda_thruster_output_controller package. Then (still from the catkin_ws dir) run ```source devel/setup.bash```.
5. Connect the SDA & SCL pins on the RPi to the SDA1 & SCL1 pins on the Teensy, and connect a ground pin on the RPi to a ground pin on the Teensy (which should have the thruster target code flashed onto it).
6. Run the test for the thruster controller controller node with ```roslaunch barracuda_thruster_output_controller thruster_controller_test.launch```. This will run the both the thruster controller node, as well as a keyboard testing node that publishes FloatStamped messages to the thruster topics. Press a number 0-7 to change the thruster index, press k to increment and i to decrement force value (N) by 1, press r to reset all thrusters, q to quit. If you can't see what you're typing in the terminal after the keyboard test exits, run ```stty sane```.
7. Exit the container in shell mode with ```exit```, then shut down the container with ```docker compose down```.

### RPi Setup Instructions
1. Use [Raspberry Pi Imager](https://www.raspberrypi.com/software/) to install Raspberry Pi OS (64-bit Bookworm) on a microSD card. Make sure that you set it up with a network that your development machine will also be able to connect to. See [this article](https://www.thedigitalpictureframe.com/how-to-add-a-second-wifi-network-to-your-raspberry-pi/) to add a new network if the OS is already installed.
2. Install Git with ```sudo apt-get install git```.
3. [Install Docker Engine](https://docs.docker.com/engine/install/debian/), then follow [these instructions](https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user) so that you can use Docker without sudo. 
4. [Enable I2C interface](https://learn.adafruit.com/adafruits-raspberry-pi-lesson-4-gpio-setup/configuring-i2c).

### Contributing Setup Instructions (VS Code)
1. Install the following VS Code extensions: 
    - [Dev Containers](catkin_ws/src/barracuda_thruster_output_controller/launch/thruster_controller_test.launch)
    - [Remote Explorer](https://marketplace.visualstudio.com/items?itemName=ms-vscode.remote-explorer)
2. Connect the current window to the RPi in VS Code with Remote Explorer.
3. Run ```git config --global user.name "yourname"``` and ```git config --global user.email "youremail"```.
4. Run ```ssh-keygen``` to create an ssh key pair, then [add the public key to your GitHub account](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account).
5. Clone this repo into a location of your choice on the RPi (using the ssh link), then build the Docker image and run the container with ```docker compose up -d```.
6. While in the remote window connected to the RPi, open the Remote Explorer extension again and select "Dev Containers" from the dropdown menu at the topc of the screen, to the right of the "Remote Explorer" text. The container you just ran should show up; attach to the container in a window. 
7. Now you should be able to edit files, commit & push changes, and interact with the shell in the integrated terminal. 

### Interfacing with the ROS Node
This node subscribes to topics named ```thrusters/i/input``` for i in {0, ..., 7} (each thruster has an associated topic), and the message type published to these topics should be uuv_gazebo_ros_plugins_msgs/FloatStamped (this message type is defined in the uuv_gazebo_ros_plugins_msgs package so you will need to import it if publishing to the thruster topics).

This node does not publish to any topics. 
