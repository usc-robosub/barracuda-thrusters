# barracuda-thrusters
Dockerized barracuda_thrusters ROS node:
- Subscribes to topics named thrusters/inputi for i in {0, ..., 7}; each thruster has an associated topic
    - The value in the message published to thrusters/inputi represents the force in units of Newtons for thruster i to apply
    - The message type is std_msgs/Float32

- Sends these force values to the Teensys via I2C
    - There are 8 thrusters total; 4 thrusters are connected to the PWM pins on one Teensy, and the other 4 are connected to the PWM pins on the other Teensy. Each Teensy target defines I2C registers 0, 4, 8, 12 for each of the four thrusters it’s responsible for generating PWM signals for, and each Teensy has a different I2C address (0x2d, 0x2e)
    - The barracuda_thrusters node sends messages (force values) for thrusters 0-3 to the four registers at the first i2c address (0x2d), and messages for thrusters 4-7 at the four registers at the second i2c address (0x2e)

- Enables and disables the Teensys based on killswitch input signal
    - When the latch closes, the killswitch signal goes hi, and this node sets the "killed" register on the Teensys to '1' using teensy.py module write function
    - When the latch opens, the killswitch signal goes lo, and the "killed" register on the Teensys is set to '0'

- Provides testing scripts
    - ```start_thruster <i>```, ```stop_thruster <i> ```, where i is 0-7, or "all"
    - get into the docker container on the pi to run these:
        - ```docker compose down && docker compose up -d --build```
        - ``` docker exec -it barracuda-thrusters bash```
        - ```start_thruster <i> ``` or ```stop_thruster <i> ```

### RPi Setup Instructions
1. Use [Raspberry Pi Imager](https://www.raspberrypi.com/software/) to install Raspberry Pi OS (64-bit Bookworm) on a microSD card. Make sure that you set it up with a network that your development machine will also be able to connect to. See [this article](https://www.thedigitalpictureframe.com/how-to-add-a-second-wifi-network-to-your-raspberry-pi/) to add a new network if the OS is already installed.
2. Install Git with ```sudo apt-get install git```.
3. [Install Docker Engine](https://docs.docker.com/engine/install/debian/), then follow [these instructions](https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user) so that you can use Docker without sudo (easiest way to do this is with the convenience script). 
4. [Enable I2C interface](https://learn.adafruit.com/adafruits-raspberry-pi-lesson-4-gpio-setup/configuring-i2c).

### Developing Directly on Your Own Pi (VS Code)
1. Install the following VS Code extensions: 
    - [Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)
    - [Remote - SSH](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-ssh)
2. Connect the current window to the RPi in VS Code with Remote Explorer.
3. Run ```git config --global user.name "yourname"``` and ```git config --global user.email "youremail"```.
4. Run ```ssh-keygen``` to create an ssh key pair, then [add the public key to your GitHub account](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account).
5. Clone this repo into a location of your choice on the RPi (using the ssh link), then build the Docker image and run the container with ```docker compose up -d```.
6. While in the remote window connected to the RPi, open the Remote Explorer extension again and select "Dev Containers" from the dropdown menu at the topc of the screen, to the right of the "Remote Explorer" text. The container you just ran should show up; attach to the container in a window. 
7. Now you should be able to edit files, commit & push changes, and interact with the shell in the integrated terminal. 
