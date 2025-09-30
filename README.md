# barracuda-thrusters
Dockerized barracuda_thrusters ROS node:
- Subscribes to topics named thrusters/inputi for i in {0, ..., 7} - each thruster has an associated topic
    - The value in the message published to thrusters/inputi represents the force in units of Newtons for thruster i to apply
    - The message type is std_msgs/Float32
- Translates the force values in Newtons into appropriate values to be outputted on the PWM pins on the Teensys that are connected to the thrusters
    - Convert force in N to force in kgF
    - Used the performance chart for 18V to get pulse width in microseconds needed to apply desired force
        - Used linear interpolation to get pulse width for kgF values between two kgF values on the chart
    - Used PWM frequency f=333Hz (T = 3003.003us) and bit resolution 8 bits to obtain a value n between 0 and 256 that would yield the desired PWM width, where the PWM width in microseconds evaluates to: T * (n/(bit resolution))
        - The PWM frequency and bit resolution are both configurable on the Teensy. Currently, these configuration values are set independently in the barracuda_thrusters node (in the "F2PWM" force-to-pwm module) and on the Teensys, and the node verifies that its stored config values are consistent with the ones stored on the Teensy. Though we could have set these values on only one "source of truth" (either in the node code or in the Teensy code), and then had them set on the other device via serial based on the value set on the "source of truth," the current approach allows us to avoid synchronization logic to make sure that the values are set on the non-"source of truth" before starting regular operation. 
- Sends these PWM values to the Teensys via I2C
    - There are 8 thrusters total; 4 thrusters are connected to the PWM pins on one Teensy, and the other 4 are connected to the PWM pins on the other Teensy. Each Teensy target defines I2C registers 0, 1, 2, 3 for each of the four thrusters it’s responsible for generating PWM signals for, and each Teensy has a different I2C address (0x2d, 0x2e)
    - The thruster controller node maintains a mapping of each thruster i (same i as in the thruster topic thrusters/i/input) to the appropriate (I2C address, register number) pair. Potential future work: store this mapping with ROS parameters instead of directly in the script
    - This subscriber node sends the desired PWM value as described earlier to the appropriate register on the appropriate Teensy upon receipt of a message published to thrusters/i/input - this means that the frequency of writes to I2C registers is dependent on the publishing frequency to the thruster input topics and network latency

### RPi Setup Instructions
1. Use [Raspberry Pi Imager](https://www.raspberrypi.com/software/) to install Raspberry Pi OS (64-bit Bookworm) on a microSD card. Make sure that you set it up with a network that your development machine will also be able to connect to. See [this article](https://www.thedigitalpictureframe.com/how-to-add-a-second-wifi-network-to-your-raspberry-pi/) to add a new network if the OS is already installed.
2. Install Git with ```sudo apt-get install git```.
3. [Install Docker Engine](https://docs.docker.com/engine/install/debian/), then follow [these instructions](https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user) so that you can use Docker without sudo. 
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
