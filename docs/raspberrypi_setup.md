## Setting up Ubuntu server 20.04 on Raspberry Pi 4 

## Step 1: Prepare the SD Card with Raspberry Pi Imager

Make sure you have inserted the microSD card into your computer, and install the Raspberry Pi Imager at your computer.

```shell
sudo snap install rpi-imager
```

Once you have installed Raspberry Pi Imager tool, find and open it and click on the “**CHOOSE OS**” menu.

<img src="https://i0.wp.com/itsfoss.com/wp-content/uploads/2020/09/raspberry-pi-imager.png?resize=800%2C600&ssl=1" alt="Raspberry Pi Imager" style="zoom:50%;" />



Scroll across the menu and click on “Ubuntu” (Core and Server Images).

<img src="https://i1.wp.com/itsfoss.com/wp-content/uploads/2020/09/raspberry-pi-imager-choose-ubuntu.png?resize=800%2C600&ssl=1" alt="Raspberry Pi Imager Choose Ubuntu" style="zoom:50%;" />



For raspberry pi 4, I choose the Ubuntu 20.04 LTS 64 bit.

<img src="https://i0.wp.com/itsfoss.com/wp-content/uploads/2020/09/raspberry-pi-imager-ubuntu-server.png?resize=800%2C600&ssl=1" alt="Raspberry Pi Imager Ubuntu Server" style="zoom:50%;" />

Select your microSD card from the “SD Card” menu, and click on “WRITE”after.

<img src="https://i1.wp.com/itsfoss.com/wp-content/uploads/2020/09/raspberry-pi-imager-sd-card.png?resize=800%2C600&ssl=1" alt="Raspberry Pi Imager Sd Card" style="zoom:50%;" />

## Step 2: Use Ubuntu server on Raspberry Pi (if you have dedicated monitor, keyboard and mouse for Raspberry Pi)

- Install the SD card into the raspberry Pi and start the device
  - Default username: ubuntu
  - Default password: ubuntu
- Right after a successful login, [Ubuntu will ask you to change the default password](https://itsfoss.com/change-password-ubuntu/). 
- Then follow the steps to make the raspberry Pi usable for our project

### Setup Wi-Fi connection for Raspberry Pi

````shell
sudo nano /etc/netplan/50-cloud-init.yaml
````

Add these lines to the file, change the \<IP\> for the desired new node (slave RPi) IP.

````yaml
    wifis:
        wlan0:
          dhcp4: true
          optional: true
          addresses: [<IP>/24]
          gateway4: 192.168.1.1
          nameservers:
              addresses: [192.168.1.1, 8.8.8.8]
          access-points:
              academy:
                  password: "ros-industrial"
````

**note**: Use 4 spaces indentation only

````shell
sudo netplan apply
````

**note**: If there are any indentation use backspace and space to adjust that.

### Setup time zone and set correct time

````shell
sudo timedatectl set-timezone Europe/Berlin
````

### Setup correct keyboard layout for DE

````shell
sudo dpkg-reconfigure keyboard-configuration
sudo setupcon
sudo reboot
````

Use `ip a` to check if the device is connected to the network.

### Install desktop environment for ease of use

````shell
sudo apt install xubuntu-desktop
sudo dpkg-reconfigure lightdm
````

**Note:** If any of the steps does not run as expected remove the power supply and wait for 30 seconds and then start the device again. This clears the RAM and apply changes  that we made.

## Step 3: Connect remotely to your Raspberry Pi via SSH 



### On Master PC

````shell
sudo apt update
sudo apt upgrade
sudo apt install net-tools
sudo apt install openssh-server openssh-client
sudo systemctl status ssh
sudo ufw allow ssh
## To enable SSH after boot
sudo systemctl enable ssh
## setup ssh key pair
ssh-keygen -A
## copy ssh publick key to slave machines
ssh-copy-id username@ip_address
````

#### Setup ROS_MASTER_URI and ROS_IP in the ~/.bashrc of the master PCC:\Users\rar\Desktopl build-essential -y
sudo apt install python3-rosdep
sudo rosdep init
sudo rosdep update
````

#### To enable multi-machine setup for running ROS

````shell
echo "export ROS_MASTER_URI=http://192.168.1.185:11311" >> ~/.bashrc
echo "export ROS_IP 192.168.1.xxx" >> ~/.bashrc  ## The ip_address of the slave RPi should be set here
source ~/.bashrc
````

## Step 4: Install and configure ROS Noetic on Slave RPi

Run the following commands or just follow the instructions from [here](http://wiki.ros.org/noetic/Installation/Ubuntu):

````shell
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-desktop
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo rosdep init
rosdep update
````

## Step 5: Install the ROS packages for the RPLidar application

````shell
# Create a new ROS workspace
mkdir -p lidar_ws/src && cd lidar_ws/src
# Download the repository
git clone https://github.com/ipa-rar/betterfactory_lidar_firos.git

# Download dependencies
cd betterfactory_lidar_firos
wstool init ~/lidar_ws/src/betterfactory_lidar_firos
wstool merge ~/lidar_ws/src/betterfactory_lidar_firos/betterfactory_lidar_firos.rosinstall 
wstool up
cd ..

# Install dependencies 
rosdep update && rosdep install --from-paths ~/snp_demo_ws/src --ignore-src


cd ~/lidar_ws && catkin_make
source ~/lidar_ws/devel/setup.bash
````

## Step 6: Configure bashrc file for automatic initialization

Add the following lines to the .bashrc file (Replace \<IP\> for the slave RPi IP configured in step 2 and \<NodeNumber\> for the desired Node number identifier):
 
````shell
alias c=clear
PS1='\[\033[1;36m\]\u\[\033[1;31m\]@\[\033[1;32m\]\h:\[\033[1;35m\]\w\[\033[1;31m\]$\[\033[0m\] '
source /opt/ros/noetic/setup.bash
source ~/lidar_ws/devel/setup.bash
export ROS_IP=<IP>
BRed='\033[1;31m'         # Red
BGreen='\033[1;32m'       # Green
BYellow='\033[1;33m'      # Yellow
BBlue='\033[1;34m'        # Blue
echo -e "${BYellow}NODE <NodeNumber>"
echo -e $'\n'${BGreen}$ROS_IP
echo -e $'\n'
````

Source your .bashrc file

````shell
source ~/.bashrc
````

## Step 7: Start the Lidar application on the Raspberry Pi

### On master PC

````shell
roscore
````

### On slave raspberry pi's

````shell
roslaunch firos_lidar_bringup lidar_bringup.launch node_name:=node_X
````

## Expected Behavior

````shell
~$ rostopic list 
/node_1/scan
/node_2/scan
/node_3/scan
/rosout
/rosout_agg
/tf

$ rosnode list 
/node_1/rplidarNode
/node_1/tf
/node_2/rplidarNode
/node_2/tf
/node_3/rplidarNode
/node_3/tf
/rosout

$ rosservice list 
/node_1/rplidarNode/get_loggers
/node_1/rplidarNode/set_logger_level
/node_1/start_motor
/node_1/stop_motor
/node_1/tf/get_loggers
/node_1/tf/set_logger_level
/node_2/rplidarNode/get_loggers
/node_2/rplidarNode/set_logger_level
/node_2/start_motor
/node_2/stop_motor
/node_2/tf/get_loggers
/node_2/tf/set_logger_level
/node_3/rplidarNode/get_loggers
/node_3/rplidarNode/set_logger_level
/node_3/start_motor
/node_3/stop_motor
/node_3/tf/get_loggers
/node_3/tf/set_logger_level
/rosout/get_loggers
/rosout/set_logger_level
````

### TF tree
<img src=".\img\frames.png" alt="frames" style="zoom:50%;" />


### ROS node graph

<img src=".\img\full_system.png" alt="full_system" style="zoom:50%;" />

### Rviz 
<img src=".\img\rviz.png" alt="rviz" style="zoom:50%;" />

