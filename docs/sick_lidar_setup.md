## Settting up the physical sick tim 571 Lidar
- Install the [SOPAS tool](https://www.sick.com/ag/en/sopas-engineering-tool/p/p367244) from the SICK Sensor Intelligence to configure the LIDAR IP address
- Inorder to identify the LIDAR in windows 10
    - Open up `Control pannel -> Network and internet -> Netowork connections -> Ethernet`
    - Right click on the Ethernet and select properties. This needs admin rights
    - Then select `Internet Protocol Version 4 (TCP/IPv4)`
    - Set IP address of series `192.168.1.xxx` and Subnet mask to `255.255.255.0` and save it.
- Now your SOPAS tool can identify your LIDAR
    - Search for the device using the ethernet interface search setting
    - Edit the IP address to the series `192.168.1.xxx` and Subnet mask to `255.255.255.0`
    - This will remove the connection with your LIDAR and you need to search again using the same settings to identify the lidar
- Now you can start using your LIDAR on Ubuntu System
    - Change your IP address to the same series of LIDAR
    - `sudo ifconfig eth0 192.168.1.xxx netmask 255.255.255.0`
- ROS system setup
    - `roslaunch sick_scan sick_tim_5xx.launch hostname:=192.168.1.1`
    - `rosrun tf static_transform_publisher 0 0 0 0 0 0 1 map cloud 10`
    - `rosrun rviz rviz`
    - Laser data is published on `/scan`

