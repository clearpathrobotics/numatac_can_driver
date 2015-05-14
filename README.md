numatac_can_driver
==================

ROS driver for NumaTac tactile sensors. See [product manual][1] for details.

### Bring-up ###
To bring up the NumaTac tactile sensors, plug in the USB connector.  Configure the CAN interface by running the following command in a command line prompt:

`sudo ip link set can0 type can bitrate 1000000`

and finally bring-up the CAN interface using:

`sudo ifconfig can0 up`

To run the driver use:

`roslaunch numatac_can_driver numatac_can_driver.launch`

[1]: www.syntouchllc.com/Products/NumaTac/_media/NumaTac_Product_Manual.pdf
