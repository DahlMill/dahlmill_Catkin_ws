gnome-terminal -x bash -c "roscore"

sleep 1

gnome-terminal -x bash -c "roslaunch usb_cam usb_cam-test.launch"

sleep 2

rosrun camera_calibration cameracalibrator.py --size 11x8 --square 0.030 image:=/usb_cam/image_raw camera:=/usb_cam

#camera_matrix:
#[fx 0 cx 0 fy cy 0 0 1]

#distortion_coefficients:
#[k1 k2 p1 p2 k3]
