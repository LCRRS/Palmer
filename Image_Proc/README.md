COLOR TRACKING

Please read this file prior to starting the program and follow the instructions below.

The Color Tracking algorithm is written to provide a multirotor drone real-time image-processing abilities through serial communication between RaspberryPi, responsible for image processing and offset calculation,  and Arduino, serving as the primary flight controller

The color Detection System is written in Python using the OpenCV module as a base. The program isolates large objects of specific color and supplies information - horizontal and vertical position respectful of the frame center as a list of string objects through the serial port to the Arduino. The third object in the list is the approximation of the distance dependent on the area of the object that is been tracked as it appears in the camera.

In order to have the Color-tracking system to work on your personal computer it is required that you have opencv and numpy modules for python installed and tested. Please follow the instructions available online to install the following modules on your system. A good starting point might be: http://docs.opencv.org/doc/tutorials/introduction/table_of_content_introduction/table_of_content_introduction.html#table-of-content-introduction

The angular measurements in the color-tracking program are camera dependent and it is required for the proper responsivness of the system that the maximum angle that the camera being used is measured and the value of it is compensated for in the algorithm.
