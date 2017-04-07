# ESP8266-multirotor-flight-control-board
Minimalistic ESP8266 based multirotor flight control board that can be controlled via phone/desktop app over Wi-Fi

Hello All!
Here is a code for controlling a quadcopter/multirotor exaclty like you play games on your computer..
as we use WASD / Space / Shift to control motions

I have made a processing based GUI for this purpose - 
'W' and 'S' to pitch forward and backward respectively
'A' and 'D' to roll left and right respectively
'Q' and 'E' to yaw counter-clockwise and clockwise respectively
Space to jump (increase throttle here)
Shift to Crouch (decrease throttle here)

The GUI has provisions to display all the important parameters like sensor values, motor values, battery voltage etc.
Also, as opposed to conventionally needing to reflash the microcontroller whenever it was needed to change PID, this sketch allows remotely and realtime changing PID values over a large range..
PID can also be changed inflight but this functionality is disabled by default.. only enable this if there is large open space and very less risk to quad and people nearby..

The controller(processing client) also has the functions of a regular FHSS transmitter - setting trims, sensitivity, channel reversing etc..
and much more versatile as you have total control over the data sent and recieved as well as theoratically infinite channels..

Joystick, PS controller functionalities will also be added in a while..

The andorid code will be made soon.. so that quad can be controlled as we control cars in motion sensing racing games - 
via motion (roll pitch yaw on phone can be transferred into quad movements) 

The images will properly explain the circuit layout and working of GUI
  
  The code is properly documented and explained with images in the file Quad.pdf... 
  Please be sure to read it before attempting to fly

For it's working, both (client and ESP) should be on the same Wi-Fi network, enter the static IP for the ESP in the ESP code as well as processing code
You can even create a Wi-Fi hotspot from your laptop or you can configure the ESP for it..
however, AP mode will put some load on ESP and might affect performance.. you can check this for yourself and then decide acordingly...
I used m laptop hotspot and it provided a LOS range of around 180m
A dedicated Wi-Fi router is better suited for this application as it will provide higher range
If possible, do your entire flying in LOS only as obstacles attenuate Wi-Fi rapidly..


PS: The ESP code is for the ESP8266 board for Arduino IDE. Install the board if you haven't already.
