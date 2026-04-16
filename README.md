# EVEI_Geofence_Code

We have gotten pretty far on both the code and the electronics. This readme is going to layout how to setup and interact with what we have done, and how to build upon it.

Setup:
1. Install the 2 main files into a folder on your desktop
   -These are computer-end-code, and main.py
   
   -The main file is what runs on the pico, handles the gps and relay
   
   -The computer code runs on your pc and loads the boundary for the geofence on the pico
3. Download Thonny and VsCode
   -https://thonny.org/
   -Thonny is an IDE that the original project was built on, as much as we would like to change it, it just hindered progress a lot
   -VsCode is pretty straight forward, you should have installed it for an engineering class, or have someone in your team that can help you
4. Setup Thonny
   -This might be a little confusing
   -At the top hit view and make sure the "Files" is checked, there should be a window on the left side
   -Plug in the pico to one of your computer ports
   -Go to the top and follow this Tools>Options>Interpreters
   -Click on the top dropdown and select MicroPython Raspberry Pi Pico
   -Then at the bottom dropdown, select the port that the pico is plugged into and hit OK
   -You should see the files tab on the left change, and you should see a main.py file show up, along with a lib file
   -The lib just contains libraries that the code needs to run, and copies of it are placed in the github
   -The main.py file should be present and if you double click it, you should be able to edit the file
5. Setup VsCode
   -Open your VsCode and open your folder with the computer-end-code in it
   -In the terminal you have to install the following package (run this command): pip install pyserial
   -In the code under the initialize function you have to change the COM- port. Change the number to the same number as the thonny interpreter used
   -You also need the coordinates.csv in your folder from the website
   -The website can be found here: https://scooter1946.github.io/epics-evei.github.io/
