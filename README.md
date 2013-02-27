-------------------------------------------
SAFETY WARNING AND DISCLAIMER
-------------------------------------------

You are using this software at your onw risk. The authors do not accept any responsibility for personal injuries and/or property damage.

Some drones supported by this framework ARE NOT TOYS. Even operation by expert users might cause SERIOUS INJURIES to people around. So, please consider flying in a properly screened or isolated flight area.

-------------------------------------------
WHAT IS MULTIROTOR CONTROLLER FOR MAVWORK?
-------------------------------------------

It is a general controller architecture to achieve speed, position or trajectory control for a Multirotor. The implementations is compatible with MAVwork (https://github.com/uavster/mavwork) and with the following multirotors: AR Drone 1, Asctec Pelican and Uastech LinkQuad. The size of this multirotors is about 20-50 cm tall, and as seen from above a square of less than 1mx1m size. It should be useful for multirotors of similar sizes, for instances oktopers of as seen from above a square of less than 1.5mx1.5m size.

For more information, please, visit http://vision4uav.com/?q=node/338 .

-------------------------------------------
 INSTALLATION
-------------------------------------------

1. Download and install MAVwork (https://github.com/uavster/mavwork).

2. Download the Multirotor Controller for MAVwork (https://github.com/jespestana/MultirotorController4mavwork).
3. Edit the makefile to set the path to all required library dependencies:
	- ATLANTE_ROOTPATH, set the path to the "atlante" library (included in MAVwork)
	- DRONECLIENT_ROOTPATH, set the path to the "drone client" library (included in MAVwork)
	- VICON_LIBPATH, set the path to the "Vicon" shared library (required for MAVwork, but soon will be optional)

4. Select the multirotor you will be using on '{Your_MAVwork_directory}/include/config_Mydrone.h', by defining only the corresponding _MULTIROTOR_IS_* constant. The MULTIROTOR_PROXY_HOST_IP must be set to the ip where the 'MAVwork' server will be running.
5. Go to '{Your_MultirotorController_directory}/bin' and run 'make clean all', this will create the mavBrain executable in the 'bin/' folder

------------------------------------------
HOW TO RUN
------------------------------------------

For AscTec Pelican:
1. Run the proxy onboard the Pelican with './run.sh' from '{Your_MAVwork_directory}/proxies/Pelican/bin/' (change permissions with 'chmod +x run.sh' if needed)

For Parrot AR.Drone:
1. Connect the computer where the proxy will run to the AR.Drone adhoc network
2. Run the AR.Drone proxy at 'proxies/ARDrone/Build/Release/cvgDroneProxy'

For Uastech LinkQuad:
1. Run the proxy onboard the LinkQuad with './run.sh' from '{Your_MAVwork_directory}/proxies/Pelican/bin/' (change permissions with 'chmod +x run.sh' if needed)

After running the specific proxy, go to '{Your_MultirotorController_directory}/bin' and run './mavBrain'. Make sure that you compiled it with the correct proxy address defined at '{Your_MAVwork_directory}/include/config_Mydrone.h' as MULTIROTOR_PROXY_HOST_IP.

------------------------------------------
MULTIROTOR CONFIGURATION
------------------------------------------
The controller and state observer architecture are explained in the Msc Thesis:

Author: J. Pestana, Tutor: P. Campoy
"On-board control algorithms for autonomous indoors navigation of Multirotor Micro Air Vehicles"
Computer Vision Group, Universidad Politecnica de Madrid, October 2012
(A Final Master Project submitted for the degree of Master in Robotics and Automation)

The configuration of the architecture is defined by the constants specified in the following files:
{Your_MAVwork_directory}/include/stateObserver/models/EKF_config_MULTIROTOR_MODEL.h"
{Your_MAVwork_directory}/include/stateObserver/models/EKF_model_MULTIROTOR_MODEL.h"
{Your_MAVwork_directory}/include/controller/config/parrot/config_controller_MULTIROTOR_MODEL.h"

where MULTIROTOR_MODEL is either Parrot, Pelican or LinkQuad. These files are always included in your .cpp files if '{Your_MAVwork_directory}/include/config_Mydrone.h' is included. The configuration constant names should be self-explanatory.

If you have any doubts, please contact me at jesus.pestana@upm.es , or through the project's github website https://github.com/jespestana/MultirotorController4mavwork .
