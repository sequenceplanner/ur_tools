# ur_tools
### Description:
A collection of tools for controlling Universal Robots using UR Scripts. Scripts are generated using `Tera` templates, and communicated to the robot using the UR Script Driver.

### Depends on:
https://github.com/sequenceplanner/ur_script_driver \
https://github.com/sequenceplanner/ur_script_msgs \
https://github.com/sequenceplanner/ur_tools_msgs.git

### Structure:
![This is an image](/images/structure.png)

### Get a robot or a simulator:

1. Get docker: \
   https://docs.docker.com/get-docker/ \
   NOTE: At least version 19.\
   NOTE: Turn off VPN during installation.
2. Get and start the URSim docker container:

   ```
   sudo docker run --rm -it \
   --name="dockursim" \
   -e ROBOT_MODEL=UR10 \
   -p 5901:5901 \
   -p 6080:6080 \
   -p 29999:29999 \
   -p 30001-30004:30001-30004 \
   --cpus=2 \
   --privileged kristoferb/ursim:latest
   ```
3. The Universal Robot Interface can now be accessed at: \
 http://localhost:6080/vnc.html?host=localhost&port=6080.
4. To start the container again, you will probably have to kill and remove it first:

   ```
   sudo docker kill dockursim
   sudo docker rm dockursim
   ```
