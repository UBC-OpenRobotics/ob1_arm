## IKFast Configuration 

IKFast is a more powerful inverse kinematics solver that creates a custom ik plugin for moveit based on the urdf of the robot. Because it is generated for our specific robot arm, it can provide more stable solutions with much shorter calculation times.

Instructions used to generate the IKFast plugin for the ob1_arm:
https://moveit.picknik.ai/humble/doc/examples/ikfast/ikfast_tutorial.html

### Troubleshooting
We load IKFast generator with a docker image; I encountered this issue while attempting to run the generation command:
```
Got permission denied while trying to connect to the Docker daemon socket at unix:///var/run/docker.sock:
```
or
```
dial unix /var/run/docker.sock: connect: permission denied
```

Solution: https://stackoverflow.com/questions/51342810/how-to-fix-dial-unix-var-run-docker-sock-connect-permission-denied-when-gro
```
sudo setfacl --modify user:<user name or ID>:rw /var/run/docker.sock
```


