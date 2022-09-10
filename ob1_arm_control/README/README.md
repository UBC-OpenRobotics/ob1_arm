# ob1_arm_control package


Author:

Yousif El-Wishahy

Email: yel.wishahy@gmail.com

# Description 

This package contains the ArmCommander class (located in scripts/arm_commander.py) that is called by the main robot loop for manipulation related tasks such as picking up objects. Additionally, inverse kinematics and pick and place test cases are also in this package in scripts/test_arm.py. 

# Usage 

```
from arm_commander import ArmCommander
```

WIP


# Testing

Ensure that test_arm.py is executible on your system
Run:
```
$ chmod +x test_arm.py
```

To run the test file, ensure the workspace has been built and sourced correctly and then run:
```
$ rosrun ob1_arm_control test_arm.py
```

*TODO: In the future consider a more robust testing method and seperating test cases into different files, and perhaps selecting test cases as launch parameters at the command line.*





