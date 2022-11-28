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

Using pytest package for testcases. The tests are in `pytest_arm.py`.

See pytest docs https://docs.pytest.org/en/7.1.x/contents.html. 

## Installing pytest
```
$pip install pytest
```

## Running pytest test cases

Run pytest command

```
pytest -o log_cli=true -q <path to pytest_arm.py>
```

`-o` enables live logging.

`-k 'test case name pattern' ` allows for selection of a specific test case from the file.

eg. 

This command will run all test cases with object in their name:
```
pytest -o log_cli=true -q src/ob1_arm_control/scripts/pytest_arm.py -k 'object' 
```

Run test cases with print logs:
```
pytest -o log_cli=true -q src/ob1_arm_control/scripts/pytest_arm.py -k 'object' -s
```

Run test cases multiple times (e.g. 10):
```
pip install pytest-repeat
pytest -o log_cli=true -q src/ob1_arm_control/scripts/pytest_arm.py -k 'object' --count=10
```

## For older test cases
Run:
```
$ chmod +x test_arm.py
```

To run the test file, ensure the workspace has been built and sourced correctly and then run:
```
$ rosrun ob1_arm_control test_arm.py
```

*TODO: In the future consider a more robust testing method and seperating test cases into different files, and perhaps selecting test cases as launch parameters at the command line.*





