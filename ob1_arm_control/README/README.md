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

You can also install option, but very useful addons
```
$pip install pytest-repeat pytest-html
```

## Running pytest test cases

### Run pytest command

```
pytest -o log_cli=true -q <path to pytest_arm.py>
```
`-q` quiet (decreased verbosity in command line logs)

`-o` override configurations

`-k 'test case name pattern'` allows for selection of a specific test case from the file.

`--count=<# of times>` repeat tests multiple times

`--repeat-scope={function,class,module,session}` scope of test reptition 

`--html=<filename>.html` generate an html test report in current directory

### examples 

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

Putting it all together: Run go target tests with quiet and log_cli modes on, repeat each test function 10 times and generate a report named `test_6dof_report.html`
```
pytest -o log_cli=true -q src/ob1_arm_control/scripts/test/test_cases.py -k 'GoTarget' --count=10 --repeat-scope=function --html=test_6dof_report.html
```





