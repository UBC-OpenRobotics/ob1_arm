# ob1_arm_control package

Author:

Yousif El-Wishahy

Email: yel.wishahy@gmail.com

# Description 

This package contains the arm_commander.py module which contains the ArmCommander class (located in ob1_arm_control/arm_commander.py) that is called by the main robot loop for control and manipulation related tasks. Test cases can be found in the test/ directory.

# File Structure
The `ob1_arm_control` ros package, contains a sub directory with the same name containing the python package.

```
ob1_arm_control/
├── CMakeLists.txt
├── README.md
├── launch
├── msg
├── ob1_arm_control **ROOT FOLDER OF PYTHON PACKAGE**
├── package.xml
├── scripts
├── setup.py
├── srv
└── test
```

# Demo

**A demo exists at `test/arm_commander_demo.py`**

# Import and Usage
Build and source the workspace, then the following imports should work from any python file in the system.


```
#import arm commander class from ob1_arm_control
from ob1_arm_control import ArmCommander

#init arm commander instance with default config
arm_commander = ArmCommander()
```

**A demo exists at `test/arm_commander_demo.py`**


# Running the tests

Using pytest package for testcases. The tests are in `test/test_cases.py`.

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
pytest -o log_cli=true -q <path to test_cases.py> -k 'object' 
```

Run test cases with print logs:
```
pytest -o log_cli=true -q <path to test_cases.py -k 'object' -s
```

Run test cases multiple times (e.g. 10):
```
pip install pytest-repeat
pytest -o log_cli=true -q <path to test_cases.py -k 'object' --count=10
```

Putting it all together: Run go target tests with quiet and log_cli modes on, repeat each test function 10 times and generate a report named `test_6dof_report.html`
```
pytest -o log_cli=true -q <path to test_cases.py -k 'GoTarget' --count=10 --repeat-scope=function --html=test_6dof_report.html
```