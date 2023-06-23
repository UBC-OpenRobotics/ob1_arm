###################################################
#                    IMPORTS        
###################################################
import pytest
from ob1_arm_control.arm_commander import ArmCommander
import numpy as np
import logging
from test_helpers import *

###################################################
#                    TEST FIXTURES          
###################################################
@pytest.fixture(scope="session")
def log():
    return logging.getLogger("Pytest Arm")

@pytest.fixture(scope="session")
def arm_commander():
    return ArmCommander()

@pytest.fixture
def setup_teardown(arm_commander):
    clear_scene(arm_commander)
    arm_commander.open_gripper()
    arm_commander.go_joint(np.zeros(arm_commander._num_joints))
    yield
    clear_scene(arm_commander)
    arm_commander.open_gripper()
    arm_commander.go_joint(np.zeros(arm_commander._num_joints))