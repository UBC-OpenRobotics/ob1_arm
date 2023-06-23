#!/usr/bin/env python3
from arm_commander import ArmCommander

ac = ArmCommander()

ac.go_position([0.1,0.1,0.1])