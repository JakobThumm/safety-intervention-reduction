"""This file defines the tests of the python bindings of the LongTermTraj class.

Owner:
    Jakob Thumm (JT)

Copyright:
    This file is part of SaRA-Shield.
    SaRA-Shield is free software: you can redistribute it and/or modify it under 
    the terms of the GNU General Public License as published by the Free Software Foundation, 
    either version 3 of the License, or (at your option) any later version.
    SaRA-Shield is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
    without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
    See the GNU General Public License for more details.
    You should have received a copy of the GNU General Public License along with SaRA-Shield. 
    If not, see <https://www.gnu.org/licenses/>. 

Contributors:

Changelog:
    5.5.22 JT Created File
"""

import pytest
from safety_shield_py import LongTermTraj, Motion


class TestLTTConstruction:
    """This class defines the tests of the py bindings of the LongTermTraj class."""

    def test_constructor1(self):
        """Test the LongTermTraj constructor 1."""
        ltt = LongTermTraj()
        assert ltt is not None
        assert ltt.getLength() == 1

    def test_constructor2(self):
        """Test the LongTermTraj constructor 2."""
        m = [Motion(1)]
        ltt = LongTermTraj(m, 0.001)
        assert ltt is not None
        assert ltt.getLength() == 1


class TestLTTFunctions:
    """This class defines a test fixture for the LongTermTraj tests and the tests itself."""

    @pytest.fixture
    def full_ltt(self):
        """Define a test fixture for the LongTermTraj class.

        Define pos, vel, and acc.
        """
        motions = []
        motions.append(Motion(
            0.0, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], 0.0
        ))
        motions.append(Motion(
            1.0, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [1.0, 0.0, 0.0], 0.0
        ))
        motions.append(Motion(
            2.0, [0.5, 0.0, 0.0], [1.0, 0.0, 0.0], [-1.0, 0.0, 0.0], 0.0
        ))
        motions.append(Motion(
            3.0, [1.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], 0.0
        ))
        return LongTermTraj(motions, 1.0)
    
    def test_interpolate_ds1(self, full_ltt):
        """Test the interpolation function for ds=1."""
        motion = full_ltt.interpolate(1.5, 1.0, 0.0, 0.0, [10, 10, 10], [10, 10, 10], [100, 100, 100])
        assert motion.getAngle()[0] == 0.125
        assert motion.getAngle()[1] == 0
        assert motion.getAngle()[2] == 0
        assert motion.getVelocity()[0] == 0.5
        assert motion.getVelocity()[1] == 0
        assert motion.getVelocity()[2] == 0
        assert motion.getAcceleration()[0] == 1.0
        assert motion.getAcceleration()[1] == 0
        assert motion.getAcceleration()[2] == 0
        
    def test_interpolate_ds05(self, full_ltt):
        """Test the interpolation function."""
        motion = full_ltt.interpolate(1.5, 0.5, 1.0, 0.0, [10, 10, 10], [10, 10, 10], [100, 100, 100])
        assert motion.getAngle()[0] == 0.125
        assert motion.getAngle()[1] == 0
        assert motion.getAngle()[2] == 0
        assert motion.getVelocity()[0] == 0.25
        assert motion.getVelocity()[1] == 0
        assert motion.getVelocity()[2] == 0
        assert motion.getAcceleration()[0] == 1.0 * 0.5 + 0.25 * 1.0
        assert motion.getAcceleration()[1] == 0
        assert motion.getAcceleration()[2] == 0

    def test_setLongTermTrajectory(self, full_ltt):
        """Test setLongTermTrajectory() function."""
        m = [Motion(1)]
        full_ltt.setLongTermTrajectory(m)
        assert full_ltt.getLength() == 1

    def test_getLength(self, full_ltt):
        """Test the getLength() function."""
        assert full_ltt.getLength() == 4

    def test_getCurrentMotion(self, full_ltt):
        """Test the getCurrentMotion() function."""
        m = full_ltt.getCurrentMotion()
        m0 = Motion(
            0.0, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], 0.0
        )
        assert m.hasSamePos(m0)
        assert m.hasSameVel(m0)
        assert m.hasSameAcc(m0)

    def test_getNextMotion(self, full_ltt):
        """Test the getNextMotion() function."""
        m = full_ltt.getNextMotion()
        m1 = Motion(
            1.0, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [1.0, 0.0, 0.0], 0.0
        )
        assert m.hasSamePos(m1)
        assert m.hasSameVel(m1)
        assert m.hasSameAcc(m1)

    def test_getNextMotionAtIndex(self, full_ltt):
        """Test the getNextMotionAtIndex() function."""
        m = full_ltt.getNextMotionAtIndex(3)
        m3 = Motion(
            3.0, [1.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], 0.0
        )
        assert m.hasSamePos(m3)
        assert m.hasSameVel(m3)
        assert m.hasSameAcc(m3)

    def test_getTrajectoryIndex0(self, full_ltt):
        """Test the getTrajectoryIndex(0) function."""
        assert full_ltt.getTrajectoryIndex(0) == 0

    def test_getTrajectoryIndex4(self, full_ltt):
        """Test the getTrajectoryIndex(4) function."""
        assert full_ltt.getTrajectoryIndex(4) == 3

    def test_getCurrentPos(self, full_ltt):
        """Test the getCurrentPos() function."""
        assert full_ltt.getCurrentPos() == 0

    def test_increasePosition(self, full_ltt):
        """Test the increasePosition() function."""
        assert full_ltt.getCurrentPos() == 0
        full_ltt.increasePosition()
        assert full_ltt.getCurrentPos() == 1
        full_ltt.increasePosition()
        assert full_ltt.getCurrentPos() == 2
        full_ltt.increasePosition()
        assert full_ltt.getCurrentPos() == 3
        full_ltt.increasePosition()
        assert full_ltt.getCurrentPos() == 3
