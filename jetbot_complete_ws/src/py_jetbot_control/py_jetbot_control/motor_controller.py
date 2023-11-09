# Modified by Mike Likes Robots 2023
# Author: Michael Hart
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY without even the implied warrranty of
# MERCHANABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/license>
#
# ==================================================================================
# Copyright (c) 2021 SparkFun Electronics
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
# ==================================================================================

from Adafruit_MotorHAT import Adafruit_MotorHAT
import qwiic
from py_jetbot_control.motor import Motor


addresses = qwiic.scan()


class MotorController:
    i2c_bus = 1
    left_channel = 1
    right_channel = 2
    left_motor_alpha = 1.0
    right_motor_alpha = 1.0

    if 96 in addresses:

        def __init__(self):
            self._motor_driver = Adafruit_MotorHAT(i2c_bus=self.i2c_bus)
            self._left_motor = Motor(
                self._motor_driver,
                channel=self.left_channel
            )
            self._right_motor = Motor(
                self._motor_driver,
                channel=self.right_channel
            )
            # self._left_motor = self._motor_driver.getMotor(self.left_channel)
            # self._right_motor = self._motor_driver.getMotor(self.right_channel)

        def set_motors(self, left_speed, right_speed):
            print(f"Set AdaFruit motors to {left_speed}, {right_speed}")
            self._left_motor.set_speed(left_speed)
            self._right_motor.set_speed(right_speed)

    # SparkFun Hardware
    elif 93 in addresses:

        def __init__(self, *args, **kwargs):
            self._motor_driver = qwiic.QwiicScmd()
            self._left_motor = Motor(
                self._motor_driver,
                channel=self.left_channel,
                alpha=self.left_motor_alpha,
            )
            self._right_motor = Motor(
                self._motor_driver,
                channel=self.right_channel,
                alpha=self.right_motor_alpha,
            )
            self.motor_driver.enable()

        def set_motors(self, left_speed, right_speed):
            print(f"Set SparkFun motors to {left_speed}, {right_speed}")
            self._left_motor.set_speed(left_speed)
            self._right_motor.set_speed(right_speed)
            self._motor_driver.enable()
