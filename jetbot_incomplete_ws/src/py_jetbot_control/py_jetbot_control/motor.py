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
import atexit
import qwiic
from Adafruit_MotorHAT import Adafruit_MotorHAT


# Scan for devices on I2C bus
addresses = qwiic.scan()


class Motor:
    alpha = 1.0
    beta = 0.0

    # Adafruit Hardware
    if 96 in addresses:

        def __init__(self, driver, channel, *args, **kwargs):
            self._driver = driver
            self._motor = self._driver.getMotor(channel)
            self._value = 0
            if channel == 1:
                self._ina = 1
                self._inb = 0
            else:
                self._ina = 2
                self._inb = 3
            atexit.register(self._release)

        def _write_value(self, value):
            """Sets motor value between [-1, 1]"""
            mapped_value = int(255.0 * (self.alpha * value + self.beta))
            speed = min(max(abs(mapped_value), 0), 255)
            self._value = value
            self._motor.setSpeed(speed)
            if mapped_value < 0:
                self._motor.run(Adafruit_MotorHAT.FORWARD)
                # The two lines below are required for the Waveshare JetBot Board only
                self._driver._pwm.setPWM(self._ina, 0, 0)
                self._driver._pwm.setPWM(self._inb, 0, speed * 16)
            else:
                self._motor.run(Adafruit_MotorHAT.BACKWARD)
                # The two lines below are required for the Waveshare JetBot Board only
                self._driver._pwm.setPWM(self._ina, 0, speed * 16)
                self._driver._pwm.setPWM(self._inb, 0, 0)

        def _release(self):
            """Stops motor by releasing control"""
            self._motor.run(Adafruit_MotorHAT.RELEASE)
            # The two lines below are required for the Waveshare JetBot Board only
            self._driver._pwm.setPWM(self._ina, 0, 0)
            self._driver._pwm.setPWM(self._inb, 0, 0)

    # SparkFun Hardware
    elif 93 in addresses:

        def __init__(self, driver, channel, *args, **kwargs):
            super(Motor, self).__init__(*args, **kwargs)  # initializes traitlets

            self._driver = driver
            atexit.register(self._release)
            self.channel = channel

        def _write_value(self, value):
            """Sets motor value between [-1, 1]"""
            self._value = value
            speed = int(255 * (self.alpha * value + self.beta))

            # Set Motor Controls: .set_drive( motor number, direction, speed)
            # Motor Number: A = 0, B = 1
            # Direction: FWD = 0, BACK = 1
            # Speed: (-255) - 255 (neg. values reverse direction of motor)

            if self.channel == 1:
                self._motor = self._driver.set_drive(self.channel - 1, 0, speed)
            elif self.channel == 2:
                self._motor = self._driver.set_drive(self.channel - 1, 0, speed)
            self._driver.enable()

        def _release(self):
            """Stops motor by releasing control"""
            self._driver.disable()

    def set_speed(self, speed):
        self._write_value(speed)
