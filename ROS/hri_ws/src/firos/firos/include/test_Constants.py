# MIT License
# 
# Copyright (c) 2019 Fraunhofer IML
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

__author__ = "Dominik Lux"
__credits__ = ["Dominik Lux"]
__maintainer__ = "Dominik Lux"

import unittest

import include.constants
from include.constants import Constants

class Test_Constants(unittest.TestCase):

    def tearDown(self):
        '''
            Reset Constants to initial State after every Test
        '''
        C = Constants
        C.LOGLEVEL = "INFO"
        C.EP_SERVER_ADRESS = None
        C.EP_SERVER_PORT = None
        C.MAP_SERVER_PORT = 10100
        C.ROSBRIDGE_PORT = 9090
        C.DATA = None
        C.PUB_FREQUENCY = 0
        C.ROS_NODE_NAME = "firos"
        C.ROS_SUB_QUEUE_SIZE = 10
        C.PATH = None
        C.configured = False

    def test_Standard_Parameters(self):
        C = Constants

        self.assertEqual(C.LOGLEVEL, "INFO")

        self.assertEqual(C.EP_SERVER_ADRESS, None)
        self.assertEqual(C.MAP_SERVER_PORT, 10100)
        self.assertEqual(C.ROSBRIDGE_PORT, 9090)
        self.assertEqual(C.DATA, None)
        self.assertEqual(C.PUB_FREQUENCY, 0)

        self.assertEqual(C.ROS_NODE_NAME, "firos")
        self.assertEqual(C.ROS_SUB_QUEUE_SIZE, 10)

        self.assertEqual(C.PATH, None)
        self.assertEqual(C.configured, False)



    def test_setConfiguration_None(self):
        self.assertEqual({}, Constants.setConfiguration(None))

    def test_setConfiguration_EmptyString(self):
        self.assertEqual({}, Constants.setConfiguration(""))

    def test_setConfiguration_Non_Existent_Path(self):
        self.assertEqual({}, Constants.setConfiguration("NON_EXISTENT_PATH"))


    def test_Constants_With_Non_Existent_Config_JSON(self):
        C = Constants
        Constants.configured = False
        C.init("NON_EXISTENT_PATH")

        self.assertEqual(C.LOGLEVEL, "INFO")

        self.assertNotEqual(C.EP_SERVER_ADRESS, None) # should be set apropiatly
        self.assertEqual(C.EP_SERVER_PORT, None)
        self.assertEqual(C.MAP_SERVER_PORT, 10100)
        self.assertEqual(C.ROSBRIDGE_PORT, 9090)
        self.assertEqual(C.DATA, {})
        self.assertEqual(C.PUB_FREQUENCY, 0)

        self.assertEqual(C.ROS_NODE_NAME, "firos")
        self.assertEqual(C.ROS_SUB_QUEUE_SIZE, 10)

        self.assertEqual(C.PATH, "NON_EXISTENT_PATH")
        self.assertEqual(C.configured, True)


    def test_Constants_With_Missing_Endpoint(self):
        C = Constants
        Constants.configured = False
        C.init("../test_data/testConfigFiles/missEndpoint")

        self.assertEqual(C.LOGLEVEL, "WARNING")

        self.assertNotEqual(C.EP_SERVER_ADRESS, None) # should be set aprioiately
        self.assertEqual(C.EP_SERVER_PORT, None)
        self.assertEqual(C.MAP_SERVER_PORT, 12345)
        self.assertEqual(C.ROSBRIDGE_PORT, 4321)
        self.assertEqual(C.DATA, C.setConfiguration("../test_data/testConfigFiles/missEndpoint"))
        self.assertEqual(C.PUB_FREQUENCY, 1)

        self.assertEqual(C.ROS_NODE_NAME, "TestFIROS")
        self.assertEqual(C.ROS_SUB_QUEUE_SIZE, 9)

        self.assertEqual(C.PATH, "../test_data/testConfigFiles/missEndpoint")
        self.assertEqual(C.configured, True)
            



    def test_Constants_With_Missing_log_level(self):
        C = Constants
        Constants.configured = False
        C.init("../test_data/testConfigFiles/missLogLevel")

        self.assertEqual(C.LOGLEVEL, "INFO")

        self.assertEqual(C.EP_SERVER_ADRESS, "123.456.789.254")
        self.assertEqual(C.EP_SERVER_PORT, 1235)
        self.assertEqual(C.MAP_SERVER_PORT, 12345)
        self.assertEqual(C.ROSBRIDGE_PORT, 4321)
        self.assertEqual(C.DATA, C.setConfiguration("../test_data/testConfigFiles/missLogLevel"))
        self.assertEqual(C.PUB_FREQUENCY, 1)

        self.assertEqual(C.ROS_NODE_NAME, "TestFIROS")
        self.assertEqual(C.ROS_SUB_QUEUE_SIZE, 9)

        self.assertEqual(C.PATH, "../test_data/testConfigFiles/missLogLevel")
        self.assertEqual(C.configured, True)
            

    def test_Constants_With_Missing_server_port(self):
        C = Constants
        Constants.configured = False
        C.init("../test_data/testConfigFiles/missServerPort")

        self.assertEqual(C.LOGLEVEL, "WARNING")

        self.assertEqual(C.EP_SERVER_ADRESS, "123.456.789.254")
        self.assertEqual(C.EP_SERVER_PORT, 1235)
        self.assertEqual(C.MAP_SERVER_PORT, 10100)
        self.assertEqual(C.ROSBRIDGE_PORT, 4321)
        self.assertEqual(C.DATA, C.setConfiguration("../test_data/testConfigFiles/missServerPort"))
        self.assertEqual(C.PUB_FREQUENCY, 1)

        self.assertEqual(C.ROS_NODE_NAME, "TestFIROS")
        self.assertEqual(C.ROS_SUB_QUEUE_SIZE, 9)

        self.assertEqual(C.PATH, "../test_data/testConfigFiles/missServerPort")
        self.assertEqual(C.configured, True)


    def test_Constants_With_Missing_ContextBroker(self):
        C = Constants
        Constants.configured = False
        C.init("../test_data/testConfigFiles/missContextBroker")

        self.assertEqual(C.LOGLEVEL, "WARNING")

        self.assertEqual(C.EP_SERVER_ADRESS, "123.456.789.254")
        self.assertEqual(C.EP_SERVER_PORT, 1235)
        self.assertEqual(C.MAP_SERVER_PORT, 12345)
        self.assertEqual(C.ROSBRIDGE_PORT, 4321)
        self.assertEqual(C.DATA, C.setConfiguration("../test_data/testConfigFiles/missContextBroker"))
        self.assertEqual(C.PUB_FREQUENCY, 1)

        self.assertEqual(C.ROS_NODE_NAME, "TestFIROS")
        self.assertEqual(C.ROS_SUB_QUEUE_SIZE, 9)

        self.assertEqual(C.PATH, "../test_data/testConfigFiles/missContextBroker")
        self.assertEqual(C.configured, True)


    def test_Constants_With_Missing_node_name(self):
        C = Constants
        Constants.configured = False
        C.init("../test_data/testConfigFiles/missNodeName")

        self.assertEqual(C.LOGLEVEL, "WARNING")

        self.assertEqual(C.EP_SERVER_ADRESS, "123.456.789.254")
        self.assertEqual(C.EP_SERVER_PORT, 1235)
        self.assertEqual(C.MAP_SERVER_PORT, 12345)
        self.assertEqual(C.ROSBRIDGE_PORT, 4321)
        self.assertEqual(C.DATA, C.setConfiguration("../test_data/testConfigFiles/missNodeName"))
        self.assertEqual(C.PUB_FREQUENCY, 1)

        self.assertEqual(C.ROS_NODE_NAME, "firos")
        self.assertEqual(C.ROS_SUB_QUEUE_SIZE, 9)

        self.assertEqual(C.PATH, "../test_data/testConfigFiles/missNodeName")
        self.assertEqual(C.configured, True)

    def test_Constants_With_Missing_ros_subscriber_queue(self):
        C = Constants
        Constants.configured = False
        C.init("../test_data/testConfigFiles/missRosSubscriberQueue")

        self.assertEqual(C.LOGLEVEL, "WARNING")

        self.assertEqual(C.EP_SERVER_ADRESS, "123.456.789.254")
        self.assertEqual(C.EP_SERVER_PORT, 1235)
        self.assertEqual(C.MAP_SERVER_PORT, 12345)
        self.assertEqual(C.ROSBRIDGE_PORT, 4321)
        self.assertEqual(C.DATA, C.setConfiguration("../test_data/testConfigFiles/missRosSubscriberQueue"))
        self.assertEqual(C.PUB_FREQUENCY, 1)

        self.assertEqual(C.ROS_NODE_NAME, "TestFIROS")
        self.assertEqual(C.ROS_SUB_QUEUE_SIZE, 10)

        self.assertEqual(C.PATH, "../test_data/testConfigFiles/missRosSubscriberQueue")
        self.assertEqual(C.configured, True)



    def test_Constants_With_Missing_Pub_Frequency(self):
        C = Constants
        Constants.configured = False
        C.init("../test_data/testConfigFiles/missPubFrequency")

        self.assertEqual(C.LOGLEVEL, "WARNING")

        self.assertEqual(C.EP_SERVER_ADRESS, "123.456.789.254")
        self.assertEqual(C.EP_SERVER_PORT, 1235)
        self.assertEqual(C.MAP_SERVER_PORT, 12345)
        self.assertEqual(C.ROSBRIDGE_PORT, 4321)
        self.assertEqual(C.DATA, C.setConfiguration("../test_data/testConfigFiles/missPubFrequency"))
        self.assertEqual(C.PUB_FREQUENCY, 0)

        self.assertEqual(C.ROS_NODE_NAME, "TestFIROS")
        self.assertEqual(C.ROS_SUB_QUEUE_SIZE, 9)

        self.assertEqual(C.PATH, "../test_data/testConfigFiles/missPubFrequency")
        self.assertEqual(C.configured, True)


    def test_Constants_With_Missing_Ros_Bridge(self):
        C = Constants
        Constants.configured = False
        C.init("../test_data/testConfigFiles/missRosBridge")

        self.assertEqual(C.LOGLEVEL, "WARNING")

        self.assertEqual(C.EP_SERVER_ADRESS, "123.456.789.254")
        self.assertEqual(C.EP_SERVER_PORT, 1235)
        self.assertEqual(C.MAP_SERVER_PORT, 12345)
        self.assertEqual(C.ROSBRIDGE_PORT, 9090)
        self.assertEqual(C.DATA, C.setConfiguration("../test_data/testConfigFiles/missRosBridge"))
        self.assertEqual(C.PUB_FREQUENCY, 1)

        self.assertEqual(C.ROS_NODE_NAME, "TestFIROS")
        self.assertEqual(C.ROS_SUB_QUEUE_SIZE, 9)

        self.assertEqual(C.PATH, "../test_data/testConfigFiles/missRosBridge")
        self.assertEqual(C.configured, True)


    def test_Constants_With_minimal_Configuration(self):
        C = Constants
        Constants.configured = False
        C.init("../test_data/testConfigFiles/minimal")

        self.assertEqual(C.LOGLEVEL, "INFO")

        self.assertNotEqual(C.EP_SERVER_ADRESS, None) # it is set appropiately
        self.assertEqual(C.EP_SERVER_PORT, None)
        self.assertEqual(C.MAP_SERVER_PORT, 10100)
        self.assertEqual(C.ROSBRIDGE_PORT, 9090)
        self.assertEqual(C.DATA, C.setConfiguration("../test_data/testConfigFiles/minimal"))
        self.assertEqual(C.PUB_FREQUENCY, 0)

        self.assertEqual(C.ROS_NODE_NAME, "firos")
        self.assertEqual(C.ROS_SUB_QUEUE_SIZE, 10)

        self.assertEqual(C.PATH, "../test_data/testConfigFiles/minimal")
        self.assertEqual(C.configured, True)
