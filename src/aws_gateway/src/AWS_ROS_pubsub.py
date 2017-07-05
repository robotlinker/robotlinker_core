#!/usr/bin/env python
from AWSIoTPythonSDK.MQTTLib import AWSIoTMQTTClient
import sys
import logging
import time
import getopt
from std_msgs.msg import String
from json_message_converter import *
import rospy
from sensor_msgs.msg import JointState
import ast

ROS_TOPIC_NAME = 'joint_states'
AWS_SEND_TOPIC = 'hello'
AWS_CALLBACK_TOPIC = 'hello2'
ROS_ROBOT_TOPIC = '/ur_driver/URScript'

class AWS_ROS_Comm:
    def __init__(self):
        self.robot_control_pub = rospy.Publisher(ROS_ROBOT_TOPIC, String)
        useWebsocket = False
        host = ""
        rootCAPath = ""
        certificatePath = ""
        privateKeyPath = ""
        host_file_addr = rospy.get_param("~aws_service_location")
        rootCAPath = rospy.get_param("~aws_service_root_CA")
        certificatePath = rospy.get_param("~aws_service_cert")
        privateKeyPath = rospy.get_param("~aws_service_private_key")
        useWebsocket = False

        host_file = open(host_file_addr, 'r')
        host = host_file.readline()[:-1]




#        try:
#        	opts, args = getopt.getopt(sys.argv[1:], "hwe:k:c:r:", ["help", "endpoint=", "key=","cert=","rootCA=", "websocket"])
#        	if len(opts) == 0:
#        		raise getopt.GetoptError("No input parameters!")
#        	for opt, arg in opts:
#        		if opt in ("-h", "--help"):
#        			print(helpInfo)
#        			exit(0)
#        		if opt in ("-e", "--endpoint"):
#        			host = arg
#        		if opt in ("-r", "--rootCA"):
#        			rootCAPath = arg
#        		if opt in ("-c", "--cert"):
#        			certificatePath = arg
#        		if opt in ("-k", "--key"):
#        			privateKeyPath = arg
#        		if opt in ("-w", "--websocket"):
#        			useWebsocket = True
#        except getopt.GetoptError:
#        	exit(1)
#        missingConfiguration = False
#        if not host:
#        	print("Missing '-e' or '--endpoint'")
#        	missingConfiguration = True
#        if not rootCAPath:
#        	print("Missing '-r' or '--rootCA'")
#        	missingConfiguration = True
#        if not useWebsocket:
#        	if not certificatePath:
#        		print("Missing '-c' or '--cert'")
#        		missingConfiguration = True
#        	if not privateKeyPath:
#        		print("Missing '-k' or '--key'")
#        		missingConfiguration = True
#        if missingConfiguration:
#        	exit(2)
        
        # Configure logging
        logger = logging.getLogger("AWSIoTPythonSDK.core")
        logger.setLevel(logging.DEBUG)
        streamHandler = logging.StreamHandler()
        formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        streamHandler.setFormatter(formatter)
        logger.addHandler(streamHandler)
        
        # Init AWSIoTMQTTClient
        self.myAWSIoTMQTTClient = None
        if useWebsocket:
        	self.myAWSIoTMQTTClient = AWSIoTMQTTClient("basicPubSub", useWebsocket=True)
        	self.myAWSIoTMQTTClient.configureEndpoint(host, 443)
        	self.myAWSIoTMQTTClient.configureCredentials(rootCAPath)
        else:
        	self.myAWSIoTMQTTClient = AWSIoTMQTTClient("basicPubSub")
        	self.myAWSIoTMQTTClient.configureEndpoint(host, 8883)
        	self.myAWSIoTMQTTClient.configureCredentials(rootCAPath, privateKeyPath, certificatePath)
        
        # AWSIoTMQTTClient connection configuration
        self.myAWSIoTMQTTClient.configureAutoReconnectBackoffTime(1, 32, 20)
        self.myAWSIoTMQTTClient.configureOfflinePublishQueueing(-1)  # Infinite offline Publish queueing
        self.myAWSIoTMQTTClient.configureDrainingFrequency(2)  # Draining: 2 Hz
        self.myAWSIoTMQTTClient.configureConnectDisconnectTimeout(10)  # 10 sec
        self.myAWSIoTMQTTClient.configureMQTTOperationTimeout(5)  # 5 sec
        # Connect and subscribe to AWS IoT
        self.myAWSIoTMQTTClient.connect()
        self.myAWSIoTMQTTClient.subscribe(AWS_CALLBACK_TOPIC, 1, self.customCallback)
        print 'here'
        time.sleep(2)

    def customCallback(self, client, userdata, message):
    	print("Received a new message: ")
    	print(message.payload)
    	print("from topic: ")
    	print(message.topic)
        ## convert aws msg to a string command
        
        #command_str = self.decode_json(message.payload)
        aws_fake_msg = '{"x":1, "y":0, "z":0}'
        command_str = self.decode_json(aws_fake_msg)
        self.robot_control_pub.publish(command_str)

    	print("--------------\n\n")

    def decode_json(self, msg):
        ## parse aws string
        dict_msg = ast.literal_eval(msg)
        linear_direction = [dict_msg['x'], dict_msg['y'], dict_msg['z'], 0, 0, 0]
        linear_speed = [ 0.04 * l for l in linear_direction]

        command = 'speedl(' + str(linear_speed) + ', 0.1)'

        return command

    def run(self):
        while True and not rospy.is_shutdown():
            print ('waiting for message ...')
            ros_msg = rospy.wait_for_message(ROS_TOPIC_NAME, JointState)
            json_msg = self.ros2json(ros_msg)
            #print json_msg
            #self.myAWSIoTMQTTClient.publish('aws_iot_test', json_msg, 1)
            self.myAWSIoTMQTTClient.publish(AWS_SEND_TOPIC, json_msg, 1)
            time.sleep(1)

    def ros2json(self, msg):
        return convert_ros_message_to_json(msg)

    def json2ros(self, rosmsg_type, msg):
        return convert_json_to_ros_message(rosmsg_type, msg)

        
if __name__=='__main__':
    rospy.init_node('aws_test')
    aws = AWS_ROS_Comm()
    aws.run()
    #rospy.spin()
