#!/usr/bin/env python3
import rospy
import paho.mqtt.client as mqtt

def on_log(client, userdata, level, buf):
    rospy.loginfo("log: ",buf)

def on_message(client, userdata, message):
    print("message received " ,str(message.payload.decode("utf-8")))
    print("message topic=",message.topic)
    print("message qos=",message.qos)
    print("message retain flag=",message.retain)

if __name__ == '__main__':
    # start node
    node = rospy.init_node("test_mqtt")
    rospy.loginfo("test_mqtt started")
    print("Hello world")

    broker_address="192.168.1.103" 
    #broker_address="iot.eclipse.org" #use external broker
    client = mqtt.Client("P1") #create new instance
    client.username_pw_set("homeassistant", "EiX5kahy7ecai7iquah5deemoN2thu6eeCheehaetoox7au9ooN1eiRooShiewah")
    client.on_log=on_log
    client.on_message=on_message 
    client.connect(broker_address, port=1883, keepalive=60) #connect to broker

    client.subscribe("/home/openmower/info")
    client.publish("/home/openmower/info",payload="OFF", qos=1)#publish

    client.loop_start() 

    while not rospy.is_shutdown():
        client.publish("/home/openmower/info",payload="OFF", qos=1)#publish
        rospy.sleep(1)
        rospy.loginfo("looping...")

    client.loop_stop()    

    