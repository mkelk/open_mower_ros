#!/usr/bin/env python3
from datetime import datetime
import json
import rospy
import paho.mqtt.client as mqtt
from mower_msgs.msg import Status

import dynamic_reconfigure.client


TST_STATUS = """ {
	"state": "stopped again 9",
	"data": {
		"temp1": 22.0,
		"temperature": 43.0,
        "stringtest": "Morten sending string _msg_"
	}
}"""

mower_status_codes = {
    Status.MOWER_STATUS_INITIALIZING : 'MOWER_STATUS_OK',
    Status.MOWER_STATUS_OK : 'MOWER_STATUS_OK'
}



last_status = None


#####################################################################3
# MQTT
#####################################################################3
def compose_tst_msg():
    now = datetime.now()
    mytime = now.strftime("%H:%M:%S")
    msg = TST_STATUS
    msg = msg.replace("_msg_", mytime)
    return msg

def on_log(client, userdata, level, buf):
    rospy.loginfo("log: ",buf)

def on_message(client, userdata, message):
    print("message received " ,str(message.payload.decode("utf-8")))
    print("message topic=",message.topic)
    print("message qos=",message.qos)
    print("message retain flag=",message.retain)

def composeEscStatus(escdata):
    data = {}
    data['status'] = escdata.status
    data['current'] = escdata.current
    data['tacho'] = escdata.tacho
    data['temperature_motor'] = escdata.temperature_motor
    data['temperature_pcb'] = escdata.temperature_pcb
    return data

def composeEscStatusJson(escdata):
    data = composeEscStatus(escdata)
    json_data = json.dumps(data, indent=2)
    print(json_data)
    return json_data

def composeStatus():
    data = {}
    if not last_status == None:
        now = datetime.now()
        mytime = "Time is: " + now.strftime("%H:%M:%S")
        data['mower_status'] = mower_status_codes[last_status.mower_status]
        data['heartbeat'] = mytime
        data['raspberry_pi_power'] = last_status.raspberry_pi_power
        data['gps_power'] = last_status.gps_power
        data['esc_power'] = last_status.esc_power
        data['rain_detected'] = last_status.rain_detected
        data['sound_module_busy'] = last_status.sound_module_busy
        data['ui_board_available'] = last_status.ui_board_available
        data['ultrasonic_ranges'] = last_status.ultrasonic_ranges
        data['emergency'] = last_status.emergency
        # TODO: Remove minutes when we have real data
        data['v_charge'] = last_status.v_charge
        data['v_battery'] = last_status.v_battery + 1.0 / 10.0 * datetime.now().minute 
        data['charge_current'] = last_status.charge_current
    return data

def composeStatusJson():
    data = composeStatus()
    json_data = json.dumps(data, indent=2)
    print(json_data)
    return json_data

#####################################################################3
# Dynamic reconfigure mower_logic
#####################################################################3
def getConfig(nodename):
    rcfg_client = dynamic_reconfigure.client.Client(nodename, timeout = 10)
    parameters = rcfg_client.get_configuration()
    return parameters

def getConfigJson(nodename):
    json_data = json.dumps(getConfig(nodename), indent=2)
    print(json_data)
    return json_data

def setManualStartMowing():
    nodename = "mower_logic"
    config = getConfig(nodename)
    config['manual_start_mowing'] = True
    rcfg_client = dynamic_reconfigure.client.Client(nodename, timeout = 10)
    rcfg_client.update_configuration(config)


#####################################################################3
# ROS subscriber callbacks
#####################################################################3
def status_callback(status):
    global last_status
    last_status = status


if __name__ == '__main__':
    # start node
    node = rospy.init_node("test_mqtt")

    # set up needed ROS subscriptions for knowing status
    sub_status = rospy.Subscriber('/mower/status', Status, status_callback, queue_size=10)

    # set up MQTT client
    broker_address="192.168.1.103" 
    mqtt_client = mqtt.Client("P1") #create new instance
    mqtt_client.username_pw_set("homeassistant", "EiX5kahy7ecai7iquah5deemoN2thu6eeCheehaetoox7au9ooN1eiRooShiewah")
    mqtt_client.on_log=on_log
    mqtt_client.on_message=on_message 
    mqtt_client.connect(broker_address, port=1883, keepalive=60) #connect to broker

    # demo of MQTT subsctiption - TODO
    # mqtt_client.subscribe("/home/openmower/info")
    # demo of setting dynamic reconfigure parameter 
    # setManualStartMowing()

    mqtt_client.loop_start() 

    while not rospy.is_shutdown():
        # publish mower status
        mqtt_client.publish("/home/openmower/info",payload=compose_tst_msg(), qos=1)
        if not last_status == None:
            mqtt_client.publish("/home/openmower/mower_status", payload=composeStatusJson(), qos=0)
            mqtt_client.publish("/home/openmower/left_motor_status", payload=composeEscStatusJson(last_status.left_esc_status), qos=0)
            mqtt_client.publish("/home/openmower/right_motor_status", payload=composeEscStatusJson(last_status.right_esc_status), qos=0)
            mqtt_client.publish("/home/openmower/mow_motor_status", payload=composeEscStatusJson(last_status.mow_esc_status), qos=0)

        # publish mower_logic dynamic config
        mqtt_client.publish("/home/openmower/mower_logic_config", payload=getConfigJson("mower_logic"), qos=0)
    
        rospy.sleep(1)
        rospy.loginfo("looping...")

    mqtt_client.loop_stop()    

    