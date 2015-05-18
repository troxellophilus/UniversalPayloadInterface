// Arduino MAVLink test code.

//#include <FastSerial.h>
#include <mavlink.h>        // Mavlink interface

#define PAYLOAD_SYSTEM_ID 101 // System ID for MAVlink packets

int ledPin = 25;

//FastSerialPort0(Serial); 

void setup() {
	pinMode(ledPin, OUTPUT);
	Serial.begin(115200);
}

void loop() { 
	// Define the system type (see mavlink_types.h for list of possible types) 
	int system_type = MAV_TYPE_ONBOARD_CONTROLLER;
	int autopilot_type = MAV_AUTOPILOT_ARDUPILOTMEGA;
	int component_id = 101; //MAV_COMP_ID_PAYLOAD;
	int base_mode = MAV_MODE_FLAG_TEST_ENABLED;
	int sys_status = MAV_STATE_STANDBY;
	
	// Initialize the required buffers 
	mavlink_message_t msg; 
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	
	// Setup the heartbeat message
	//mavlink_msg_heartbeat_pack(PAYLOAD_SYSTEM_ID, component_id, &msg, system_type, autopilot_type, base_mode, 0, sys_status);
	mavlink_msg_mission_request_list_pack(PAYLOAD_SYSTEM_ID, component_id, &msg, MAV_TYPE_QUADROTOR, MAV_COMP_ID_ALL);

	// Copy the message to send buffer 
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
	
	// Send the message (.write sends as bytes) 
	Serial.write(buf, len);
	
	// slow down there young padawan
	delay(1000);
	
	comm_receive();
}

void comm_receive() { 
	static int w = 0;
	mavlink_message_t msg; 
	mavlink_status_t status;
	
	//receive data over serial 
	while(Serial.available() > 0) { 
		uint8_t c = Serial.read();
		
		//try to get a new message 
		if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) { 
			// Handle message
 			switch(msg.msgid) {
			        case MAVLINK_MSG_ID_HEARTBEAT:
					if (w == 0) {
						digitalWrite(ledPin, HIGH);
						w = 1;
					}
					else {
						digitalWrite(ledPin, LOW);
						w = 0;
					}
			        	break;
				case MAVLINK_MSG_ID_MISSION_ITEM:
					digitalWrite(ledPin, HIGH);
					break;
				default:
					//Do nothing
				break;
			}
		} 
		// And get the next one
	}
}
