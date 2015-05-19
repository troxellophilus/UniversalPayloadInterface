// Arduino MAVLink test code.

#include <FastSerial.h>
//#define MAVLINK_CHECK_MESSAGE_LENGTH
#define MAVLINK_COMM_NUM_BUFFERS 1
#include <mavlink.h>        // Mavlink interface

#define PAYLOAD_SYSTEM_ID 101 // System ID for MAVlink packets

FastSerialPort0(Serial); 

int ledPin = A2;
int received_heartbeat = 0;

void send_message(mavlink_message_t* msg)
{
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
  uint16_t len = mavlink_msg_to_send_buffer(buf, msg);
  for(uint16_t i = 0; i < len; i++) {
    Serial.write(buf[i]);
  }
}

void setup() {
	pinMode(ledPin, OUTPUT);
	Serial.begin(57600);
}

void loop() { 
	// Define the system type (see mavlink_types.h for list of possible types) 
	int system_type = MAV_TYPE_ONBOARD_CONTROLLER;
	int autopilot_type = MAV_AUTOPILOT_GENERIC;
	int component_id = 101; //MAV_COMP_ID_PAYLOAD;
	int base_mode = MAV_MODE_FLAG_STABILIZE_ENABLED;
	int sys_status = MAV_STATE_UNINIT;
	
	// Initialize the required buffers 
	mavlink_message_t msg; 
	
	// Setup the heartbeat message
	mavlink_msg_heartbeat_pack(PAYLOAD_SYSTEM_ID, component_id, &msg, system_type, autopilot_type, base_mode, 0, sys_status);
	//mavlink_msg_mission_request_list_pack(PAYLOAD_SYSTEM_ID, component_id, &msg, MAV_TYPE_QUADROTOR, MAV_COMP_ID_ALL);

	// Send the message (.write sends as bytes) 
	delay(500);
	if (received_heartbeat) {
		digitalWrite(ledPin, HIGH);
		send_message(&msg);
		received_heartbeat = 0;
		delay(500);
		digitalWrite(ledPin, LOW);
	}
	
	comm_receive();
}

void comm_receive() { 
	int i = 0;
	int frames = 0;
	mavlink_message_t recv_msg; 
	mavlink_status_t recv_status;
	
	//receive data over serial 
	while(Serial.available() > 0) { 
		uint8_t c = Serial.read();

		//try to get a new message 
		if(mavlink_parse_char(0, c, &recv_msg, &recv_status)) { 
			frames++;
			received_heartbeat = 1;
			// Handle message
 			switch(recv_msg.msgid) {
			        case MAVLINK_MSG_ID_HEARTBEAT:
					mavlink_heartbeat_t hb;
					mavlink_msg_heartbeat_decode(&recv_msg, &hb);
					// do something with it

					if (!received_heartbeat) {
						received_heartbeat = 1;
					}
			        	break;
				case MAVLINK_MSG_ID_PARAM_VALUE:
					break;
				default:
					//Do nothing
				break;
			}
		} 
		if (i > 64 || frames > 10) {
			break;
		}
		delayMicroseconds(200);
		i++;
		// And get the next one
	}
	//delay(50);
}
