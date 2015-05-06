// Arduino MAVLink test code.

#include <mavlink.h>        // Mavlink interface

#define PAYLOAD_SYSTEM_ID 101 // System ID for MAVlink packets

void setup() {
	Serial.begin(57600);
}

void loop() { 
	// Define the system type (see mavlink_types.h for list of possible types) 
	int system_type = MAV_QUADROTOR;
	int autopilot_type = MAV_AUTOPILOT_ARDUPILOTMEGA;
	int component_id = MAV_COMP_ID_PAYLOAD;
	
	// Initialize the required buffers 
	mavlink_message_t msg; 
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	
	// Setup the heartbeat message
	mavlink_msg_heartbeat_pack(PAYLOAD_SYSTEM_ID, component_id, &msg, system_type, autopilot_type);
	
	// Copy the message to send buffer 
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
	
	// Send the message (.write sends as bytes) 
	Serial.write(buf, len);
	
	comm_receive();
}

void comm_receive() { 
	mavlink_message_t msg; 
	mavlink_status_t status;
	
	//receive data over serial 
	while(Serial.available() > 0) { 
		uint8_t c = Serial.read();
		
		//try to get a new message 
		if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) { 
			// Handle message
 			switch(msg.msgid) {
			        case MAVLINK_MSG_ID_SET_MODE: {
			        	// set mode
			        }
			        break;
			        case MAVLINK_MSG_ID_ACTION:
					// EXECUTE ACTION
				break;
				default:
					//Do nothing
				break;
			}
		} 
		// And get the next one
	}
}
