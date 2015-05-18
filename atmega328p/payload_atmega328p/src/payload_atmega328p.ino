// Arduino MAVLink test code.

//#include <FastSerial.h>
#include <mavlink.h>        // Mavlink interface
#include <SoftwareSerial.h>

#define PAYLOAD_SYSTEM_ID 101 // System ID for MAVlink packets

int ledPin = A2;
int received_heartbeat = 0;

//FastSerialPort0(Serial); 

void send_message(mavlink_message_t* msg)
{
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
//  Serial.write(MAVLINK_STX);
//  Serial.write(msg->len);
//  Serial.write(msg->seq);
//  Serial.write(msg->sysid);
//  Serial.write(msg->compid);
//  Serial.write(msg->msgid);
  uint16_t len = mavlink_msg_to_send_buffer(buf, msg);
  for(uint16_t i = 0; i < len; i++)
  {
    Serial.write(buf[i]);
  }
//  Serial.write(msg->ck_a);
//  Serial.write(msg->ck_b);
}

void setup() {
	Serial.begin(57600);
	pinMode(ledPin, OUTPUT);
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
	
	// Setup the heartbeat message
	mavlink_msg_heartbeat_pack(PAYLOAD_SYSTEM_ID, component_id, &msg, system_type, autopilot_type, base_mode, 0, sys_status);
	//mavlink_msg_mission_request_list_pack(PAYLOAD_SYSTEM_ID, component_id, &msg, MAV_TYPE_QUADROTOR, MAV_COMP_ID_ALL);

	// Send the message (.write sends as bytes) 
	delay(1000);
	if (received_heartbeat) {
		send_message(&msg);
	}
	
	// slow down there young padawan
	//delay(1000);

	//mavlink_msg_mission_request_list_pack(PAYLOAD_SYSTEM_ID, component_id, &msg, MAV_TYPE_QUADROTOR, MAV_COMP_ID_ALL);

	//len = mavlink_msg_to_send_buffer(buf, &msg);

	//if (received_heartbeat)
	//	Serial.write(buf, len);

	//delay(1000);

	while (Serial.available() < 5); // Wait for data to be available before trying to get it
	
	comm_receive();

	//delay(100);
}

void comm_receive() { 
	mavlink_message_t recv_msg; 
	mavlink_status_t recv_status;
	
	//receive data over serial 
	while(Serial.available() > 0) { 
		uint8_t c = Serial.read();

		//try to get a new message 
		if(mavlink_parse_char(0, c, &recv_msg, &recv_status)) { 

			// Handle message
 			switch(recv_msg.msgid) {
			        case MAVLINK_MSG_ID_HEARTBEAT:
					digitalWrite(ledPin, HIGH);
					received_heartbeat = 1;
			        	break;
				case MAVLINK_MSG_ID_PARAM_VALUE:
					break;
				default:
					//Do nothing
				break;
			}
		} 
		// And get the next one
	}
	digitalWrite(ledPin, LOW);
}
