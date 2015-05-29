/*
 * Universal Payload Interface
 * Arduino MAVLink Code
 */

/* Make sure this is included before Arduino's HardwareSerial */
#include <FastSerial.h>

/* Make sure this is defined before including mavlink.h */
#define MAVLINK_COMM_NUM_BUFFERS 1
#include <mavlink.h>

#define PAYLOAD_SYSTEM_ID 101 // System ID for MAVlink packets

FastSerialPort0(Serial); // Required for FastSerial

int ledPin = A2;
int received_heartbeat = 0;

// System Status Variables
uint8_t  mavlink_connected = 0;
uint16_t hb_count = 0;
uint32_t led_pin = A2;

// *************************
// MAVLINK MESSAGE VARIABLES

// Heartbeat (msg id 1)
uint32_t mav_custom_mode = 0;
uint8_t  mav_type = 0;
uint8_t  mav_autopilot = 0;
uint8_t  mav_base_mode = 0;
uint8_t  mav_system_status = 0;
uint8_t  mav_mavlink_version = 0;

// GPS Raw Int (msg id 24)
int32_t  mav_latitude = 0;
int32_t  mav_longitude = 0;
int32_t  mav_altitude = 0;
uint16_t mav_velocity = 0;
uint8_t  mav_fix_type = 0;
uint8_t  mav_sat_visible = 0;

// VFR HUD (msg id 74)
int32_t  mav_airspeed = 0;
uint32_t mav_groundspeed = 0;
uint32_t mav_altitude = 0;
uint32_t mav_climb = 0;
int16_t  mav_heading = 0;
uint16_t mav_throttle = 0;

void send_message(mavlink_message_t* msg) {
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];

	uint16_t len = mavlink_msg_to_send_buffer(buf, msg);

	for(uint16_t i = 0; i < len; i++) {
		Serial.write(buf[i]);
	}
}

void send_heartbeat() {
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

	send_message(&msg);
}

void setup() {
	pinMode(ledPin, OUTPUT);
	Serial.begin(57600);
}

void loop() { 
	// Received 5 heartbeats, connection confirmed
	if (hb_count > 5) {
		mavlink_connected = 1;
		digitalWrite(ledPin, HIGH);
	}

	// Do things if we are connected
	if (mavlink_connected > 0)
		send_heartbeat();
	
	// Receive messages and handle them
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
			// Handle message
 			switch(recv_msg.msgid) {
			        case MAVLINK_MSG_ID_HEARTBEAT:
					hb_count++;
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
	}
}
