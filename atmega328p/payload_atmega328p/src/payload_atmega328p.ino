/*
 * Universal Payload Interface
 * Atmega328p MAVLink Code
 */

/* Make sure this is included before Arduino's HardwareSerial */
#include <FastSerial.h>

/* Make sure this is defined before including mavlink.h */
#define MAVLINK_COMM_NUM_BUFFERS 1
#include <mavlink.h>

#define PAYLOAD_SYSTEM_ID 101 // System ID for MAVlink packets

#define STANDBY    0
#define FLASH_LED  1

FastSerialPort0(Serial); // Required for FastSerial

// System Status Variables
uint8_t  mavlink_connected = 0;
uint8_t  pl_state = STANDBY;
uint16_t hb_count = 0;
uint32_t led_pin = A2;

// Payload identifiers
uint8_t  pl_system_id = 0;
uint8_t  pl_component_id = 0;

// APM identifiers
uint8_t  mav_system_id = 0;
uint8_t  mav_component_id = 0;

// *************************
// MAVLINK MESSAGE VARIABLES

// Heartbeat (msg id 1)
uint8_t  mav_type = 0;
uint8_t  mav_autopilot = 0;
uint8_t  mav_base_mode = 0;
uint8_t  mav_system_status = 0;

// Mission Current (msg id 42)
uint16_t mav_seq = 0;

// *************************
// MESSAGE SEND HELPERS

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

	send_message(&msg);
}

void send_set_waypoint(uint16_t wp) {
	mavlink_message_t msg;
	mavlink_mission_set_current_t set_point;

	set_point.seq = wp;
	set_point.target_system = mav_system_id;
	set_point.target_component = mav_component_id;

	mavlink_msg_mission_set_current_encode(pl_system_id, pl_component_id, &msg, &set_point);

	send_message(&msg);
}

// ************************
// PAYLOAD UPDATE FUNCTIONS
void pl_update() {
	static uint16_t frames = 0;
	static int wp = 0;

	// Run payload state machine
	switch (pl_state) {
		case STANDBY:
			// Send command every so often
			if (frames % 1000 == 0) {
				send_set_waypoint(wp);
				wp++;
				if (wp > 3)
					wp = 0;
			}
			digitalWrite(led_pin, HIGH);
			break;
		case FLASH_LED:
			static int timer = 0;

			if (frames % 10 == 0) {
				if (digitalRead(led_pin) == HIGH)
					digitalWrite(led_pin, LOW);
				else
					digitalWrite(led_pin, HIGH);
			}

			if (timer++ > 100)
				pl_state = STANDBY;
			break;
	}
	//delay(100);

	frames++;
}

// ************************
// MAIN LOOP

void setup() {
	pinMode(led_pin, OUTPUT);
	Serial.begin(57600);
}

void loop() { 
	// Received 5 heartbeats, connection confirmed
	if (hb_count == 5) {
		mavlink_connected = 1;
	}

	// If we don't hear from the mav for a while, assume we lost connection
	static uint32_t loss = 0;
	static int hb_last = hb_count;
	if (hb_count == hb_last) {
		loss++;
		if (loss > 50000) {
			mavlink_connected = 0;
			digitalWrite(led_pin, LOW);
			loss = 0;
			hb_count = 0;
		}
	}
	else {
		loss = 0;
		digitalWrite(led_pin, HIGH);
	}
	hb_last = hb_count;

	// Do things if we are connected
	if (mavlink_connected > 0)
		pl_update();
	
	// Receive messages and handle them
	comm_receive();
}

// ************************
// MESSAGE RECEIVE HELPERS

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
			
			// Update system and component id
			mav_system_id = recv_msg.sysid;
			mav_component_id = recv_msg.compid;

			// Handle message
 			switch(recv_msg.msgid) {
			        case MAVLINK_MSG_ID_HEARTBEAT:
					handle_heartbeat(&recv_msg);
			        	break;
				case MAVLINK_MSG_ID_MISSION_CURRENT:
					handle_mission_current(&recv_msg);
					break;
				default:
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

void handle_heartbeat(mavlink_message_t *msg) {
	hb_count++;
	mav_type = mavlink_msg_heartbeat_get_type(msg);
	mav_autopilot = mavlink_msg_heartbeat_get_autopilot(msg);
	mav_base_mode = mavlink_msg_heartbeat_get_base_mode(msg);
	mav_system_status = mavlink_msg_heartbeat_get_system_status(msg);
}

void handle_mission_current(mavlink_message_t *msg) {
	mav_seq = mavlink_msg_mission_current_get_seq(msg);
	pl_state = FLASH_LED;
}
