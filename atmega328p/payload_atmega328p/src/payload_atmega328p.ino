/*
 * Universal Payload Interface
 * Atmega328p MAVLink Code
 */

/* Make sure this is included before Arduino's HardwareSerial */
#include <FastSerial.h>

/* Make sure this is defined before including mavlink.h */
#define MAVLINK_COMM_NUM_BUFFERS 1
#include <mavlink.h>

#define PL_SYS_ID  101 // system id of the payload interface
#define PL_COMP_ID 101 // Component id of payload interface

#define NUM_WP 4 // the number of waypoints we will send to the mav

#define PL_STATE_DISCONNECTED       0
#define PL_STATE_CONNECTED          1
#define PL_STATE_SEND_WAYPOINTS     2
#define PL_STATE_FLASH_LED          3

FastSerialPort0(Serial); // Required for FastSerial

// System Status Variables
uint8_t  mavlink_connected = 0;
uint8_t  pl_state = PL_STATE_DISCONNECTED;
uint16_t hb_count = 0;
uint32_t led_pin = 13;

// Payload identifiers
uint8_t  pl_system_id = PL_SYS_ID;
uint8_t  pl_component_id = PL_COMP_ID;

// APM identifiers
uint8_t  mav_system_id = 0;
uint8_t  mav_component_id = 0;

// Waypoint Variables
uint8_t  wp_timeout = 0;
uint8_t  wp_clear_sent = 0;
uint8_t  wp_count_sent = 0;
uint8_t  wp_request_seq = -1;
mavlink_mission_item_t pl_waypoints[NUM_WP];

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
	int system_type = MAV_TYPE_GCS;
	int autopilot_type = MAV_AUTOPILOT_INVALID;
	int base_mode = MAV_MODE_STABILIZE_ARMED;
	int sys_status = MAV_STATE_ACTIVE;
	
	// Initialize the required buffers 
	mavlink_message_t msg; 
	
	// Setup the heartbeat message
	mavlink_msg_heartbeat_pack(pl_system_id, pl_component_id, &msg, system_type, autopilot_type, base_mode, 0, sys_status);

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

void send_waypoint_count(uint16_t c) {
	mavlink_message_t msg;
	mavlink_mission_count_t wp_count;

	wp_count.count = c;
	wp_count.target_system = mav_system_id;
	wp_count.target_component = mav_component_id;

	mavlink_msg_mission_count_encode(pl_system_id, pl_component_id, &msg, &wp_count);

	send_message(&msg);
}

void send_waypoint(float lat, float lon, float alt, uint16_t seq, uint16_t com, uint8_t cur) {
	mavlink_message_t msg;
	mavlink_mission_item_t waypoint;

	waypoint.param1 = 0;
	waypoint.param2 = 0;
	waypoint.param3 = 0;
	waypoint.param4 = 0;
	waypoint.x = lat;
	waypoint.y = lon;
	waypoint.z = alt;
	waypoint.seq = seq;
	waypoint.command = com;
	waypoint.target_system = mav_system_id;
	waypoint.target_component = mav_component_id;
	waypoint.frame = MAV_FRAME_GLOBAL;
	waypoint.current = cur;
	waypoint.autocontinue = 1;

	mavlink_msg_mission_item_encode(pl_system_id, pl_component_id, &msg, &waypoint);

	send_message(&msg);
}

void send_waypoint(mavlink_mission_item_t *wp) {
	mavlink_message_t msg;

	mavlink_msg_mission_item_encode(pl_system_id, pl_component_id, &msg, wp);

	send_message(&msg);
}

void send_clear_waypoints() {
	mavlink_message_t msg;
	mavlink_mission_clear_all_t wp_clear;

	wp_clear.target_system = mav_system_id;
	wp_clear.target_component = mav_component_id;

	mavlink_msg_mission_clear_all_encode(pl_system_id, pl_component_id, &msg, &wp_clear);

	send_message(&msg);
}

void send_status_text(char * txt, MAV_SEVERITY severity) {
	mavlink_message_t msg;
	mavlink_statustext_t status_text;

	status_text.severity = severity;
	memcpy(status_text.text, txt, 50);

	mavlink_msg_statustext_encode(mav_system_id, mav_component_id, &msg, &status_text);

	send_message(&msg);
}

// ************************
// PAYLOAD UPDATE FUNCTIONS

void pl_update() {
	static uint16_t frames = 0;

	// Run payload state machine
	switch (pl_state) {
		case PL_STATE_DISCONNECTED:
			digitalWrite(led_pin, LOW);

			// Received 10 heartbeats, connection confirmed
			if (hb_count >= 10) {
				pl_state = PL_STATE_CONNECTED;

				// update the target ids in the waypoint list
				int i;
				for (i = 0; i < NUM_WP; i++) {
					pl_waypoints[i]->target_system = mav_system_id;
					pl_waypoints[i]->target_component = mav_component_id;
				}
			}

			// send heartbeats from the payload
			delay(10);
			send_heartbeat();

			// while a connection hasn't been established, do nothing
			break;
		case PL_STATE_CONNECTED:
			digitalWrite(led_pin, HIGH);

			if (wp_count_sent == 0) {
				// allow the autopilot to warm up
				delay(5000);

				// clear out any waypoints already on the autopilot
				//send_clear_waypoints();
				//wp_clear_sent = 1;

				pl_state = PL_STATE_SEND_WAYPOINTS;
			}

			// send heartbeats from the payload
			delay(10);
			send_heartbeat();

			break;
		case PL_STATE_FLASH_LED:
			int i;

			// flash the LED 5 times in 1 second to visually acknowledge something
			for (i = 0; i < 5; i++) {
				if (digitalRead(led_pin) == HIGH)
					digitalWrite(led_pin, LOW);
				else
					digitalWrite(led_pin, HIGH);
				delay(200);
			}

			pl_state = PL_STATE_CONNECTED;
			break;
		case PL_STATE_SEND_WAYPOINTS:
			static unsigned long last_time = millis();

			if (wp_count_sent && millis() - last_time >= 2000) {
				last_time = millis();
				wp_timeout++;

				// handle timeouts depending on current request seq
				if (wp_timeout >= 5) {
					wp_count_sent = 0;
					wp_timeout = 0;
				}
				else if (wp_request_seq >= 0) {
					send_waypoint(&pl_waypoints[wp_request_seq]);
				}
				else {
					send_waypoint_count(NUM_WP);
				}
			}
			
			if (wp_count_sent == 0) {
				send_waypoint_count(NUM_WP);
				wp_count_sent++;
			}

			// recv handlers will handle sending waypoints when requested
			break;
	}

	frames++;
}

// ************************
// MAIN SETUP AND LOOP

void init_waypoint(float lat, float lon, float alt, uint16_t seq, uint16_t com, uint8_t cur,
							mavlink_mission_item_t *waypoint) {
	waypoint->param1 = 0;
	waypoint->param2 = 0;
	waypoint->param3 = 0;
	waypoint->param4 = 0;
	waypoint->x = lat;
	waypoint->y = lon;
	waypoint->z = alt;
	waypoint->seq = seq;
	waypoint->command = com;
	waypoint->target_system = mav_system_id;
	waypoint->target_component = mav_component_id;
	waypoint->frame = MAV_FRAME_LOCAL_NED;
	waypoint->current = cur;
	waypoint->autocontinue = 0;
}

void setup() {
	pinMode(led_pin, OUTPUT);
	Serial.begin(57600);

	// Initialize the waypoint list
	//   Modify the init_waypoints in the switch statement and NUM_WP to change waypoints
	int i = 0;
	mavlink_mission_item_t waypoint;
	while (i < NUM_WP) {
		switch (i) {
			case 0:
				init_waypoint(123, 234, 0, i, MAV_CMD_NAV_WAYPOINT, 1, &waypoint);
				break;
			case 1:
				init_waypoint(0, 0, 10, i, MAV_CMD_NAV_TAKEOFF, 0, &waypoint);
				break;
			case 2:
				init_waypoint(123, 234, 20, i, MAV_CMD_NAV_WAYPOINT, 0, &waypoint);
				break;
			case 3:
				init_waypoint(0, 0, 0, i, MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, &waypoint);
				break;
		}
		pl_waypoints[i] = waypoint;
		i++;
	}
}

void loop() { 
	// Carry out payload state machine
	pl_update();
	
	// Receive messages and handle them
	comm_receive();
}

// ************************
// MESSAGE RECEIVE HELPERS

void comm_receive() {
	static unsigned long last_time = millis();
	static uint8_t timeout = 0;
	int i = 0;
	int frames = 0;
	mavlink_message_t recv_msg; 
	mavlink_status_t recv_status;

	// block for 1 second on serial read
	while (Serial.available() == 0 && pl_state != PL_STATE_DISCONNECTED && pl_state != PL_STATE_SEND_WAYPOINTS) {
		if (millis() - last_time >= 1000) {
			last_time = millis();
			timeout++; // block on serial read
		}

		// we haven't heard from the autopilot in 10 seconds
		if (timeout >= 10) {
			pl_state = PL_STATE_DISCONNECTED;
			mavlink_connected = 0;
			hb_count = 0;
			wp_clear_sent = 0;
			wp_count_sent = 0;
			wp_request_seq = -1;
		}
	}
	
	// receive data over serial 
	while (Serial.available() > 0) { 
		uint8_t c = Serial.read();

		//try to get a new message 
		if(mavlink_parse_char(0, c, &recv_msg, &recv_status)) { 
			timeout = 0; // reset timeout
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
				case MAVLINK_MSG_ID_MISSION_REQUEST:
					handle_mission_request(&recv_msg);
					break;
				case MAVLINK_MSG_ID_MISSION_ACK:
					handle_mission_ack(&recv_msg);
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
	//pl_state = PL_STATE_FLASH_LED;
}

void handle_mission_request(mavlink_message_t *msg) {
	wp_timeout = 0;
	wp_request_seq = mavlink_msg_mission_request_get_seq(msg);
	send_waypoint(&pl_waypoints[wp_request_seq]);
}

void handle_mission_ack(mavlink_message_t *msg) {
	wp_request_seq = -1;
	pl_state = PL_STATE_FLASH_LED;
}
