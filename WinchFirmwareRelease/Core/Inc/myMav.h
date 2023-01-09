/*
 * myMav.h
 *
 *  Created on: Nov 2, 2022
 *      Author: Venom
 */

#ifndef INC_MYMAV_H_
#define INC_MYMAV_H_

#include "main.h"

//Current thing vars
_Bool current_affair = false;
uint8_t curr_spike_count = 0;
uint8_t poop_back_count = 0;
__IO uint32_t curr_curr = 0;
uint32_t prev_curr = 0;
uint32_t hover_curr = 0;

mavlink_system_t mavlink_system = {
    21, // System ID (1-255)
    20  // Component ID (a MAV_COMPONENT value)
};

//Mavlink ack variables
_Bool time_to_ack = false;
uint16_t ComID = 0;

/*
 * This is a blocking and non-genric api for battery status
 */

void MavLinkReceiveHoverCurr(UART_HandleTypeDef* huart, uint8_t byte)
{
	mavlink_message_t msg;
	mavlink_status_t status;
	//uint8_t* buff[64] = {0};

	_Bool msg_rcvd = false;

	while(!msg_rcvd)
	{
		HAL_UART_Receive(huart, &byte, 1, HAL_MAX_DELAY);

		if(mavlink_parse_char(MAVLINK_COMM_3, byte, &msg, &status))
		{

			if(msg.msgid == MAVLINK_MSG_ID_BATTERY_STATUS)
			{
				// Get just one field from payload
				mavlink_battery_status_t battery;
				mavlink_msg_battery_status_decode(&msg, &battery);
				hover_curr = battery.current_battery;
				msg_rcvd = true;
			}

		}
	}

}
//Connected to the UART Interrupt. Try to differ the sub-routine to handle the complicated task.
void MavLinkReceive(UART_HandleTypeDef* huart, uint8_t byte)
{
	mavlink_message_t msg;
	mavlink_status_t status;
	uint8_t* buff[64] = {0};

		if(mavlink_parse_char(MAVLINK_COMM_3, byte, &msg, &status))
		{

			switch(msg.msgid)
			{

				case MAVLINK_MSG_ID_HEARTBEAT: // ID for GLOBAL_POSITION_INT
				{
					// Get all fields in payload (into global_position)
					mavlink_heartbeat_t heartbeat;
					mavlink_msg_heartbeat_decode(&msg, &heartbeat);

					sprintf((char*)buff, "Heartbeat Status: %d, TYPE: %d\r\n", heartbeat.system_status, heartbeat.type);
					HAL_UART_Transmit(huart, (uint8_t *)buff, sizeof(buff), HAL_MAX_DELAY);

				}
				break;


				case MAVLINK_MSG_ID_ATTITUDE:
				{
					// Get just one field from payload
					mavlink_attitude_t attitude;
					mavlink_msg_attitude_decode(&msg, &attitude);

					//sprintf((char*)buff, "Pitch Agnle: %f\r\n", RAD2DEG(attitude.pitch));
					//HAL_UART_Transmit(huart, (uint8_t *)buff, sizeof(buff), HAL_MAX_DELAY);

				}
				break;


				case MAVLINK_MSG_ID_BATTERY_STATUS:
				{
					// Get just one field from payload
					mavlink_battery_status_t battery;
					mavlink_msg_battery_status_decode(&msg, &battery);

					curr_curr = battery.current_battery;
				}
				break;



				case MAVLINK_MSG_ID_COMMAND_ACK:
				{

					mavlink_command_ack_t cmdack;
					mavlink_msg_command_ack_decode(&msg, &cmdack);

					sprintf((char*)buff, "Do CMD ACK: %d\r\n", cmdack.command);
					HAL_UART_Transmit(huart, (uint8_t *)buff, sizeof(buff), HAL_MAX_DELAY);
				}
				break;


				case MAVLINK_MSG_ID_COMMAND_LONG:
				{

					mavlink_command_long_t command_long;
					mavlink_msg_command_long_decode(&msg, &command_long);

					int command = command_long.command;

					switch(command)
					{

						case MAV_CMD_DO_WINCH:
						{
							//Set the winch flag and run the winch sequence.
							time_to_ack = true;
						}
						break;

						case MAV_CMD_DO_SET_SERVO:
						{
								command = 42;
						}

						break;

						default:
							break;

				  }

				}


				break;

				case MAVLINK_MSG_ID_COMMAND_INT:
				{

					mavlink_command_int_t command_int;
					mavlink_msg_command_int_decode(&msg, &command_int);

					switch(command_int.command)
					{

					case MAV_CMD_DO_WINCH:
					{
						//Set the winch flag and run the winch sequence.
					}
					break;

					case MAV_CMD_DO_SET_SERVO:
					{

						sprintf((char*)buff, "Do Set Servo CMD Status: %f, TYPE: %f\r\n", command_int.param1, command_int.param2);
						HAL_UART_Transmit(huart, (uint8_t *)buff, sizeof(buff), HAL_MAX_DELAY);

						//Send an ack back to the sender via the MAV_CMD_ACK
						//Make sure you write a re-entrant function.
						time_to_ack = true;
						ComID = command_int.command;
					}


					default:
						break;

				}
			}
			break;

			default:
			break;

			}

		}

		else
		{

		}


}

uint8_t MavLinkTransfer()
{
	mavlink_message_t msg;
	uint8_t len = 0;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];

	mavlink_msg_param_request_read_pack(mavlink_system.sysid, mavlink_system.compid, &msg, 1, 1, "SYSID_SW_TYPE", -1);
	len = mavlink_msg_to_send_buffer(buf, &msg);

	return len;
	//Mav packet is ready to be transferred.
}


uint8_t MavLinkRequestData(UART_HandleTypeDef* huart)
{
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];

	mavlink_msg_request_data_stream_pack(mavlink_system.sysid, mavlink_system.compid, &msg, 1, 1, MAV_DATA_STREAM_ALL, 1, 1);
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);


	HAL_UART_Transmit(huart, (uint8_t*)buf, len, HAL_MAX_DELAY);

	return len;
	//Mav packet is ready to be transferred.


}


uint8_t MavlinkHeartBeatSend(UART_HandleTypeDef* huart)
{

	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];

	//int sysid = 20;                   ///< ID 20 for this airplane. 1 PX, 255 ground station
	//int compid = 158;                ///< The component sending the message
	int type = MAV_TYPE_QUADROTOR;   ///< This system is an airplane / fixed wing

	// Define the system type, in this case an airplane -> on-board controller
	//uint8_t system_type = MAV_TYPE_GENERIC;
	uint8_t autopilot_type = MAV_AUTOPILOT_PX4;

	uint8_t system_mode = MAV_MODE_PREFLIGHT; ///< Booting up
	uint32_t custom_mode = 100;                 ///< Custom mode, can be defined by user/adopter
	uint8_t system_state = MAV_STATE_STANDBY; ///< System ready for flight

	// Pack the message
	mavlink_msg_heartbeat_pack(21,20, &msg, type, autopilot_type, system_mode, custom_mode, system_state);

	// Copy the message to the send buffer
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

	HAL_Delay(2000);

	HAL_UART_Transmit(huart, (uint8_t*)buf, len, HAL_MAX_DELAY);

	return len;


}


uint8_t MavlinkDoCMDSend(UART_HandleTypeDef* huart)
{

	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];

	// Pack the message
	//mavlink_msg_heartbeat_pack(1,0, &msg, type, autopilot_type, system_mode, custom_mode, system_state);
	mavlink_msg_command_int_pack(21, 20, &msg, 1, MAV_COMP_ID_AUTOPILOT1, MAV_FRAME_GLOBAL, MAV_CMD_DO_WINCH, 0, 0, 1, 1, 20, 1, 1, 1, -20);
	// Copy the message to the send buffer
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

	HAL_Delay(2000);

	HAL_UART_Transmit(huart, (uint8_t*)buf, len, HAL_MAX_DELAY);

	return len;


}


uint8_t MavlinkWinchStatus(UART_HandleTypeDef* huart)
{

	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];

	// Pack the message
	mavlink_msg_winch_status_pack(1, MAV_COMP_ID_USER1, &msg, uwTick, 20, 2, 1, 12, 5, 20, 1);

	// Copy the message to the send buffer
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

	HAL_Delay(2000);

	HAL_UART_Transmit(huart, (uint8_t*)buf, len, HAL_MAX_DELAY);

	return len;


}

uint8_t MavlinkCmdAcknowledge(UART_HandleTypeDef* huart, uint8_t command_id, uint8_t target_system, uint8_t target_component)
{

	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];

	uint8_t result = MAV_RESULT_ACCEPTED;
	uint8_t progress = result; //For simple cmds.

	// Pack the message
	mavlink_msg_command_ack_pack(21, 20, &msg, MAV_CMD_DO_WINCH, result, progress, 0, 1, MAV_COMP_ID_ALL);

	// Copy the message to the send buffer
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

	HAL_UART_Transmit(huart, (uint8_t*)buf, len, HAL_MAX_DELAY);

	return len;


}

#endif /* INC_MYMAV_H_ */
