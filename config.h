/*
 * config.h
 * This file is part of can_to_modbus
 *
 * Copyright (C) 2016 - Trilys
 *
 * can_to_modbus is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * can_to_modbus is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with can_to_modbus. If not, see <http://www.gnu.org/licenses/>.
 */

//Definition for CAN:
#ifdef VCAN
	#define CAN_INTERFACE0 "vcan0"
	#define CAN_INTERFACE1 "vcan1"
#else
	#define CAN_INTERFACE0 "can0"
	#define CAN_INTERFACE1 "can1"
#endif

#define MAX_TIME_CANRC_WAITER 3000000
#define MAX_NUMBER_ID_FILTER 3

#define IP_INTERFACE0 "10.10.10.11"
#define IP_INTERFACE1 "10.10.10.12"

#define IP_PORT0 502
#define IP_PORT1 502

//Definition for CCGX:
#define ADD_CHARGER_CURRENTMAX 2316
#define ADD_CHARGER_ON_OFF 2317
#define ADD_SOLAR_ON_OFF 774
#define ADD_VEBUS_CURRENTMAX 22
#define ADD_VEBUS_STATECHARGE 30
#define ADD_VEBUS_MODE 33 // 1=Charger Only;2=Inverter Only;3=On;4=Off

#define UID_VEBUS 246
#define UID_SOLAR 245

// Find a way to set it dynamically
#define MAX_TOTAL_AC_CURRENT 10.0 // In Amps

// CANA = CAN0 = babord
// Permanent connection to onboard 220V -> tribord

// (actual_info->num_interface[0]) ? 0 : 1