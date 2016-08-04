/*
 * can_min.h
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

//Send the first sizeOfData bytes of data_to_send to id.
//Return 0 if ok
//ex : CAN_send(0x600,CAN, 8); or CAN_send(0x601,"\x4b\x20\x29\x01\xe8\x03\x00\x00", 8);
char CAN_send(char num_can_interface, long id, char *data_to_send, char sizeOfData);

//Receive CAN from canIDtoFilter marked as decimal.
//Return 0 if ok
char CAN_receive(unsigned char num_can_interface, unsigned char sizeOfFilter, long canIDtoFilter[], long delayToOffInUs, long *canIdReceived, char canReceived[], char *sizeOfCAN);

