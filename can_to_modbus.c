/*
 * CAN_to_ModBus.c
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
 * 
 * To compile this file:
 * gcc can_min.c lib_can.c can_to_modbus.c -o can_to_modbus -lpthread (-D VCAN -D DEBUG) `pkg-config --libs --cflags libmodbus`
 * 
 * This program manage 3 threads, two will receive information from 2 CAN
 * then send data to each modbus interface and the last is the loop main.
 * 
 */

#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>

//Catch ^C
#include <signal.h>

//Access to CAN interfaces
#include "can_min.h"

//Access to GPIO
#include "gpio.h"

//Get modbus library
#include <modbus.h>

//Get configuration file
#include "config.h"

//Timer to exit correctly, when ^C pressed
#include <sys/time.h>

//Sleep during loop to slow CPU rate
#include <unistd.h>

typedef struct {
	unsigned long *canIdReceived;
	unsigned char *length;
	unsigned char *data;
	unsigned char *num_interface;
	unsigned long *canIDtoFilter;
	unsigned char *sizeOfFilter;
} s_info_CAN;

static volatile char local_running = 1;
volatile float vmax[2] = {3.4, 3.4};
volatile int16_t current_max[2] = {0, 0}; //as to be sent to Color control, in A/10 (123=12.3A)

void local_sigterm(int signo)
{
	if (local_running == 0) {
		printf("\nViolent kill...\n");
		exit(0);
	}
	printf("\ncan_to_modbus killed : %d\n", signo);
	local_running = 0;
}

//Thread for receiving CAN information then send through modbus
static void * t_CAN_RC (void * p_data)
{
	unsigned char i;
	s_info_CAN *actual_info = p_data;
	
	int soc, outModbus, relay;
	
	modbus_t *mb;
	if (actual_info->num_interface[0] == 0) {
		mb = modbus_new_tcp(IP_INTERFACE0, IP_PORT0);
	} else {
		mb = modbus_new_tcp(IP_INTERFACE1, IP_PORT1);
	}
	modbus_connect(mb);

#ifdef DEBUG
	printf("\nCAN_RC : Try to get ID=");
	for (i = 0; i < actual_info->sizeOfFilter[0]; i += 1) {
		printf("%04hx,",actual_info->canIDtoFilter[i]);
	}
#endif
	for (;local_running;) {
/*		actual_info->length[0]=250;*/
		actual_info->canIdReceived[0] = 0;
		if(CAN_receive(actual_info->num_interface[0], actual_info->sizeOfFilter[0], actual_info->canIDtoFilter, MAX_TIME_CANRC_WAITER, &actual_info->canIdReceived[0], actual_info->data, &actual_info->length[0]) || actual_info->length[0] == 0){
			printf("\n/!\\/!\\/!\\CAN_RC error, nothing received on interface %02hx/!\\/!\\/!\\", actual_info->num_interface[0]);
		} else {
			#ifdef DEBUG
				printf("\nDBG : CAN_receive : interface=%hx, ID=%04hx, length=%d, data=",actual_info->num_interface[0], actual_info->canIdReceived[0], actual_info->length[0]);
				for (i = 0; i < actual_info->length[0]; i += 1) {
					printf("%02hhx ", actual_info->data[i]);
				}
			#endif
			//% bat : 10f8159e : 3ème byte : 0à255
			//Send 1 register through modbus.
			//int modbus_write_register(modbus_t *ctx, int addr, int value);
			if ((actual_info->canIdReceived[0] & 0x7fFFffFF) == 0x10f8159e) {	//SOC
				static int slow = 0;
				if (slow >= 10){
					slow = 0;
				soc = actual_info->data[2]*10;
				relay = ( (actual_info->data[7] & 0x10) == 0x10); // high power relay closed if 8th byte is 0x1F
				#ifdef DEBUG
					printf("\nDBG : SOC received : %02hx=%d",actual_info->data[2],soc);
					printf("\nDBG : relay received : %02hx",actual_info->data[7]);
				#endif
				modbus_set_slave(mb, UID_VEBUS);
				outModbus=modbus_write_register(mb, ADD_VEBUS_STATECHARGE, soc);
				if (outModbus == -1) {	//Si non envoyé : se reconnecter puis réenvoyer.
					printf("\n---------------------------------------");
					printf("\n-------------ERREUR - SOC--------------");
					printf("\n----Le serveur n'à pas envoyé d'ACK----");
					printf("\n--------Reconnection necessaire--------");
					printf("\n---------------------------------------");
					modbus_close(mb);
					modbus_connect(mb);
					outModbus=modbus_write_register(mb, ADD_VEBUS_STATECHARGE, soc);
				}
				#ifdef DEBUG
					printf("\noutModBus=%d", outModbus);
				#endif
				if (!relay){
					outModbus = modbus_write_register(mb, ADD_VEBUS_MODE, 4); //Off, test with 2: inverter only
					modbus_set_slave(mb, UID_SOLAR);
					outModbus = modbus_write_register(mb, ADD_SOLAR_ON_OFF, 0);
				}
				else{
					// outModbus = modbus_write_register(mb, ADD_VEBUS_MODE, 3); //On
				}
				if (outModbus == -1) {	//Si non envoyé : se reconnecter puis réenvoyer.
					printf("\n---------------------------------------");
					printf("\n------------ERREUR - Relay-------------");
					printf("\n----Le serveur n'à pas envoyé d'ACK----");
					printf("\n--------Reconnection necessaire--------");
					printf("\n---------------------------------------");
					modbus_close(mb);
					modbus_connect(mb);
					if (!relay){
						modbus_set_slave(mb, UID_SOLAR);
						outModbus = modbus_write_register(mb, ADD_SOLAR_ON_OFF, 0);
						modbus_set_slave(mb, UID_VEBUS);
						outModbus = modbus_write_register(mb, ADD_VEBUS_MODE, 4); //Off
					}
					else{
						// outModbus = modbus_write_register(mb, ADD_VEBUS_MODE, 3); //On
					}
				}
				}
				else slow++;
				#ifdef DEBUG
					printf("\noutModBus=%d", outModbus);
				#endif
			} else if ((actual_info->canIdReceived[0] & 0x7fFFffFF) == 0x10f8169e) {	//current_max
				// current_max = ((actual_info->data[5]+256*actual_info->data[6])/20.0 - 1600)/5;
				// #ifdef DEBUG
					// printf("\nDBG : current_max received : %02hx%02hx=%f",actual_info->data[5],actual_info->data[6],current_max);
				// #endif
				// outModbus = modbus_write_register(mb, ADD_CHARGER_CURRENTMAX, current_max);
				// if (outModbus == -1) {	//Si non envoyé : se reconnecter puis réenvoyer.
					// printf("\n---------------------------------------");
					// printf("\n-------------ERREUR - IMAX-------------");
					// printf("\n----Le serveur n'a pas envoyé d'ACK----");
					// printf("\n--------Reconnection nécessaire--------");
					// printf("\n---------------------------------------");
					// modbus_close(mb);
					// modbus_connect(mb);
					// outModbus = modbus_write_register(mb, ADD_CHARGER_CURRENTMAX, current_max);
				// }
				// //outModbus=modbus_write_register(mb, ADD_VEBUS_CURRENTMAX, current_max);
			} else if((actual_info->canIdReceived[0] & 0x7fFFffFF) == 0x18f81f9e) {	//Vmax
				vmax[actual_info->num_interface[0]] = (actual_info->data[0]+256*actual_info->data[1])/1000.0;
				modbus_set_slave(mb, UID_SOLAR);
				if (vmax[actual_info->num_interface[0]] > 3.35) {
					outModbus = modbus_write_register(mb, ADD_SOLAR_ON_OFF, 0);
				} else if ( (vmax[actual_info->num_interface[0]] < 3.28) && relay ) {
					outModbus = modbus_write_register(mb, ADD_SOLAR_ON_OFF, 1);
				}
				if (outModbus == -1) {	//Si non envoyé : se reconnecter puis réenvoyer.
					printf("\n---------------------------------------");
					printf("\n---------ERREUR - VMAX SOLAR-----------");
					printf("\n----Le serveur n'a pas envoyé d'ACK----");
					printf("\n--------Reconnection nécessaire--------");
					printf("\n---------------------------------------");
					modbus_close(mb);
					modbus_connect(mb);
					modbus_set_slave(mb, UID_SOLAR);
					if (vmax[actual_info->num_interface[0]] > 3.45) {
						outModbus = modbus_write_register(mb, ADD_SOLAR_ON_OFF, 0);
					} else if ( (vmax[actual_info->num_interface[0]] < 3.38) && relay ) {
						outModbus = modbus_write_register(mb, ADD_SOLAR_ON_OFF, 1);
					}
				}
				
				// ---------------- 2018/01/19 added optos ----------------
				// ------------ end charge of multiplus using optos -------
				static int gpio_pin[2] = {70,77};
				int read_value = 0;
				//act on gpio corresponding to received message
				gpio_get_value(gpio_pin[actual_info->num_interface[0]], &read_value);
				//printf("\nDBG : interface : %d",actual_info->num_interface[0]);
				//printf("\nDBG : vmax : %.2f",vmax[actual_info->num_interface[0]]);
				//printf("\nDBG : read_value : %d",read_value);
				if (vmax[actual_info->num_interface[0]] > 3.55){
					gpio_set_value(gpio_pin[actual_info->num_interface[0]],1); //stop charge (input contact on victron)
				}else if (vmax[actual_info->num_interface[0]] <= 3.35){
					gpio_set_value(gpio_pin[actual_info->num_interface[0]],0); //resume charge
				}

				
				
				// if (vmax[actual_info->num_interface[0]] > 3.47) {
					// if ( (current_max[actual_info->num_interface[0]]) >= ((3.57 - vmax[actual_info->num_interface[0]])*1000) ){
						// current_max[actual_info->num_interface[0]] -= 5;
					// }
					// else if ( (current_max[actual_info->num_interface[0]]) <= ((3.55 - vmax[actual_info->num_interface[0]])*1000) ){
						// if ( (current_max[0] + current_max[1]) < (MAX_TOTAL_AC_CURRENT * 10) ){
							// current_max[actual_info->num_interface[0]] += 1;
						// }
					// }
					// if (current_max[actual_info->num_interface[0]] < 0){
						// current_max[actual_info->num_interface[0]] = 0;
					// }
				// } else if ( vmax[actual_info->num_interface[0]] < 3.47 ) {
					// if ( (current_max[0] + current_max[1]) < (MAX_TOTAL_AC_CURRENT * 10) ){
						// current_max[actual_info->num_interface[0]] += 5;
					// }
					// else if ( vmax[actual_info->num_interface[0]] < 3.3 && ( (vmax[(actual_info->num_interface[0]) ? 0 : 1] - vmax[actual_info->num_interface[0]]) > 0.05) ){
						// if (current_max[(actual_info->num_interface[0]) ? 0 : 1] > 5){
							// current_max[actual_info->num_interface[0]] += 5;
							// current_max[(actual_info->num_interface[0]) ? 0 : 1] -= 5;
						// }
					// }
				// }
				// modbus_set_slave(mb, UID_VEBUS);
				// outModbus = modbus_write_register(mb, ADD_VEBUS_CURRENTMAX, current_max[actual_info->num_interface[0]]);
				// printf("\nDBG : current sent : %d",current_max[actual_info->num_interface[0]]);
				// if (outModbus == -1) {	//Si non envoyé : se reconnecter puis réenvoyer.
					// printf("\n---------------------------------------");
					// printf("\n--------ERREUR - VMAX CHARGER----------");
					// printf("\n----Le serveur n'a pas envoyé d'ACK----");
					// printf("\n--------Reconnection nécessaire--------");
					// printf("\n---------------------------------------");
					// modbus_close(mb);
					// modbus_connect(mb);
					// modbus_set_slave(mb, UID_VEBUS);
					// outModbus = modbus_write_register(mb, ADD_VEBUS_CURRENTMAX, current_max[actual_info->num_interface[0]]);
				// }
				// #ifdef DEBUG
					// printf("\nDBG : V_max received : %02hx%02hx=%f",actual_info->data[0],actual_info->data[1],vmax);
				// #endif
			}
		}
	}
	free(actual_info);
	modbus_close(mb);
	modbus_free(mb);
	return NULL;
}

//Get data from CAN then send them through Modbus
int main (void)
{
#ifdef DEBUG
	printf ("\n+----------------------------------------------+");
	printf ("\n|    TRILYS_CAN_TO_MODBUS - DEBUG ACTIVATED    |");
	printf ("\n+----------------------------------------------+\n");
#endif
	int i = 0;
	int ret0 = 0, ret1 = 0;
	pthread_t th_can0_receiver;
	pthread_t th_can1_receiver;

	//Init CAN information which need to be received:
	unsigned long can0_id_received;
	unsigned char can0_length;
	unsigned char can0_data[8];
	unsigned char can0_num_interface;
	unsigned long can0_id_filtered[MAX_NUMBER_ID_FILTER];
	unsigned char can0_size_filter;
	
	s_info_CAN *info_can0 = malloc(sizeof *info_can0);
	info_can0->canIdReceived = &can0_id_received;
	info_can0->length = &can0_length;
	info_can0->data = can0_data;
	info_can0->num_interface = &can0_num_interface;
	info_can0->canIDtoFilter = can0_id_filtered;
	info_can0->sizeOfFilter = &can0_size_filter;
	
	//Now we can work with can_id_received, length and data[] as local variable but there are shared with main and t_CAN_RC
//	Exemple :
//	can_id_received = 1234;
//	can_length = 2;
//	can_data[0]=0x56;
//	can_data[1]=0x78;

	//Init CAN1 information which need to be received:
	unsigned long can1_id_received;
	unsigned char can1_length;
	unsigned char can1_data[8];
	unsigned char can1_num_interface;
	unsigned long can1_id_filtered[MAX_NUMBER_ID_FILTER];
	unsigned char can1_size_filter;
	
	s_info_CAN *info_can1 = malloc(sizeof *info_can1);
	info_can1->canIdReceived = &can1_id_received;
	info_can1->length = &can1_length;
	info_can1->data = can1_data;
	info_can1->num_interface = &can1_num_interface;
	info_can1->canIDtoFilter = can1_id_filtered;
	info_can1->sizeOfFilter = &can1_size_filter;

	//Init end of prog
	signal(SIGTERM, local_sigterm);
	signal(SIGHUP, local_sigterm);
	signal(SIGINT, local_sigterm);

	//Init can0 and can1
	can0_num_interface = 0;
	can0_id_filtered[0] = 0x10f8159e;	//SOC,% [2]
	can0_id_filtered[1] = 0x10f8169e;	//CurrentMAX in battery ([5]+256×[6])÷20 - 1600
	can0_id_filtered[2] = 0x18f81f9e;	//Vmax ([0]+256×[1])÷1000 V
	can0_size_filter = 3;
	
	can1_num_interface = 1;
	can1_id_filtered[0] = 0x10f8159e;
	can1_id_filtered[1] = 0x10f8169e;
	can1_id_filtered[2] = 0x18f81f9e;
	can1_size_filter = 3;
	

	//Create thread CAN_RC waiter
	ret0 = pthread_create (&th_can0_receiver, NULL, t_CAN_RC, info_can0);
	if (! ret0) {
		#ifdef DEBUG
			printf ("\nCreation du thread CAN0 réussie !");
		#endif
	} else {
		printf ("\n/!\\/!\\/!\\Erreur de creation thread/!\\/!\\/!\\\n");
		free(info_can0);
		free(info_can1);
		return -1;
	}
	ret1 = pthread_create (&th_can1_receiver, NULL, t_CAN_RC, info_can1);
	if (! ret1) {
		#ifdef DEBUG
			printf ("\nCreation du thread CAN1 réussie !");
		#endif
	} else {
		printf ("\n/!\\/!\\/!\\Erreur de creation thread/!\\/!\\/!\\\n");
		free(info_can0);
		free(info_can1);
		return -1;
	}
	struct timeval t0, t1;
	long diff;
	
	//Useless loop
	for (;local_running;) {
		#ifdef DEBUG
			printf ("\nlocal_running = %u", local_running);
		#endif
		gettimeofday(&t0, 0);
		usleep(10000000);
		gettimeofday(&t1, 0);
		diff = (t1.tv_sec-t0.tv_sec);
		if (diff < 10) {
			local_sigterm(0);
		}
	}

	free(info_can0);
	free(info_can1);

	printf("\n\n-------End of program-------\n\n");
	return EXIT_SUCCESS;
}

