 /*
  * can_min.c
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <signal.h>

#include <sys/socket.h> // for sa_family_t 
#include <linux/can.h>
#include <linux/can/error.h>

//Include for CAN
#include <sys/ioctl.h>
#include <unistd.h>
#include <net/if.h>
#include <linux/can/raw.h>

#include "config.h"

#include "can_min.h"

char CAN_send(char num_can_interface, long id, char *data_to_send, char sizeOfData)
{
#ifdef DEBUG
	printf("\nIn CAN_send");
#endif
	int s; // can raw socket
	int required_mtu;
	struct sockaddr_can addr;
	struct canfd_frame frame_to_send;
	struct ifreq ifr;
	int i;
#ifdef DEBUG
	printf("\ndata_to_send (%u) = %02lx#", sizeOfData, id);
	for (i = 0; i < sizeOfData; i += 1) {
		printf("%02hhx", data_to_send[i]);
	}
#endif

	//Init frame_to_send_send
	frame_to_send.can_id = id;
	frame_to_send.len = sizeOfData;
	frame_to_send.flags = 0;
	for (i = 0; i < frame_to_send.len; i += 1)
	{
		frame_to_send.data[i] = data_to_send[i];
	}
	required_mtu = 16;

	// open socket
	if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		perror("socket");
		return 1;
	}

	if (num_can_interface == 0) {
		strncpy(ifr.ifr_name, CAN_INTERFACE0, IFNAMSIZ - 1);
	} else {
		strncpy(ifr.ifr_name, CAN_INTERFACE1, IFNAMSIZ - 1);
	}
	ifr.ifr_name[IFNAMSIZ - 1] = '\0';
	ifr.ifr_ifindex = if_nametoindex(ifr.ifr_name);
	if (!ifr.ifr_ifindex) {
		perror("if_nametoindex");
		return 1;
	}

	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);

	if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		perror("bind");
		return 1;
	}

	// send frame_to_send 
	if (write(s, &frame_to_send, required_mtu) != required_mtu) {
		perror("write");
		return 1;
	}
	close(s);
	return 0;
}

static volatile char running = 1;

void sigterm(int signo)
{
	running = 0;
}

//Receive CAN from canIDtoFilter marked as decimal.
char CAN_receive(unsigned char num_can_interface, unsigned char sizeOfFilter, long canIDtoFilter[], long delayToOffInUs, long *canIdReceived, char canReceived[], char *sizeOfCAN){
	unsigned char i;
#ifdef DEBUG
	printf("\nCAN_receive : interface:%hx, sizeID:%lx, delay:%ums, filters:", num_can_interface, sizeOfFilter, delayToOffInUs);
	for (i = 0; i < sizeOfFilter; i += 1) {
		printf("%04hx,",canIDtoFilter[i]);
	}
#endif
	running = 1;	//Ne pas mettre en commentaire, en fait c'est utile…
	fd_set rdfs;
	int s_sock;
	int ret;
	struct sockaddr_can addr;
	char ctrlmsg[CMSG_SPACE(sizeof(struct timeval)) + CMSG_SPACE(sizeof(__u32))];
	struct iovec iov;
	struct msghdr msg;
	struct cmsghdr *cmsg;
	struct can_filter *rfilter;
	struct canfd_frame frame;
	char nbytes;
	struct ifreq ifr;
	struct timeval timeout, *timeout_current = NULL, timeout_config = { 0, 0 };
	
	signal(SIGTERM, sigterm);
	signal(SIGHUP, sigterm);
	signal(SIGINT, sigterm);
	//Init time to wait before exit
	timeout_config.tv_usec = delayToOffInUs;
	timeout_config.tv_sec = timeout_config.tv_usec / 1000;
	timeout_config.tv_usec = (timeout_config.tv_usec % 1000) * 1000;
	timeout_current = &timeout;

#ifdef DEBUG
	if (num_can_interface == 0) {
		printf("open %d '%s'. Delay = %ums\n", 0, CAN_INTERFACE0, delayToOffInUs);
	} else {
		printf("open %d '%s'. Delay = %ums\n", 0, CAN_INTERFACE1, delayToOffInUs);
	}
#endif

	s_sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
	if (s_sock < 0) {
		perror("socket");
		return 1;
	}

	//Use interface can_interface
	if (num_can_interface == 0) {
		strncpy(ifr.ifr_name, CAN_INTERFACE0, IFNAMSIZ - 1);
	} else {
		strncpy(ifr.ifr_name, CAN_INTERFACE1, IFNAMSIZ - 1);
	}
	ifr.ifr_name[IFNAMSIZ - 1] = '\0';
	ifr.ifr_ifindex = if_nametoindex(ifr.ifr_name);
	if (!ifr.ifr_ifindex) {
		perror("if_nametoindex");
		return 1;
	}

#ifdef DEBUG
	printf("using interface name '%s'.\n", ifr.ifr_name);
#endif
	if (ioctl(s_sock, SIOCGIFINDEX, &ifr) < 0) {
		perror("SIOCGIFINDEX");
		return 1;
	}
	addr.can_ifindex = ifr.ifr_ifindex;

	rfilter = malloc(sizeof(struct can_filter) * sizeOfFilter);
	if (!rfilter) {
		fprintf(stderr, "Failed to create filter space!\n");
		return 1;
	}
	// Create filter to get only what we need
	for (i = 0; i < sizeOfFilter; i += 1) {
		rfilter[i].can_id=canIDtoFilter[i];
		rfilter[i].can_mask=2047;
	}
	setsockopt(s_sock, SOL_CAN_RAW, CAN_RAW_FILTER, rfilter, sizeof(struct can_filter) * sizeOfFilter);
	free(rfilter);
	if (bind(s_sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		perror("bind");
		return 1;
	}

	// these settings are static and can be held out of the hot path 
	iov.iov_base = &frame;
	msg.msg_name = &addr;
	msg.msg_iov = &iov;
	msg.msg_iovlen = 1;
	msg.msg_control = &ctrlmsg;

	while (running) {
		FD_ZERO(&rdfs);
		FD_SET(s_sock, &rdfs);

		if (timeout_current)
			*timeout_current = timeout_config;

		if ((ret = select(s_sock+1, &rdfs, NULL, NULL, timeout_current)) <= 0) {
			//perror("select");
			fprintf(stderr, "\nTrilys: END01 due to timeout\n");
			running = 0;
			*sizeOfCAN = 0;
			continue;
		}

		//If CAN/filtered detected
		if (FD_ISSET(s_sock, &rdfs)) {
			// these settings may be modified by recvmsg()
			iov.iov_len = sizeof(frame);
			msg.msg_namelen = sizeof(addr);
			msg.msg_controllen = sizeof(ctrlmsg);  
			msg.msg_flags = 0;

			nbytes = recvmsg(s_sock, &msg, 0);
			if (nbytes < 0) {
				perror("read");
				return 1;
			}
#ifdef DEBUG
	printf("\nTest100: frame.len=%u, .can_id=%lx, .data=", frame.len,frame.can_id);
	for (i = 0; i < frame.len; i += 1) {
		printf(".%02hhx", frame.data[i]);
	}
	printf("\n");
#endif
			*sizeOfCAN = frame.len;
			*canIdReceived=frame.can_id;
			for (i = 0; i < *sizeOfCAN; i += 1) {
				canReceived[i] = frame.data[i];
			}
			running = 0;
		}
		fflush(stdout);
	}
	close(s_sock);
	return 0;
}


