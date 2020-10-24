#include "../include/simulator.h"

/* ******************************************************************
   ALTERNATING BIT AND GO-BACK-N NETWORK EMULATOR: VERSION 1.1  J.F.Kurose

   This code should be used for PA2, unidirectional data transfer 
   protocols (from A to B). Network properties:
   - one way network delay averages five time units (longer if there
   are other messages in the channel for GBN), but can be larger
   - packets can be corrupted (either the header or the data portion)
   or lost, according to user-defined probabilities
   - packets will be delivered in the order in which they were sent
   (although some can be lost).
 **********************************************************************/

#include<stdio.h>
#include<string.h>

#define A 0
#define B 1
#define TRUE 1
#define FALSE 0
#define BUFFER_SIZE 1000
#define MSG_LEN 20
#define RTT 10
#define min(a,b) (a < b? a:b)

int A_base;
int A_nextseqnum;
int A_npkts;
int A_buflen;
int A_winsize;
float A_timerval;

int B_expseqnum;

struct A_dtype{
	struct pkt packet;
	float start_time;
};

struct A_dtype A_buffer[BUFFER_SIZE];

int compute_checksum(int seqnum, int acknum, char *payload);
int validate_checksum(struct pkt packet);

/********* STUDENTS WRITE THE NEXT SIX ROUTINES *********/

/* called from layer 5, passed the data to be sent to other side */
void A_output(message)
	struct msg message;
{
	// printf("A - REQ TO SEND msg:%s at time:%f\n\n", message.data, get_sim_time());

	/* making a packet for the message and storing it in a local buffer */
	A_buffer[A_npkts].packet.seqnum = A_npkts;
	A_buffer[A_npkts].packet.acknum = 1;
	message.data[MSG_LEN] = '\0';
	strcpy(A_buffer[A_npkts].packet.payload, message.data);
	A_buffer[A_npkts].packet.checksum = compute_checksum(A_buffer[A_npkts].packet.seqnum, A_buffer[A_npkts].packet.acknum, A_buffer[A_npkts].packet.payload);
	A_buflen++;
	A_npkts++;

	/* sending the packet if it falls in the current window */
	if (A_nextseqnum < A_base + A_winsize){
		// printf("A - SENT seq:%d ack:%d cs:%d payload:%s at time:%f\n", A_buffer[A_nextseqnum].packet.seqnum, A_buffer[A_nextseqnum].packet.acknum, A_buffer[A_nextseqnum].packet.checksum, A_buffer[A_nextseqnum].packet.payload, get_sim_time());
		tolayer3(A, A_buffer[A_nextseqnum].packet);
		A_buffer[A_nextseqnum].start_time = get_sim_time();
		if (A_nextseqnum == A_base){
			// printf("timer started for %f units\n", A_timerval);
			starttimer(A, A_timerval);
		}
		A_nextseqnum++;
	}
}

/* called from layer 3, when a packet arrives for layer 4 */
void A_input(packet)
	struct pkt packet;
{
	// printf("A - RECV seq:%d ack:%d cs:%d payload:%s at time:%f\n", packet.seqnum, packet.acknum, packet.checksum, packet.payload, get_sim_time());

	/* validating the checksum */
	if (!validate_checksum(packet)){
		// printf("corrupted ACK\n");
		return;
	}

	/* ignoring duplicate acknowledgements */
	if (packet.acknum < A_base){
		// printf("duplicate ACK\n");
		return;
	}

	/* moving sender base to the right and updating the timer */
	int prevbase = A_base;
	A_base = packet.acknum + 1;
	if (A_base == A_nextseqnum){
		/* stopping the timer */
		// printf("timer stopped\n");
		stoptimer(A);
	}
	else {
		/* restarting the timer */
		stoptimer(A);
		float timerval = A_timerval - (get_sim_time() - A_buffer[A_base].start_time);
		// printf("timer restarted for %f units\n", timerval);
		starttimer(A, timerval);
	}
	A_buflen -= A_base - prevbase;

	/* transmitting next packets (if any) in buffer if they fall in the current window */
	if (A_buflen > 0){
		int i;
		for (i = A_nextseqnum; i < min(A_npkts, A_base + A_winsize); i++){
			// printf("A - SENT seq:%d ack:%d cs:%d payload:%s at time:%f\n", A_buffer[i].packet.seqnum, A_buffer[i].packet.acknum, A_buffer[i].packet.checksum, A_buffer[i].packet.payload, get_sim_time());
			tolayer3(A, A_buffer[i].packet);
			A_buffer[i].start_time = get_sim_time();
			if (i == A_base){
				// printf("timer started for %f units\n", A_timerval);
				starttimer(A, A_timerval);
			}
			A_nextseqnum++;
		}
	}
}

/* called when A's timer goes off */
void A_timerinterrupt()
{
	// printf("timer expired for packet number %d\n", A_base);

	/* retransmitting all the packets in the window */
	int i;
	float curr_time = get_sim_time();
	for (i = A_base; i < A_nextseqnum; i++){
		// printf("A - SENT seq:%d ack:%d cs:%d payload:%s at time:%f\n", A_buffer[i].packet.seqnum, A_buffer[i].packet.acknum, A_buffer[i].packet.checksum, A_buffer[i].packet.payload, curr_time);
		A_buffer[i].start_time = curr_time;
		tolayer3(A, A_buffer[i].packet);
		if (i == A_base){
			// printf("timer started for %f units\n", A_timerval);
			starttimer(A, A_timerval);
		}
	}
}

/* the following routine will be called once (only) before any other */
/* entity A routines are called. You can use it to do any initialization */
void A_init()
{
	A_base = 0;
	A_nextseqnum = 0;
	A_npkts = 0;
	A_buflen = 0;
	A_winsize = getwinsize();
	A_timerval = 2*RTT;
}

/* Note that with simplex transfer from a-to-B, there is no B_output() */

/* called from layer 3, when a packet arrives for layer 4 at B*/
void B_input(packet)
	struct pkt packet;
{
	// printf("B - RECV seq:%d ack:%d cs:%d payload:%s at time:%f\n", packet.seqnum, packet.acknum, packet.checksum, packet.payload, get_sim_time());

	/* validating checksum of the received packet */
	int is_crpt = FALSE;
	if (!validate_checksum(packet)){
		// printf("corrupted packet\n");
		is_crpt = TRUE;
	}

	/* sending ACK to host A */
	struct pkt ack;
	ack.seqnum = 1;
	if (is_crpt || (packet.seqnum != B_expseqnum)){
		ack.acknum = B_expseqnum - 1;
	}
	else {
		ack.acknum = packet.seqnum;
	}
	strcpy(ack.payload, packet.payload);
	ack.checksum = compute_checksum(ack.seqnum, ack.acknum, ack.payload);
	// printf("B - SENT seq:%d ack:%d cs:%d payload:%s at time:%f\n", ack.seqnum, ack.acknum, ack.checksum, ack.payload, get_sim_time());
	tolayer3(B, ack);

	/* delivering data to layer 5 of host B if packet is neither out-of-order nor corrupt */
	if (B_expseqnum == packet.seqnum && !is_crpt){
		char payload[MSG_LEN];
		strncpy(payload, packet.payload, sizeof(payload));
		// printf("B - Delivered to layer 5 payload:%s\n", payload);
		tolayer5(B, payload);
		B_expseqnum++;
	}
}

/* the following routine will be called once (only) before any other */
/* entity B routines are called. You can use it to do any initialization */
void B_init()
{
	B_expseqnum = 0;
}

/**
 * function for calculating checksum
 *
 * @param seqnum Sequence number
 * @param acknum Acknowledgement number
 * @param payload Payload
 * @return checksum Checksum
 */
int compute_checksum(int seqnum, int acknum, char *payload){
	int i, checksum = 0;
	for (i = 0; i < MSG_LEN; i++){
		checksum += payload[i];
	}
	checksum += seqnum + acknum;
	return checksum;
}

/**
 * function for validating checksum
 *
 * @param packet Received packet
 * @return 0 if corrupted, otherwise 1
 */
int validate_checksum(struct pkt packet){
	char payload[MSG_LEN];
	strncpy(payload, packet.payload, sizeof(payload));
	int i, checksum = 0;
	for (i = 0; i < MSG_LEN; i++){
		checksum += payload[i];
	}
	checksum += packet.seqnum + packet.acknum;
	if (checksum == packet.checksum){
		return TRUE;
	}
	return FALSE;
}
