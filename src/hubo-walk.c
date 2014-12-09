/* Standard Stuff */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* Required Hubo Headers */
#include <hubo.h>

/* For Ach IPC */
#include <errno.h>
#include <fcntl.h>
#include <assert.h>
#include <unistd.h>
#include <pthread.h>
#include <ctype.h>
#include <stdbool.h>
#include <math.h>
#include <inttypes.h>
#include "ach.h"
//server libraries

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>


/* For system time */
#include <time.h>

#include "hubo-defines.h"

#include "hubo-controlled-move.h"
#define BUF_SIZE 1024

void step_left(struct hubo_ref *H_ref, struct hubo_state *H_state, size_t fs);
void step_right(struct hubo_ref *H_ref, struct hubo_state *H_state, size_t fs);
void reverse_left(struct hubo_ref *H_ref, struct hubo_state *H_state, size_t fs);
void reverse_right(struct hubo_ref *H_ref, struct hubo_state *H_state, size_t fs);
void turn_left(struct hubo_ref *H_ref, struct hubo_state *H_state, size_t fs);
void turn_right(struct hubo_ref *H_ref, struct hubo_state *H_state, size_t fs);

#define TIME_BASE SIM_TIME //use SIM_TIME for simulation, and REAL_TIME for a physical robot. 
#if TIME_BASE == SIM_TIME
/* For sleep */
#include "hubo-sleep-sim.h"
#elif TIME_BASE == REAL_TIME
//include a real-time sleep function here in the future
#endif

/* Ach Channel IDs */
ach_channel_t chan_hubo_ref;      // Feed-Forward (Reference)
ach_channel_t chan_hubo_state;    // Feed-Back (State)

int main(int argc, char **argv) {

    /* Open Ach Channel */
    int r = ach_open(&chan_hubo_ref, HUBO_CHAN_REF_NAME , NULL);
    assert( ACH_OK == r );

    r = ach_open(&chan_hubo_state, HUBO_CHAN_STATE_NAME , NULL);
    assert( ACH_OK == r );



    /* Create initial structures to read and write from */
    struct hubo_ref H_ref;
    struct hubo_state H_state;
    memset( &H_ref,   0, sizeof(H_ref));
    memset( &H_state, 0, sizeof(H_state));

    /* for size check */
    size_t fs;

    ach_put( &chan_hubo_ref, &H_ref, sizeof(H_ref));

    /* Get the current feed-forward (state) */
    r = ach_get( &chan_hubo_state, &H_state, sizeof(H_state), &fs, NULL, ACH_O_LAST );
    if(ACH_OK != r) {
        assert( sizeof(H_state) == fs );
    }
        char buf[BUF_SIZE];
    struct sockaddr_in self, other;
    int len = sizeof(struct sockaddr_in);
    int n, s, port;

    if (argc < 2) {
	fprintf(stderr, "Usage: %s <port>\n", argv[0]);
	return 1;
    }

    /* initialize socket */
    if ((s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) {
	perror("socket");
	return 1;
    }

    /* bind to server port */
    port = atoi(argv[1]);
    memset((char *) &self, 0, sizeof(struct sockaddr_in));
    self.sin_family = AF_INET;
    self.sin_port = htons(port);
    self.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(s, (struct sockaddr *) &self, sizeof(self)) == -1) {
	perror("bind");
	return 1;
    }
    while ((n = recvfrom(s, buf, BUF_SIZE, 0, (struct sockaddr *) &other, &len)) != -1) {
    	printf("Received from %s:%d: ", 
	inet_ntoa(other.sin_addr), 
	ntohs(other.sin_port)); 
	fflush(stdout);
	write(1, buf, n);
	write(1, "\n", 1);
	
	/* echo back to client */
	sendto(s, buf, n, 0, (struct sockaddr *) &other, len);
	if ( strcmp(buf,"forward")==0 ){
    		step_left(&H_ref, &H_state, fs);
    		
        }
        
        else if ( strcmp(buf,"left")==0 ){
    		turn_left(&H_ref, &H_state, fs);
        }
        
        else if ( strcmp(buf,"right")==0 ){
        	turn_right(&H_ref, &H_state, fs);     
        }
        
        else if ( strcmp(buf,"back")==0 ){
    		reverse_right(&H_ref, &H_state, fs);
        }
        else if ( strcmp(buf,"close")==0 ){
    		break;
        }

        
/*
	step_left(&H_ref, &H_state, fs);
	turn_left(&H_ref, &H_state, fs);
	reverse_left(&H_ref, &H_state, fs);	
	turn_right(&H_ref, &H_state, fs);
	step_right(&H_ref, &H_state, fs);
	reverse_right(&H_ref, &H_state, fs);
*/
	
    }
    close(s);
    return 0;
}

void step_left(struct hubo_ref *H_ref, struct hubo_state *H_state, size_t fs){
	printf("Step Left:\n");
	joint_pos *jp;
	double hip_angle = -0.14;
	double leg_bend_left = -0.4;
	double leg_bend_right = leg_bend_left; //-0.65
	double lean = -0.8; //-1.0
	jp = (joint_pos *) malloc(sizeof(joint_pos)*8);

	printf("Moving hips into position over right foot\n");
	jp[0].j = LHR;
	jp[0].p = -1.0 * hip_angle;
	jp[1].j = RHR;
	jp[1].p = -1.0 * hip_angle;
	jp[2].j = LAR;
	jp[2].p = hip_angle;
	jp[3].j = RAR;
	jp[3].p = hip_angle;
	controlled_move(jp, 4, 24, H_state, H_ref, fs);
	
	printf("balance on right leg\n");
	jp[0].j = LHP;
	jp[0].p = leg_bend_left;
	jp[1].j = LKN;
	jp[1].p = -2.0 * leg_bend_left;
	jp[2].j = LAP;
	jp[2].p = leg_bend_left;
	controlled_move(jp, 3, 8, H_state, H_ref, fs);
	
	printf("squat down on right leg\n");
	jp[0].j = RHP;
	jp[0].p = leg_bend_right;
	jp[1].j = RKN;
	jp[1].p = -2.0 * leg_bend_right;
	jp[2].j = RAP;
	jp[2].p = leg_bend_right;
	jp[3].j = LKN;
	jp[3].p = 0.0;
	jp[4].j = LAP;
	jp[4].p = -leg_bend_left;
	controlled_move(jp, 5, 10, H_state, H_ref, fs);

	printf("lean forward\n");
	jp[0].j = RHP;	
	jp[0].p = lean;
	jp[1].j = LHP;
	jp[1].p = lean;
	jp[2].j = LAP;
	jp[2].p = -leg_bend_right;
	controlled_move(jp, 3, 16, H_state, H_ref, fs);

	printf("Center hips\n");
	jp[0].j = RHR;
	jp[0].p = 0.0;
	jp[1].j = LHR;
	jp[1].p = 0.0;
	jp[2].j = RAR;
	jp[2].p = 0.0;
	jp[3].j = LAR;
	jp[3].p = 0.0;
	controlled_move(jp, 4, 24, H_state, H_ref, fs);

	printf("Straighten right knee\n");
	jp[0].j = RHP;	
	jp[0].p = -leg_bend_left;
	jp[1].j = LKN;
	jp[1].p = -2.0*leg_bend_left;
	jp[2].j = RKN;
	jp[2].p = 0.0;
	jp[3].j = LAP;
	jp[3].p = leg_bend_left;
	jp[4].j = RAP;
	jp[4].p = leg_bend_right;
	jp[5].j = LHP;
	jp[5].p = leg_bend_left;
	controlled_move(jp, 6, 45, H_state, H_ref, fs);

	printf("Moving hips into position over left foot\n");
	jp[0].j = RHR;
	jp[0].p = hip_angle;
	jp[1].j = LHR;
	jp[1].p = hip_angle;
	jp[2].j = RAR;
	jp[2].p = -1.0 * hip_angle;
	jp[3].j = LAR;
	jp[3].p = -1.0 * hip_angle;
	controlled_move(jp, 4, 30, H_state, H_ref, fs);

	printf("Straighten left leg\n");
	jp[0].j = LHP;
	jp[0].p = 0.0;
	jp[1].j = LKN;
	jp[1].p = 0.0;
	jp[2].j = LAP;
	jp[2].p = 0.0;
	controlled_move(jp, 3, 10, H_state, H_ref, fs);

	printf("Balancing on left leg\n");
	jp[0].j = RHP;
	jp[0].p = leg_bend_right;
	jp[1].j = RKN;
	jp[1].p = -2.0 * leg_bend_right;
	jp[2].j = RAP;
	jp[2].p = leg_bend_right;
	controlled_move(jp, 3, 8, H_state, H_ref, fs);

	printf("Put right foot down on ground\n");
	jp[0].j = RHP;
	jp[0].p = 0.0;
	jp[1].j = RKN;
	jp[1].p = 0.0;
	jp[2].j = RAP;
	jp[2].p = 0.0;
	controlled_move(jp, 3, 20, H_state, H_ref, fs);

	printf("Recenter hips.  Walking complete.\n");
	jp[0].j = RHR;
	jp[0].p = 0.0;
	jp[1].j = LHR;
	jp[1].p = 0.0;
	jp[2].j = RAR;
	jp[2].p = 0.0;
	jp[3].j = LAR;
	jp[3].p = 0.0;
	controlled_move(jp, 4, 24, H_state, H_ref, fs);

	free(jp);
}

void step_right(struct hubo_ref *H_ref, struct hubo_state *H_state, size_t fs){
	printf("Step Right:\n");
	joint_pos *jp;
	double hip_angle = -0.14;
	double leg_bend_left = -0.4;
	double leg_bend_right = leg_bend_left;
	double lean = -0.8;
	jp = (joint_pos *) malloc(sizeof(joint_pos)*8);


	printf("Moving hips into position over left foot\n");
	jp[0].j = RHR;
	jp[0].p = hip_angle;
	jp[1].j = LHR;
	jp[1].p = hip_angle;
	jp[2].j = RAR;
	jp[2].p = -1.0 * hip_angle;
	jp[3].j = LAR;
	jp[3].p = -1.0 * hip_angle;
	controlled_move(jp, 4, 30, H_state, H_ref, fs);

	printf("Straighten left leg\n");
	jp[0].j = LHP;
	jp[0].p = 0.0;
	jp[1].j = LKN;
	jp[1].p = 0.0;
	jp[2].j = LAP;
	jp[2].p = 0.0;
	controlled_move(jp, 3, 10, H_state, H_ref, fs);

	printf("Balancing on left leg\n");
	jp[0].j = RHP;
	jp[0].p = leg_bend_right;
	jp[1].j = RKN;
	jp[1].p = -2.0 * leg_bend_right;
	jp[2].j = RAP;
	jp[2].p = leg_bend_right;
	controlled_move(jp, 3, 8, H_state, H_ref, fs);

	printf("step forward with right leg\n");
	jp[0].j = LHP;
	jp[0].p = leg_bend_left;
	jp[1].j = LKN;
	jp[1].p = -2.0 * leg_bend_left;
	jp[2].j = LAP;
	jp[2].p = leg_bend_left;
	jp[3].j = RKN;
	jp[3].p = 0.0;
	jp[4].j = RAP;
	jp[4].p = -1.0 * leg_bend_right;
	controlled_move(jp, 5, 20, H_state, H_ref, fs);

	printf("lean forward\n");
	jp[0].j = RHP;	
	jp[0].p = lean;
	jp[1].j = LHP;
	jp[1].p = lean;
	jp[2].j = RAP;
	jp[2].p = -leg_bend_left;
	controlled_move(jp, 3, 16, H_state, H_ref, fs);

	printf("Center hips\n");
	jp[0].j = RHR;
	jp[0].p = 0.0;
	jp[1].j = LHR;
	jp[1].p = 0.0;
	jp[2].j = RAR;
	jp[2].p = 0.0;
	jp[3].j = LAR;
	jp[3].p = 0.0;
	controlled_move(jp, 4, 24, H_state, H_ref, fs);

	printf("Straighten left knee\n");
	jp[0].j = RHP;	
	jp[0].p = leg_bend_right;
	jp[1].j = LHP;
	jp[1].p = -leg_bend_left;
	jp[2].j = RKN;
	jp[2].p = -2.0*leg_bend_right;
	jp[3].j = LKN;
	jp[3].p = 0.0;
	jp[4].j = LAP;
	jp[4].p = leg_bend_left;
	jp[5].j = RAP;
	jp[5].p = leg_bend_right;
	controlled_move(jp, 6, 45, H_state, H_ref, fs);

	printf("Moving hips into position over right foot\n");
	jp[0].j = RHR;
	jp[0].p = -hip_angle;
	jp[1].j = LHR;
	jp[1].p = -hip_angle;
	jp[2].j = RAR;
	jp[2].p = hip_angle;
	jp[3].j = LAR;
	jp[3].p = hip_angle;
	controlled_move(jp, 4, 30, H_state, H_ref, fs);

	printf("Straighten right leg\n");
	jp[0].j = RHP;
	jp[0].p = 0.0;
	jp[1].j = RKN;
	jp[1].p = 0.0;
	jp[2].j = RAP;
	jp[2].p = 0.0;
	controlled_move(jp, 3, 20, H_state, H_ref, fs);

	printf("Balancing on right leg\n");
	jp[0].j = LHP;
	jp[0].p = leg_bend_left;
	jp[1].j = LKN;
	jp[1].p = -2.0 * leg_bend_left;
	jp[2].j = LAP;
	jp[2].p = leg_bend_left;
	controlled_move(jp, 3, 8, H_state, H_ref, fs);

	printf("Put left foot down on ground\n");
	jp[0].j = LHP;
	jp[0].p = 0.0;
	jp[1].j = LKN;
	jp[1].p = 0.0;
	jp[2].j = LAP;
	jp[2].p = 0.0;
	controlled_move(jp, 3, 20, H_state, H_ref, fs);

	printf("Recenter hips.  Walking complete.\n");
	jp[0].j = RHR;
	jp[0].p = 0.0;
	jp[1].j = LHR;
	jp[1].p = 0.0;
	jp[2].j = RAR;
	jp[2].p = 0.0;
	jp[3].j = LAR;
	jp[3].p = 0.0;
	controlled_move(jp, 4, 24, H_state, H_ref, fs);

	free(jp);
}

void reverse_left(struct hubo_ref *H_ref, struct hubo_state *H_state, size_t fs){
	printf("Reverse Left:\n");
	joint_pos *jp;
	double hip_angle = -0.14;
	double leg_bend_left = -0.4;
	double leg_bend_right = leg_bend_left;
	double lean = -0.8;
	jp = (joint_pos *) malloc(sizeof(joint_pos)*8);

	printf("Moving hips into position over right foot\n");
	jp[0].j = LHR;
	jp[0].p = -1.0 * hip_angle;
	jp[1].j = RHR;
	jp[1].p = -1.0 * hip_angle;
	jp[2].j = LAR;
	jp[2].p = hip_angle;
	jp[3].j = RAR;
	jp[3].p = hip_angle;
	controlled_move(jp, 4, 24, H_state, H_ref, fs);

	printf("Balancing on right leg\n");
	jp[0].j = LHP;
	jp[0].p = leg_bend_left;
	jp[1].j = LKN;
	jp[1].p = -2.0 * leg_bend_left;
	jp[2].j = LAP;
	jp[2].p = leg_bend_left;
	controlled_move(jp, 3, 8, H_state, H_ref, fs);

	printf("Straighten left knee\n");
	jp[0].j = RHP;	
	jp[0].p = leg_bend_right;
	jp[1].j = LHP;
	jp[1].p = -leg_bend_left;
	jp[2].j = RKN;
	jp[2].p = -2.0*leg_bend_right;
	jp[3].j = LKN;
	jp[3].p = 0.0;
	jp[4].j = LAP;
	jp[4].p = leg_bend_left;
	jp[5].j = RAP;
	jp[5].p = leg_bend_right;
	controlled_move(jp, 6, 45, H_state, H_ref, fs);

	printf("Center hips\n");
	jp[0].j = RHR;
	jp[0].p = 0.0;
	jp[1].j = LHR;
	jp[1].p = 0.0;
	jp[2].j = RAR;
	jp[2].p = 0.0;
	jp[3].j = LAR;
	jp[3].p = 0.0;
	controlled_move(jp, 4, 24, H_state, H_ref, fs);

	printf("step forward with right leg\n");
	jp[0].j = LHP;
	jp[0].p = leg_bend_left;
	jp[1].j = LKN;
	jp[1].p = -2.0 * leg_bend_left;
	jp[2].j = LAP;
	jp[2].p = leg_bend_left;
	jp[3].j = RKN;
	jp[3].p = 0.0;
	jp[4].j = RAP;
	jp[4].p = -1.0 * leg_bend_right;
	controlled_move(jp, 5, 20, H_state, H_ref, fs);
	hubo_sleep(1.0, H_state, fs);

	printf("Moving hips into position over left foot\n");
	jp[0].j = RHR;
	jp[0].p = hip_angle;
	jp[1].j = LHR;
	jp[1].p = hip_angle;
	jp[2].j = RAR;
	jp[2].p = -1.0 * hip_angle;
	jp[3].j = LAR;
	jp[3].p = -1.0 * hip_angle;
	controlled_move(jp, 4, 30, H_state, H_ref, fs);

	printf("Straighten left leg\n");
	jp[0].j = LHP;
	jp[0].p = 0.0;
	jp[1].j = LKN;
	jp[1].p = 0.0;
	jp[2].j = LAP;
	jp[2].p = 0.0;
	controlled_move(jp, 3, 10, H_state, H_ref, fs);

	printf("Balancing on left leg\n");
	jp[0].j = RHP;
	jp[0].p = leg_bend_right;
	jp[1].j = RKN;
	jp[1].p = -2.0 * leg_bend_right;
	jp[2].j = RAP;
	jp[2].p = leg_bend_right;
	controlled_move(jp, 3, 8, H_state, H_ref, fs);
	hubo_sleep(1.0, H_state, fs);	

	printf("Put right foot down on ground\n");
	jp[0].j = RHP;
	jp[0].p = 0.0;
	jp[1].j = RKN;
	jp[1].p = 0.0;
	jp[2].j = RAP;
	jp[2].p = 0.0;
	controlled_move(jp, 3, 20, H_state, H_ref, fs);

	printf("Recenter hips.  Walking complete.\n");
	jp[0].j = RHR;
	jp[0].p = 0.0;
	jp[1].j = LHR;
	jp[1].p = 0.0;
	jp[2].j = RAR;
	jp[2].p = 0.0;
	jp[3].j = LAR;
	jp[3].p = 0.0;
	controlled_move(jp, 4, 24, H_state, H_ref, fs);

	free(jp);
}

void reverse_right(struct hubo_ref *H_ref, struct hubo_state *H_state, size_t fs){
	printf("Reverse Right:\n");
	joint_pos *jp;
	double hip_angle = -0.14;
	double leg_bend_left = -0.4;
	double leg_bend_right = leg_bend_left;
	double lean = -0.8;
	jp = (joint_pos *) malloc(sizeof(joint_pos)*8);

	printf("Moving hips into position over left foot\n");
	jp[0].j = RHR;
	jp[0].p = hip_angle;
	jp[1].j = LHR;
	jp[1].p = hip_angle;
	jp[2].j = RAR;
	jp[2].p = -1.0 * hip_angle;
	jp[3].j = LAR;
	jp[3].p = -1.0 * hip_angle;
	controlled_move(jp, 4, 30, H_state, H_ref, fs);

	printf("Balancing on left leg\n");
	jp[0].j = RHP;
	jp[0].p = leg_bend_right;
	jp[1].j = RKN;
	jp[1].p = -2.0 * leg_bend_right;
	jp[2].j = RAP;
	jp[2].p = leg_bend_right;
	controlled_move(jp, 3, 8, H_state, H_ref, fs);

	printf("Straighten right knee\n");
	jp[0].j = RHP;	
	jp[0].p = -leg_bend_left;
	jp[1].j = LKN;
	jp[1].p = -2.0*leg_bend_left;
	jp[2].j = RKN;
	jp[2].p = 0.0;
	jp[3].j = LAP;
	jp[3].p = leg_bend_left;
	jp[4].j = RAP;
	jp[4].p = leg_bend_right;
	jp[5].j = LHP;
	jp[5].p = leg_bend_left;
	controlled_move(jp, 6, 45, H_state, H_ref, fs);

	printf("Center hips\n");
	jp[0].j = RHR;
	jp[0].p = 0.0;
	jp[1].j = LHR;
	jp[1].p = 0.0;
	jp[2].j = RAR;
	jp[2].p = 0.0;
	jp[3].j = LAR;
	jp[3].p = 0.0;
	controlled_move(jp, 4, 24, H_state, H_ref, fs);

	printf("squat down on right leg\n");
	jp[0].j = RHP;
	jp[0].p = leg_bend_right;
	jp[1].j = RKN;
	jp[1].p = -2.0 * leg_bend_right;
	jp[2].j = RAP;
	jp[2].p = leg_bend_right;
	jp[3].j = LKN;
	jp[3].p = 0.0;
	jp[4].j = LAP;
	jp[4].p = -leg_bend_left;
	controlled_move(jp, 5, 20, H_state, H_ref, fs);
	hubo_sleep(1.0, H_state, fs);

	printf("Moving hips into position over right foot\n");
	jp[0].j = LHR;
	jp[0].p = -1.0 * hip_angle;
	jp[1].j = RHR;
	jp[1].p = -1.0 * hip_angle;
	jp[2].j = LAR;
	jp[2].p = hip_angle;
	jp[3].j = RAR;
	jp[3].p = hip_angle;
	controlled_move(jp, 4, 24, H_state, H_ref, fs);

	printf("Straighten right leg\n");
	jp[0].j = RHP;
	jp[0].p = 0.0;
	jp[1].j = RKN;
	jp[1].p = 0.0;
	jp[2].j = RAP;
	jp[2].p = 0.0;
	controlled_move(jp, 3, 20, H_state, H_ref, fs);

	printf("Balancing on right leg\n");
	jp[0].j = LHP;
	jp[0].p = leg_bend_left;
	jp[1].j = LKN;
	jp[1].p = -2.0 * leg_bend_left;
	jp[2].j = LAP;
	jp[2].p = leg_bend_left;
	controlled_move(jp, 3, 8, H_state, H_ref, fs);
	hubo_sleep(1.0, H_state, fs);	

	printf("Put left foot down on ground\n");
	jp[0].j = LHP;
	jp[0].p = 0.0;
	jp[1].j = LKN;
	jp[1].p = 0.0;
	jp[2].j = LAP;
	jp[2].p = 0.0;
	controlled_move(jp, 3, 20, H_state, H_ref, fs);

	printf("Recenter hips.  Walking complete.\n");
	jp[0].j = RHR;
	jp[0].p = 0.0;
	jp[1].j = LHR;
	jp[1].p = 0.0;
	jp[2].j = RAR;
	jp[2].p = 0.0;
	jp[3].j = LAR;
	jp[3].p = 0.0;
	controlled_move(jp, 4, 24, H_state, H_ref, fs);

	free(jp);
}

void turn_left(struct hubo_ref *H_ref, struct hubo_state *H_state, size_t fs){
	printf("Turn Left:\n");
	joint_pos *jp;
	double hip_angle = -0.14;
	double leg_bend_left = -0.3;
	double leg_bend_right = leg_bend_left;
	double turn_angle = 0.20;
	jp = (joint_pos *) malloc(sizeof(joint_pos)*8);

	printf("Moving hips into position over left foot\n");
	jp[0].j = RHR;
	jp[0].p = hip_angle;
	jp[1].j = LHR;
	jp[1].p = hip_angle;
	jp[2].j = RAR;
	jp[2].p = -1.0 * hip_angle;
	jp[3].j = LAR;
	jp[3].p = -1.0 * hip_angle;
	controlled_move(jp, 4, 30, H_state, H_ref, fs);

	printf("Balancing on left leg\n");
	jp[0].j = RHP;
	jp[0].p = leg_bend_right;
	jp[1].j = RKN;
	jp[1].p = -2.0 * leg_bend_right;
	jp[2].j = RAP;
	jp[2].p = leg_bend_right;
	controlled_move(jp, 3, 4, H_state, H_ref, fs);

	printf("Turning right leg\n");
	jp[0].j = RHY;
	jp[0].p = turn_angle;
	controlled_move(jp, 1, 8, H_state, H_ref, fs);

	printf("Put right foot down on ground\n");
	jp[0].j = RHP;
	jp[0].p = 0.0;
	jp[1].j = RKN;
	jp[1].p = 0.0;
	jp[2].j = RAP;
	jp[2].p = 0.0;
	controlled_move(jp, 3, 20, H_state, H_ref, fs);

	printf("Moving hips into position over right foot\n");
	jp[0].j = LHR;
	jp[0].p = -1.0 * hip_angle;
	jp[1].j = RHR;
	jp[1].p = -1.0 * hip_angle;
	jp[2].j = LAR;
	jp[2].p = hip_angle;
	jp[3].j = RAR;
	jp[3].p = hip_angle;
	controlled_move(jp, 4, 24, H_state, H_ref, fs);

	printf("Balancing on right leg\n");
	jp[0].j = LHP;
	jp[0].p = leg_bend_left;
	jp[1].j = LKN;
	jp[1].p = -2.0 * leg_bend_left;
	jp[2].j = LAP;
	jp[2].p = leg_bend_left;
	controlled_move(jp, 3, 4, H_state, H_ref, fs);

	printf("Turning body\n");
	jp[0].j = RHY;
	jp[0].p = 0.0;
	controlled_move(jp, 1, 8, H_state, H_ref, fs);

	printf("Put left foot down on ground\n");
	jp[0].j = LHP;
	jp[0].p = 0.0;
	jp[1].j = LKN;
	jp[1].p = 0.0;
	jp[2].j = LAP;
	jp[2].p = 0.0;
	controlled_move(jp, 3, 20, H_state, H_ref, fs);

	printf("Recenter hips.  Walking complete.\n");
	jp[0].j = RHR;
	jp[0].p = 0.0;
	jp[1].j = LHR;
	jp[1].p = 0.0;
	jp[2].j = RAR;
	jp[2].p = 0.0;
	jp[3].j = LAR;
	jp[3].p = 0.0;
	controlled_move(jp, 4, 24, H_state, H_ref, fs);

	free(jp);
}

void turn_right(struct hubo_ref *H_ref, struct hubo_state *H_state, size_t fs){
	printf("Turn Right:\n");
	joint_pos *jp;
	double hip_angle = -0.14;
	double leg_bend_left = -0.3;
	double leg_bend_right = leg_bend_left;
	double turn_angle = 0.20;
	jp = (joint_pos *) malloc(sizeof(joint_pos)*8);

	printf("Moving hips into position over right foot\n");
	jp[0].j = LHR;
	jp[0].p = -1.0 * hip_angle;
	jp[1].j = RHR;
	jp[1].p = -1.0 * hip_angle;
	jp[2].j = LAR;
	jp[2].p = hip_angle;
	jp[3].j = RAR;
	jp[3].p = hip_angle;
	controlled_move(jp, 4, 24, H_state, H_ref, fs);

	printf("Balancing on right leg\n");
	jp[0].j = LHP;
	jp[0].p = leg_bend_left;
	jp[1].j = LKN;
	jp[1].p = -2.0 * leg_bend_left;
	jp[2].j = LAP;
	jp[2].p = leg_bend_left;
	controlled_move(jp, 3, 4, H_state, H_ref, fs);

	printf("Turning left leg\n");
	jp[0].j = LHY;
	jp[0].p = -turn_angle;
	controlled_move(jp, 1, 8, H_state, H_ref, fs);

	printf("Put left foot down on ground\n");
	jp[0].j = LHP;
	jp[0].p = 0.0;
	jp[1].j = LKN;
	jp[1].p = 0.0;
	jp[2].j = LAP;
	jp[2].p = 0.0;
	controlled_move(jp, 3, 20, H_state, H_ref, fs);

	printf("Moving hips into position over left foot\n");
	jp[0].j = RHR;
	jp[0].p = hip_angle;
	jp[1].j = LHR;
	jp[1].p = hip_angle;
	jp[2].j = RAR;
	jp[2].p = -1.0 * hip_angle;
	jp[3].j = LAR;
	jp[3].p = -1.0 * hip_angle;
	controlled_move(jp, 4, 30, H_state, H_ref, fs);

	printf("Balancing on left leg\n");
	jp[0].j = RHP;
	jp[0].p = leg_bend_right;
	jp[1].j = RKN;
	jp[1].p = -2.0 * leg_bend_right;
	jp[2].j = RAP;
	jp[2].p = leg_bend_right;
	controlled_move(jp, 3, 4, H_state, H_ref, fs);

	printf("Turning body\n");
	jp[0].j = LHY;
	jp[0].p = 0.0;
	controlled_move(jp, 1, 8, H_state, H_ref, fs);

	printf("Put right foot down on ground\n");
	jp[0].j = RHP;
	jp[0].p = 0.0;
	jp[1].j = RKN;
	jp[1].p = 0.0;
	jp[2].j = RAP;
	jp[2].p = 0.0;
	controlled_move(jp, 3, 20, H_state, H_ref, fs);

	printf("Recenter hips.  Walking complete.\n");
	jp[0].j = RHR;
	jp[0].p = 0.0;
	jp[1].j = LHR;
	jp[1].p = 0.0;
	jp[2].j = RAR;
	jp[2].p = 0.0;
	jp[3].j = LAR;
	jp[3].p = 0.0;
	controlled_move(jp, 4, 24, H_state, H_ref, fs);

	free(jp);
}
