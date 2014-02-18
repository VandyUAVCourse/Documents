#include "urg_device_driver.h"

#define HK_RESET "\rRS;\n"
#define HK_SCIP2 "\rSCIP2.0\n"
#define HK_STOP  "\rQT;\n"


// double g_hokuyo_internal_clock_factor = 0.99718478;


inline char* hokuyo_skipLine(char* buf){
	char* tmp = buf;
	while (*tmp!=0 && *tmp!='\n' && *tmp!='\r')
		tmp++;
	return (*tmp=='\n')?tmp+1:0;
}


///read data until "\n\n"
int hokuyo_readPacket(HokuyoURG* urg, char* buffer, int buffer_size, int max_failures){
	int i = 0;
	int timeout = max_failures;
	
	if (urg->fd <= 0){
		if (DEBUG) fprintf(stderr,"Invalid urg->fd\n");
		return -1;
	}
	
	memset(buffer,'\0',buffer_size);
	int last_char_LF = 0;
	char* curr_buffer = buffer;
	int remaining_size = buffer_size;
	
	while (1){
		int c = read (urg->fd, curr_buffer, remaining_size);
		if (!c){ ///no char read - timeout 
			if (DEBUG) fprintf(stderr,"NULL - device not there?!\n");
			timeout--;
			usleep(25000);
		} else { ///we have read "c" num characters
			for (i=0; i<c; i++){
				if (last_char_LF && (curr_buffer[i]=='\n')){
					curr_buffer++;
					return curr_buffer - buffer;
				}
				last_char_LF = (curr_buffer[i]=='\n');
			}
			curr_buffer += c;
			remaining_size -= c;
			if (remaining_size <= 0){
				fprintf(stderr,"FATAL ERROR: insufficient buffer size!\n");
				return 0;
			}
		}
		if (timeout < 0){
			fprintf(stderr, "FATAL ERROR: Device not there anymore! aborting!\n");
			exit(-1);
			//return 0;
		}
	}

}


int hokuyo_readPacket_valid(HokuyoURG* urg, char* buffer, int buffer_size, int max_failures, int scip2_mode){
	int c = hokuyo_readPacket(urg, buffer, buffer_size, max_failures);
	int min_ack = (scip2_mode==1)?8:5;
	while (c <= min_ack){ ///in SCIP2.0 mode minimum length is 9
		//if (DEBUG) fprintf(stderr,"bullshit-packet received.. repeating\n");
		c = hokuyo_readPacket(urg, buffer, buffer_size, max_failures);
	}
	return c;
}

///repeadily send version-commands until version is received properly --> flushing buffers!
void hokuyo_flush(HokuyoURG* urg){
	char buffer[URG_BUFSIZE];
	write(urg->fd, "\rVV;\n",strlen("\rVV;\n"));
	int c = 1;
	while (c != 0){
		c = hokuyo_readPacket(urg, buffer, URG_BUFSIZE, 1);
		if ( (c > 20) && (buffer[0] == 'V') && (buffer[1] == 'V') && (buffer[2] == ';'))
			c = 0;
		else
			c = 1;
	}
}

///open device
int hokuyo_open(HokuyoURG* urg, const char* filename){
	urg->isProtocol2=0;
	urg->isInitialized=0;
	urg->isContinuous=0;
	urg->fd=open(filename, O_RDWR| O_NOCTTY | O_SYNC | O_RSYNC);
	if (urg->fd <= 0)
		fprintf(stderr, "Error: unable to open device file.\n     -----> Is the device connected?\n     -----> Did you enter the correct path to the device?\n     -----> Do I have the permisson to read + write the device?\n     -----> Is the led of the hokuyo flashing? (if yes->error, unplug it from power!)\n     -----> (Note: Smoke coming out of the Hokuyo _could_ be a sign of a malfunction!)\n");
	return urg->fd;
}


///init device
int hokuyo_init(HokuyoURG* urg){
	if (urg->fd <= 0){
		return 0;
	}
	int c = 0;
	char buffer[URG_BUFSIZE];
	///check if SCIP2.0 is already initialized
	///stop Hokuyo in SCIP2.0 mode. Will accept command if it was running before
	write(urg->fd, "\rTM2;\n", strlen("\rTM2;\n"));
	write(urg->fd, HK_STOP, strlen(HK_STOP));
	c = hokuyo_readPacket_valid(urg, buffer, URG_BUFSIZE, 10, 0);
	fprintf(stderr,"Checking for SCIP2.0...\n");
	write(urg->fd, HK_RESET, strlen(HK_RESET));
	c = hokuyo_readPacket_valid(urg, buffer, URG_BUFSIZE, 10, 0);
	if (c == 7){
		///SCIP2.0 not activated
		fprintf(stderr,"SCIP2.0 not initilized: initializing SCIP2.0 mode...\n");
		write(urg->fd, HK_SCIP2, strlen(HK_SCIP2));
		c = hokuyo_readPacket_valid(urg, buffer, URG_BUFSIZE, 10, 0);
		if (strncmp(buffer,"RS;\nE",5) == 0){
			c = hokuyo_readPacket_valid(urg, buffer, URG_BUFSIZE, 10, 0);
		}
		if (c == 11){
			if (!strcmp(buffer,"SCIP2.0\n0\n\n")){
				fprintf(stderr,"SCIP2.0 initialized successfully\n");
				urg->isInitialized = 1;
				urg->isProtocol2 = 1;
				hokuyo_flush(urg);
				return 1;
			} else {
				fprintf(stderr,"Could not change to SCIP2.0 mode - UPGRADE device!\n");
				urg->isProtocol2 = 0;
				urg->isInitialized = 0;
				hokuyo_flush(urg);
				return 0;
			}
		} else {
			fprintf(stderr,"Error: eventually SCIP2.0 not supported!\n");
			fprintf(stderr,"Answer was: \"%s\"",buffer);
			fprintf(stderr,"Eventually Upgrade to SCIP2.0 needed! Aborting!\n");
			urg->isProtocol2 = 0;
			urg->isInitialized = 0;
			hokuyo_flush(urg);
			return 0;
		}
	} else {
		fprintf(stderr,"SCIP2.0 already initialized\n");
		urg->isProtocol2 = 1;
		urg->isInitialized = 1;
		hokuyo_flush(urg);
		return 1;
	}
}


int hokuyo_reset(HokuyoURG* urg){
	if (urg->fd <= 0){
		return 0;
	}
	int c = 0;
	char buffer[URG_BUFSIZE];
	write(urg->fd, HK_RESET, strlen(HK_RESET));
	c = hokuyo_readPacket_valid(urg, buffer, URG_BUFSIZE, 10, 1);
	if (strcmp(buffer,"RS;\n00P\n\n") != 0){
		hokuyo_flush(urg);
		///error; RS was not accepted
		//fprintf(stderr,"Received: %s",buffer);
		return 0;
	}
	return 1;
}

///pareses starting from buffer 4 chars and interprets this as timestamp
int hokuyo_getTime(char* buffer){
	int i=0;
	int result_time = 0;
	for (i=0; i<4; i++){
		char t = (buffer[i] - (char)0x30)&0x3F;
		//fprintf(stderr,"befor= %x  after= %x\n", buffer[i], (buffer[i] - (char)0x30)&0x3F);
		result_time = result_time + (((unsigned int)t) << ((3-i)*6));
	}
	return result_time;
}



int hokuyo_estimateStartingTime(HokuyoURG* urg, struct timeval* startTime){
	if (urg->fd <= 0)
		return 0;
	if (startTime == NULL)
		return 0;
	gettimeofday(startTime,NULL);
	int c = 0;
	char buffer[URG_BUFSIZE];
	write(urg->fd, "\rTM0;\n", strlen("\rTM0;\n"));
	c = hokuyo_readPacket_valid(urg, buffer, URG_BUFSIZE, 10, 1);
	if (strcmp(buffer,"TM0;\n00P\n\n") != 0){
		fprintf(stderr, "Error: recieved: %s", buffer);
		return 0;
	}
	struct timeval t1;
	struct timeval t2;
	struct timeval t3;
	struct timeval delta;
	delta.tv_sec = 0;
	delta.tv_usec = 0;
	int i=0;
	int hokuyo_time = 0;
	//fprintf(stderr,"#please wait - calibrating (phase1)...");
	for (i=0; i<100; i++){
		gettimeofday(&t1,NULL);	
		write(urg->fd, "\rTM1;\n", strlen("\rTM1;\n"));
		c = hokuyo_readPacket_valid(urg, buffer, URG_BUFSIZE, 10, 1);
		gettimeofday(&t2,NULL);
		if (strncmp(buffer,"TM1;\n00P\n",9)!=0){
			//fprintf(stderr,"got wrong response\n");
			i--;
		} else {
			hokuyo_time = hokuyo_getTime(buffer+9);
			tvu_subtract(&t3,&t2,&t1);
			tvu_add(&delta,&delta,&t3);
		}
	}
	//fprintf(stderr,"done\n");
	///now s shoud be 0 and us about 200-600us
	double s = delta.tv_sec / 200.;
	double us = delta.tv_usec / 200.;
	if ( s > 0 ){
		fprintf(stderr,"sending a command (one way) takes longer than 1 sec (about %f), this is extremely strange!!\n",s);
	}
	t2.tv_sec = hokuyo_time / 1000;
	t2.tv_usec = (hokuyo_time % 1000) * 1000;
	
	t3.tv_sec = s;
	t3.tv_usec = us;
	
	tvu_subtract(startTime, &t1, &t2);
	tvu_add(startTime, startTime, &t3);
	
	write(urg->fd, "\rTM2;\n", strlen("\rTM2;\n"));
	c = hokuyo_readPacket_valid(urg, buffer, URG_BUFSIZE, 10, 1);

	///buggy stuff. sometimes TM2;T is received instead of TM2;
	if (strcmp(buffer,"TM2;\n00P\n\n") != 0 && strcmp(buffer,"TM2;T\n00P\n\n") != 0){
		fprintf(stderr, "Error: could not disable TM mode ??: restart hokuyo driver! received message was: %s", buffer);
		return 0;
	}
	hokuyo_flush(urg);
	return 1;
	
}


int hokuyo_startContinuous(HokuyoURG* urg, int startStep, int endStep, int clusterCount){
	if (!urg->isInitialized)
		return 0;
	if (urg->isContinuous)
		return 1;

	char buffer[URG_BUFSIZE];
			
	///turn on laser
	int c = 0;
	write(urg->fd, "\rBM;\n", strlen("\rBM;\n"));
	c = hokuyo_readPacket_valid(urg, buffer, URG_BUFSIZE, 10, 1);
	if ((strncmp(buffer,"BM;\n00",6) != 0) && (strncmp(buffer,"BM;\n02",6) != 0)){
		///error
		fprintf(stderr,"Error: could not switch on laser! Message was %s\n", buffer);
		write(urg->fd, "\rQT;\n", strlen("\rQT;\n"));
		c = hokuyo_readPacket_valid(urg, buffer, URG_BUFSIZE, 10, 1);
		return 0;
	}

	///laser is turned on or it was already on
	char command[32];
	sprintf (command, "\rMD%04d%04d%02d000;\n", startStep, endStep, clusterCount);
	write(urg->fd, command, strlen(command));	
	c = hokuyo_readPacket_valid(urg, buffer, URG_BUFSIZE, 10, 1);
	char *ack = hokuyo_skipLine(buffer);
	if ((ack[0] != '9' || ack[1] != '9') && (ack[0] != '0' || ack[1] != '0')){
		fprintf(stderr,"Error initializing laser: Message was %s\n",buffer);
		write(urg->fd, "\rQT;\n", strlen("\rQT;\n"));
		c = hokuyo_readPacket_valid(urg, buffer, URG_BUFSIZE, 10, 1);
		return 0;
	}
	urg->isContinuous = 1;
	return 1;
}

int hokuyo_stopContinuous(HokuyoURG* urg){
	if (urg->fd <= 0)
		return 0;
	char buffer[URG_BUFSIZE];
	int c = 0;
	int k = 0;
	for (k = 0; k<10; k++){
		write(urg->fd, "\rQT;\n", strlen("\rQT;\n"));
		c = hokuyo_readPacket_valid(urg, buffer, URG_BUFSIZE, 10, 1);	
		if (strcmp(buffer,"QT;\n00P\n\n") == 0)
			break;
	}
	if (k == 10){
		fprintf(stderr, "Error: Unable to disable Laser! Message was %s\n",buffer);
		return 0;
	}
	urg->isContinuous = 0;
	hokuyo_flush(urg);
	return 1;
}

int hokuyo_close(HokuyoURG* urg){
	if (urg->fd <= 0)
		return 0;
	write(urg->fd, "\rQT;\n", strlen("\rQT;\n"));
	write(urg->fd, "\rRS;\n", strlen("\rRS;\n"));
	hokuyo_flush(urg);
	return 1;
}


unsigned int parseInt(int bytes, char** s){
	int i;
	char* b=*s;
	unsigned int ret=0;
	int j=bytes-1;
	for (i=0; i<bytes;){
		if (*b==0||*b=='\n'){
			*s=0;
			return 0;
		}
		if (*(b+1)=='\n'){ //check for a wrapped line
			b++;
		} else {
			unsigned char c=*b-0x30;
			ret+=((unsigned int ) c)<<(6*j);
			i++;
			j--;
		}
		b++;
	}
	*s=b;
	return ret;
}


int hokuyo_getQCLaserMessage(HokuyoURG* urg, qc_laser_laser_message* msg, int* diffMS){
	if (urg->fd <= 0)
		return 0;
	if (urg->isContinuous == 0)
		return 0;
	
	char buffer[URG_BUFSIZE];
	int c = 0;
	struct timeval now;
	while (c < 33) ///minimum length of laser message
	 c = hokuyo_readPacket_valid(urg, buffer, URG_BUFSIZE, 10, 1);
	gettimeofday(&now,NULL);
	if (diffMS != NULL){
		struct timeval difftv;
		difftv.tv_sec = 0;
		difftv.tv_usec = *diffMS;
		tvu_subtract(&now,&now,&difftv);
	}
	msg->timestamp_sec = now.tv_sec;
	msg->timestamp_usec = now.tv_usec;
	msg->status = 1;
	char *curr = buffer;
	if (curr[0] != 'M' || curr[1] !='D'){
		msg->status = 0;
		fprintf (stderr,"F");
		return 0;
	}
	curr+=2;
	char v[5];
	v[4]=0;
	strncpy(v,curr,4); msg->startStep=atoi(v); curr+=4; //startStep
	strncpy(v,curr,4); msg->stopStep  =atoi(v); curr+=4; //stopStep
	v[2]=0; strncpy(v,curr,2); msg->clusterCount=atoi(v); //Clustercount
	curr = hokuyo_skipLine(buffer);
	msg->startAngle = (-135./180.)*M_PI + msg->startStep * (M_PI/512.);
	msg->incrementAngle = ((msg->clusterCount == 0) || (msg->clusterCount == 1))? (M_PI/512.) : (M_PI/512.) * msg->clusterCount;
	///check for status
	if (curr[0] != '9' || curr[1] != '9'){
		msg->status = 0;
		fprintf (stderr,"F");
		return 0;
	}
	curr+=4; ///time of the hokuyo
	msg->internal_timestamp_hokuyo = hokuyo_getTime(curr);
	///read the data
	curr = hokuyo_skipLine(curr);
	
	int i=0;
	while(curr!=0){
		msg->ranges[i++]=(int)parseInt(3,&curr);
	}
	i--;
	msg->num_ranges=i;
	///
	return 1;
}

//try to estiumate the difference between hokuyo timestamp and the pc timestamp just after getting the laser data
int hokuyo_estimateDeltaT(HokuyoURG* urg, int* deltaMS){
	///1) reset
	///2) estimate basis time
	///3) assume linear factor 0.99718
	///4) calculate time just afer receiving message
	///5) repreat 1-4 for 10 times
	///6) average
	int m = 0;
	///FIXME how much messages?
	int maxMessages = 10; ///1 sec
	int differences_us = 0;
	struct timeval diff;
	struct timeval pctime;
	qc_laser_laser_message tlmsg;
	tlmsg.ranges = calloc(683, sizeof(int));
	struct timeval startTime;
	int run = 0;
	int maxRuns = 10;
	
	fprintf(stderr,"Calibrating, please wait (~ 1min): ");
	for (run = 0; run < maxRuns; run++){
		if (!hokuyo_reset(urg))
			return 0;
		if (!hokuyo_estimateStartingTime(urg,&startTime))
			return 0;
		fprintf(stderr,"*");
		hokuyo_startContinuous(urg, 44, 725, 0);
		for (m = 0; m < maxMessages; m++){
			if (hokuyo_getQCLaserMessage(urg, &tlmsg, NULL)){
				tvu_add_ms(&diff,&startTime,tlmsg.internal_timestamp_hokuyo*0.99718);
				pctime.tv_sec = tlmsg.timestamp_sec;
				pctime.tv_usec = tlmsg.timestamp_usec;
				tvu_subtract(&diff, &pctime, &diff);
				if (diff.tv_sec > 0){
					m--;
				} else {
					differences_us += diff.tv_usec;
				}
			}else
				m--;
				usleep(10000);
		}
		hokuyo_stopContinuous(urg);;
		hokuyo_flush(urg);
		usleep(1000);
	}
	differences_us = differences_us / (maxRuns * maxMessages);
	fprintf(stderr," done (%d us)\n",differences_us);
	*deltaMS = differences_us;
	free(tlmsg.ranges);
	return 1;
}

