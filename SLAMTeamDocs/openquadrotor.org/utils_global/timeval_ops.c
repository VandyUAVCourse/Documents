#include "timeval_ops.h"

void tvu_correct_timeval (struct timeval* t){
	if (t->tv_usec < 0){
		t->tv_usec += ONE_MILLION;
		t->tv_sec -= 1;	
	} else {
		if (t->tv_usec >= ONE_MILLION){
			t->tv_usec -= ONE_MILLION;
			t->tv_sec += 1;	
		}
	}
}
	
	
double tvu_timeval_to_double(const struct timeval* t){
	return t->tv_sec * 1.0 + t->tv_usec * 1.0e-6;
}

struct timeval tvu_double_to_timeval(const double* v){
	struct timeval t;
	t.tv_sec = (int)*v;
	t.tv_usec = (int)(*v - ((double)t.tv_sec)) * ONE_MILLION;
	return t;
}


///calc t2-t1
void tvu_subtract (struct timeval* result, struct timeval* t2, struct timeval* t1){
	/* Perform the carry for the later subtraction by updating y. */
	result->tv_sec = t2->tv_sec - t1->tv_sec;
	result->tv_usec = t2->tv_usec - t1->tv_usec;
	tvu_correct_timeval(result);
// 	if (result->tv_usec < 0){
// 		result->tv_usec += ONE_MILLION;
// 		result->tv_sec -= 1;
// 	}
}

void tvu_add (struct timeval* result, struct timeval* t1, struct timeval* t2){
	result->tv_sec = t2->tv_sec + t1->tv_sec;
	result->tv_usec = t2->tv_usec + t1->tv_usec;
	tvu_correct_timeval(result);
// 	if (result->tv_usec >= ONE_MILLION){
// 		result->tv_usec -= ONE_MILLION;
// 		result->tv_sec += 1;
// 	}
}

void tvu_add_ms(struct timeval* result, struct timeval* t1, int ms){
	struct timeval t2;
	t2.tv_sec = ms / 1000;
	t2.tv_usec = (ms % 1000)*1000;
	return tvu_add (result, t1, &t2);
}
