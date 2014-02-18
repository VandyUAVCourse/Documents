#ifndef _TIMEVAL_OPS_H
#define _TIMEVAL_OPS_H

#include <sys/time.h>

#define ONE_MILLION 1000000

///see http://www.xorp.org/release-0.2/docs/kdoc/html/libxorp/timeval_hh.html
///for a c++ implementation

#ifdef __cplusplus
extern "C" {
#endif

///checks for illigal values (like usec < 0 or usec >= 1000000)
///corrects this if needed!
void tvu_correct_timeval (struct timeval* t);
	
double tvu_timeval_to_double(const struct timeval* t);

struct timeval tvu_double_to_timeval(const double* v);


///calculate result = t2-t1
void tvu_subtract (struct timeval* result, struct timeval* t2, struct timeval* t1);

///calculate result = t1+t2
void tvu_add (struct timeval* result, struct timeval* t1, struct timeval* t2);

void tvu_add_ms(struct timeval* result, struct timeval* t1, int ms);


#ifdef __cplusplus
}
#endif

#endif //_TIMEVAL_OPS_H
