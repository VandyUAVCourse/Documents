#ifndef _LASER_UTILS_H_
#define _LASER_UTILS_H_

#include <ipcMessages/qc_laser_messages.h>

class LaserUtils{
	public:
	static void deepMessageCopy(const qc_laser_laser_message& src, qc_laser_laser_message& dest);
};

#endif // _LASER_UTILS_H_
