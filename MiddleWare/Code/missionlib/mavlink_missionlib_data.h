/*******************************************************************************
 
 Copyright (C) 2011 Lorenz Meier lm ( a t ) inf.ethz.ch
 
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 
 ****************************************************************************/

/* This assumes you have the mavlink headers on your include path
 or in the same folder as this source file */

// Disable auto-data structures
#ifndef MAVLINK_NO_DATA
#define MAVLINK_NO_DATA
#endif

#include "mavlink.h"
#include <stdbool.h>

/* MISSION LIB DATA STORAGE */

enum MAVLINK_PM_PARAMETERS
{
	MAVLINK_PM_PARAM_SYSTEM_ID,
	MAVLINK_PM_PARAM_ATT_K_D,
	MAVLINK_PM_MAX_PARAM_COUNT // VERY IMPORTANT! KEEP THIS AS LAST ENTRY
};


/* DO NOT EDIT THIS FILE BELOW THIS POINT! */

#ifndef MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN
#define MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN 16
#endif


//extern void mavlink_pm_queued_send();
struct mavlink_pm_storage {
	char param_names[MAVLINK_PM_MAX_PARAM_COUNT][MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN];      ///< Parameter names
	float param_values[MAVLINK_PM_MAX_PARAM_COUNT];    ///< Parameter values
	uint16_t next_param;
	uint16_t size;
};

typedef struct mavlink_pm_storage mavlink_pm_storage;

void mavlink_pm_reset_params(mavlink_pm_storage* pm);
