#ifndef _QC_IPC_INTERFACE_H
#define _QC_IPC_INTERFACE_H

#include <unistd.h>
#include <iostream>
#include <cstring>
#include <ipc/ipc.h>
#include <utils_ipc/colormakros.h>

int qc_ipc_connect(char* modulName);

#endif // _QC_IPC_INTERFACE_H
