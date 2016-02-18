#include "hal.h"

void matrixDbgCheck(bool a){
  osalDbgCheck(a);
}

void matrixDbgPanic(const char *msg){
  osalSysHalt(msg);
}

void matrixDbgPrint(const char *msg){
  (void)msg;
}
