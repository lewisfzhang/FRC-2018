#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sstream> 
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 

#include "com_team254_lib_util_drivers_RPLidarJNI.h"
#include "com_team254_lib_util_drivers_RPLidarJNI_DataPoint.h"
#include "include/rplidar.h" //RPLIDAR standard sdk, all-in-one header

/* 
 * error - wrapper for perror
 */
void error(char *msg) {
    perror(msg);
    exit(0);
}

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#ifdef _WIN32
#include <Windows.h>
#define delay(x)   ::Sleep(x)
#else
#include <unistd.h>
static inline void delay(_word_size_t ms){
    while (ms>=1000){
        usleep(1000*1000);
        ms-=1000;
    };
    if (ms!=0)
        usleep(ms*1000);
}
#endif

using namespace rp::standalone::rplidar;

RPlidarDriver *drv = RPlidarDriver::CreateDriver(RPlidarDriver::DRIVER_TYPE_SERIALPORT);

JNIEXPORT void JNICALL Java_com_team254_lib_util_drivers_RPLidarJNI_init
  (JNIEnv *env, jobject obj) {
    const char *opt_com_path = "/dev/ttyUSB0";
    _u32 opt_com_baudrate = 115200;

    if (!drv) {
        fprintf(stderr, "insufficent memory, exit\n");
        exit(-2);
    }

    // make connection...
    if (IS_FAIL(drv->connect(opt_com_path, opt_com_baudrate))) {
        fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n", opt_com_path);
        RPlidarDriver::DisposeDriver(drv);
    }
}

JNIEXPORT jboolean JNICALL Java_com_team254_lib_util_drivers_RPLidarJNI_checkHealth
  (JNIEnv *env, jobject obj) {
    u_result op_result;
    rplidar_response_device_health_t healthinfo;

    op_result = drv->getHealth(healthinfo);
    if (IS_OK(op_result)) {
        printf("RPLidar health status : %d\n", healthinfo.status);
        if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
            fprintf(stderr, "Error, rplidar internal error detected. Please reboot the device to retry.\n");
            return JNI_FALSE;
        } else {
            return JNI_TRUE;
        }

    } else {
        fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
        return JNI_FALSE;
    }
}

JNIEXPORT void JNICALL Java_com_team254_lib_util_drivers_RPLidarJNI_startMotor
  (JNIEnv *env, jobject obj) {
    drv->startMotor();
}

JNIEXPORT void JNICALL Java_com_team254_lib_util_drivers_RPLidarJNI_startScan
  (JNIEnv *env, jobject obj) {
    drv->startScan();
}

JNIEXPORT jstring JNICALL Java_com_team254_lib_util_drivers_RPLidarJNI_grabScanData
  (JNIEnv *env, jobject obj) {
    u_result op_result;
    rplidar_response_measurement_node_t nodes[360*2];
    size_t count = _countof(nodes);
    std::stringstream packet;
    int nodes_buffered = 0;
    op_result = drv->grabScanData(nodes, count);
    
    drv->ascendScanData(nodes, count);
    for (int pos = 0; pos < (int) count; pos++) {         
        char buf[300];
        sprintf(buf, "%llu,%03.2f,%08.2f\n", 
            nodes[pos].timestamp, 
            (nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f,
            nodes[pos].distance_q2/4.0f);
        packet << buf;
        nodes_buffered++;

        if(nodes_buffered > 50) {
            const char* packet_str = packet.str().c_str();
            return env->NewStringUTF(packet_str);
        }
    }

    return NULL;
}

JNIEXPORT void JNICALL Java_com_team254_lib_util_drivers_RPLidarJNI_stop
  (JNIEnv *env, jobject obj) {
    drv->stop();
    drv->stopMotor();
}