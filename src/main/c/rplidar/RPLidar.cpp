#include "com_team254_lib_util_drivers_RPLidarJNI.h"
#include "com_team254_lib_util_drivers_RPLidarJNI_DataPoint.h"
#include "rplidar.h"

RPlidarDriver *drv;

JNIEXPORT void JNICALL Java_com_team254_lib_util_drivers_RPLidarJNI_init
  (JNIEnv *env, jobject obj) {
    const char *opt_com_path = "/dev/ttyUSB0";
    _u32 opt_com_baudrate = 115200;
    *drv = RPlidarDriver::CreateDriver(RPlidarDriver::DRIVER_TYPE_SERIALPORT);

    if (!drv) {
        fprintf(stderr, "insufficent memory, exit\n");
        exit(-2);
    }

    // make connection...
    if (IS_FAIL(drv->connect(opt_com_path, opt_com_baudrate))) {
        fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n", opt_com_path);
        goto on_finished;
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

JNIEXPORT void JNICALL Java_com_team254_lib_util_drivers_RPLidarJNI_grabScanData
  (JNIEnv *env, jobject obj, jdoubleArray arr) {}

JNIEXPORT void JNICALL Java_com_team254_lib_util_drivers_RPLidarJNI_stop
  (JNIEnv *env, jobject obj) {
    drv->stop();
    drv->stopMotor();
}