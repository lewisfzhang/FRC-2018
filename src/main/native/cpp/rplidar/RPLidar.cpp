#include <cstdio>
#include <cstdlib>

#include "com_team254_lib_util_drivers_RPLidarJNI.h"
#include "com_team254_lib_util_drivers_RPLidarJNI_DataPoint.h"
#include "include/rplidar.h" //RPLIDAR standard sdk, all-in-one header

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
    rplidar_response_device_health_t healthinfo;

    u_result op_result = drv->getHealth(healthinfo);
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

JNIEXPORT jint JNICALL Java_com_team254_lib_util_drivers_RPLidarJNI_grabRawScanData
  (JNIEnv *env, jobject obj, jdoubleArray distancesArr, jdoubleArray anglesArr, jlongArray timestampsArr) {
    // alias this constant to use a shorter name :P
    constexpr size_t BUFFER_LEN = com_team254_lib_util_drivers_RPLidarJNI_DATA_BUFFER_LENGTH;
    
    // get scan data via the SDK's driver
    rplidar_response_measurement_node_t nodes[BUFFER_LEN];
    size_t count = BUFFER_LEN;
    u_result op_result = drv->grabScanData(nodes, count); // get the data
    if (IS_FAIL(op_result)) return -1;
    op_result = drv->ascendScanData(nodes, count); // sort by angle
    if (IS_FAIL(op_result)) return -1;
    
    jint returnVal = (jint) count;
    
    // get pointers to the array data
    jdouble* tmp_distances = (jdouble*) env->GetPrimitiveArrayCritical(distancesArr, NULL);
    jdouble* tmp_angles = (jdouble*) env->GetPrimitiveArrayCritical(anglesArr, NULL);
    jlong* tmp_timestamps = (jlong*) env->GetPrimitiveArrayCritical(timestampsArr, NULL);
    
    if (tmp_distances==NULL || tmp_angles==NULL || tmp_timestamps==NULL) {
        // out of memory exception thrown (do we need to deal with that?)
        returnVal = -1;
    } else {
        // extract the data into the arrays
        for (int i = 0; i < (int) count; i++) {
            tmp_distances[i] = static_cast<jdouble>( nodes[i].distance_q2/4.0 );
            tmp_angles[i] = static_cast<jdouble>( (nodes[i].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0 );
            tmp_timestamps[i] = static_cast<jlong>( nodes[i].timestamp );
        }
    }
    
    // release the pointers we got, and copy the data back if necessary
    if (tmp_timestamps) env->ReleasePrimitiveArrayCritical(timestampsArr, tmp_timestamps, 0);
    if (tmp_angles)     env->ReleasePrimitiveArrayCritical(anglesArr, tmp_angles, 0);
    if (tmp_distances)  env->ReleasePrimitiveArrayCritical(distancesArr, tmp_distances, 0);
    
    return returnVal;
}

JNIEXPORT void JNICALL Java_com_team254_lib_util_drivers_RPLidarJNI_stop
  (JNIEnv *env, jobject obj) {
    drv->stop();
    drv->stopMotor();
}