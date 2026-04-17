#include "LiDAR.h"
#include "sl_lidar.h" 
#include "sl_lidar_driver.h"

static inline void delay_ms(sl_word_size_t ms){
    while (ms>=1000){
        usleep(1000*1000);
        ms-=1000;
    };
    if (ms!=0)
        usleep(ms*1000);
}

int main() {
	IChannel* _channel;
	ILidarDriver * drv = *createLidarDriver();
    sl_lidar_response_device_info_t devinfo;

	_channel = (*createSerialPortChannel(opt_channel_param_first, opt_channel_param_second));
    
	if (SL_IS_OK((drv)->connect(_channel))) {
        op_result = drv->getDeviceInfo(devinfo);

        if (SL_IS_OK(op_result)) 
        {
	        connectSuccess = true;
        }
        else{
            delete drv;
			drv = NULL;
        }
    }


	LiDAR lidar = new LiDAR("/dev/ttyUSB0", (int)BAUDRATE);
    drv->startScan(0,1);

    // fetech result and print it out...
    while (1) {
        sl_lidar_response_measurement_node_hq_t nodes[8192];
        size_t   count = _countof(nodes);

        op_result = drv->grabScanDataHq(nodes, count);

        if (SL_IS_OK(op_result)) {
            drv->ascendScanData(nodes, count);
            for (int pos = 0; pos < (int)count ; ++pos) {
                printf("%s theta: %03.2f Dist: %08.2f Q: %d \n", 
                    (nodes[pos].flag & SL_LIDAR_RESP_HQ_FLAG_SYNCBIT) ?"S ":"  ", 
                    (nodes[pos].angle_z_q14 * 90.f) / 16384.f,
                    nodes[pos].dist_mm_q2/4.0f,
                    nodes[pos].quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
            }
        }

        if (ctrl_c_pressed){ 
            break;
        }
    }

    drv->stop();
	delay_ms(200);

	if(opt_channel_type == CHANNEL_TYPE_SERIALPORT) drv->setMotorSpeed(0);

	if(drv) {
        delete drv;
        drv = NULL;
    }
    return 0;
}