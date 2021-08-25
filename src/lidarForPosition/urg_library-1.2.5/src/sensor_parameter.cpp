/*!
  \file
  \brief 返回传感器参数
  \author 

  $Id$
*/

#include "Urg_driver.h"
#include "Connection_information.h"
#include <iostream>
#include "sensor_parameter.h"

using namespace std;
using namespace qrk;

/*	1.首先要明确PrintfDeviceInformation(Urg_driver urg)是传值方式传参，实参要向形参复制对象，在复制对象时会调用拷贝构造函数。
	2.由于Urg_driver类中定义拷贝构造函数为private，将使用默认拷贝构造函数不会自动复制堆资源（即通过new得到的资源）。
	3.因此PrintfDeviceInformation(Urg_driver urg){}执行时，无法访问private方法
	解决的办法有：1.使用引用传参，
*/
/**
  *  该函数用于打印当前设备信息状态
  *  param information 连接信息
  *  retval none
*/
void PrintfDeviceInformation(const Urg_driver& urg)
{
	/*返回传感器模型字符串。返回与传感器模型对应的字符串消息。此消息取决于传感器。
	 *retval：传感器模型串*/
    cout << "Sensor product type: " << urg.product_type() << endl;
	/*返回当前传感器固件版本字符串。返回与当前传感器固件版本对应的字符串消息。此消息取决于传感器。
	 *retval：固件版本字符串*/
    cout << "Sensor firmware version: " << urg.firmware_version() << endl;
	/*返回传感器序列号字符串。返回与传感器序列号对应的字符串消息。此消息取决于传感器。
	 *retval：序列号字符串*/
    cout << "Sensor serial ID: " << urg.serial_id() << endl;
	/*返回当前传感器状态字符串。返回与当前传感器状态对应的字符串消息。此消息取决于传感器。
	 *retval：   当前传感器状态字符串*/
    cout << "Sensor status: " << urg.status() << endl;
	/*返回当前传感器状态字符串。返回与当前传感器状态对应的字符串消息。此消息取决于传感器。
	 *retval：   电流传感器状态串
	 *Attention 有关详细信息，请参阅SCIP通信协议规范*/
    cout << "Sensor state: " << urg.state() << endl;
	/*返回当前传感器最大步长 最小步长*/
    cout << "step: ["
         << urg.min_step() << ", "
         << urg.max_step() << "]" << endl;
    cout << "distance: ["
         << urg.min_distance()
         << ", " << urg.max_distance() <<  "]" << endl;

    cout << "scan interval: " << urg.scan_usec() << " [usec]" << endl;
    cout << "sensor data size: " << urg.max_data_size() << endl;
}
