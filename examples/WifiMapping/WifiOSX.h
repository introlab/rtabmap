/*
 * WifiOSX.h
 *
 *  Created on: Mar 26, 2015
 *      Author: mathieu
 */

#ifndef WIFIOSX_H_
#define WIFIOSX_H_

#include <string>
#include <vector>

struct AccessPoint
{
    std::string ssid;
    std::string bssid;
    int rssi;
};

int getRssi(const std::string& interfaceName);
std::vector<AccessPoint> scanAir(const std::string& interfaceName);


#endif /* WIFIOSX_H_ */
