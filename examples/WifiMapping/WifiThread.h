/*
Copyright (c) 2010-2016, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Universite de Sherbrooke nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef WIFITHREAD_H_
#define WIFITHREAD_H_

#ifdef _WIN32
	#ifndef UNICODE
	#define UNICODE
	#endif

	#include <windows.h>
	#include <wlanapi.h>
	#include <Windot11.h>           // for DOT11_SSID struct
	#include <objbase.h>
	#include <wtypes.h>

	#include <stdio.h>
	#include <stdlib.h>

	// Need to link with Wlanapi.lib and Ole32.lib
	#pragma comment(lib, "wlanapi.lib")
	#pragma comment(lib, "ole32.lib")
#elif __APPLE__
#include "WifiOSX.h"
#else
	#include <sys/socket.h>
	#include <linux/wireless.h>
	#include <sys/ioctl.h>
#endif
#include <rtabmap/core/UserDataEvent.h>
#include <rtabmap/utilite/UTimer.h>

// A percentage value that represents the signal quality
// of the network. WLAN_SIGNAL_QUALITY is of type ULONG.
// This member contains a value between 0 and 100. A value
// of 0 implies an actual RSSI signal strength of -100 dbm.
// A value of 100 implies an actual RSSI signal strength of -50 dbm.
// You can calculate the RSSI signal strength value for wlanSignalQuality
// values between 1 and 99 using linear interpolation.
inline int quality2dBm(int quality)
{
	// Quality to dBm:
	if(quality <= 0)
		return -100;
	else if(quality >= 100)
		return -50;
	else
		return (quality / 2) - 100;
}


class WifiThread : public UThread, public UEventsSender
{
public:
	WifiThread(const std::string & interfaceName, float rate = 0.5) :
		interfaceName_(interfaceName),
		rate_(rate)
	{}
	virtual ~WifiThread() {}

private:
	virtual void mainLoop()
	{
		uSleep(1000/rate_);
		if(!this->isKilled())
		{
			int dBm = 0;
#ifdef _WIN32
			//From https://msdn.microsoft.com/en-us/library/windows/desktop/ms706765(v=vs.85).aspx
			// Declare and initialize variables.
			HANDLE hClient = NULL;
			DWORD dwMaxClient = 2;      //
			DWORD dwCurVersion = 0;
			DWORD dwResult = 0;

			// variables used for WlanEnumInterfaces
			PWLAN_INTERFACE_INFO_LIST pIfList = NULL;
			PWLAN_INTERFACE_INFO pIfInfo = NULL;

			// variables used for WlanQueryInterfaces for opcode = wlan_intf_opcode_current_connection
			PWLAN_CONNECTION_ATTRIBUTES pConnectInfo = NULL;
			DWORD connectInfoSize = sizeof(WLAN_CONNECTION_ATTRIBUTES);
			WLAN_OPCODE_VALUE_TYPE opCode = wlan_opcode_value_type_invalid;

			dwResult = WlanOpenHandle(dwMaxClient, NULL, &dwCurVersion, &hClient);
			if (dwResult != ERROR_SUCCESS)
			{
				UERROR("WlanOpenHandle failed with error: %u\n", dwResult);
			}
			else
			{
				dwResult = WlanEnumInterfaces(hClient, NULL, &pIfList);
				if (dwResult != ERROR_SUCCESS)
				{
					UERROR("WlanEnumInterfaces failed with error: %u\n", dwResult);
				}
				else
				{
					// take the first interface found
					int i = 0;
					pIfInfo = (WLAN_INTERFACE_INFO *) & pIfList->InterfaceInfo[i];
					if(pIfInfo->isState == wlan_interface_state_connected)
					{
						dwResult = WlanQueryInterface(hClient,
													  &pIfInfo->InterfaceGuid,
													  wlan_intf_opcode_current_connection,
													  NULL,
													  &connectInfoSize,
													  (PVOID *) &pConnectInfo,
													  &opCode);

						if (dwResult != ERROR_SUCCESS)
						{
							UERROR("WlanQueryInterface failed with error: %u\n", dwResult);
						}
						else
						{
							int quality = pConnectInfo->wlanAssociationAttributes.wlanSignalQuality;
							dBm = quality2dBm(quality);
						}
					}
					else
					{
						UERROR("Interface not connected!");
					}
				}
			}
			if (pConnectInfo != NULL)
			{
				WlanFreeMemory(pConnectInfo);
				pConnectInfo = NULL;
			}

			if (pIfList != NULL)
			{
				WlanFreeMemory(pIfList);
				pIfList = NULL;
			}
#elif __APPLE__
			dBm = getRssi(interfaceName_);
#else
			// Code inspired from http://blog.ajhodges.com/2011/10/using-ioctl-to-gather-wifi-information.html

			//have to use a socket for ioctl
			int sockfd;
			/* Any old socket will do, and a datagram socket is pretty cheap */
			if((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) == -1) {
				UERROR("Could not create simple datagram socket");
				return;
			}

			struct iwreq req;
			struct iw_statistics stats;

			strncpy(req.ifr_name, interfaceName_.c_str(), IFNAMSIZ);

			//make room for the iw_statistics object
			req.u.data.pointer = (caddr_t) &stats;
			req.u.data.length = sizeof(stats);
			// clear updated flag
			req.u.data.flags = 1;

			//this will gather the signal strength
			if(ioctl(sockfd, SIOCGIWSTATS, &req) == -1)
			{
				//die with error, invalid interface
				UERROR("Invalid interface (\"%s\"). Tip: Try with sudo!", interfaceName_.c_str());
			}
			else if(((iw_statistics *)req.u.data.pointer)->qual.updated & IW_QUAL_DBM)
			{
				//signal is measured in dBm and is valid for us to use
				dBm = ((iw_statistics *)req.u.data.pointer)->qual.level - 256;
			}
			else
			{
				UERROR("Could not get signal level.");
			}

			close(sockfd);
#endif
			if(dBm != 0)
			{
				double stamp = UTimer::now();

				// Create user data [level, stamp] with the value and a timestamp
				cv::Mat data(1, 2, CV_64FC1);
				data.at<double>(0) = double(dBm);
				data.at<double>(1) = stamp;
				this->post(new UserDataEvent(data));
				//UWARN("posting level %d dBm", dBm);
			}
		}
	}

private:
	std::string interfaceName_;
	float rate_;
};

#endif /* WIFITHREAD_H_ */
