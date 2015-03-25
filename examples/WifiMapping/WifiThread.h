/*
Copyright (c) 2010-2014, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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

#include <sys/socket.h>
#include <linux/wireless.h>
#include <sys/ioctl.h>
#include <rtabmap/core/UserDataEvent.h>
#include <rtabmap/utilite/UTimer.h>

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
				int level = ((iw_statistics *)req.u.data.pointer)->qual.level - 256;
				double stamp = UTimer::now();

				// Create user data [level, stamp] with the value (int = 4 bytes) and a timestamp (double = 8 bytes)
				std::vector<unsigned char> data(sizeof(int) + sizeof(double));
				memcpy(data.data(), &level, sizeof(int));
				memcpy(data.data()+sizeof(int), &stamp, sizeof(double));
				this->post(new UserDataEvent(data));
				//UWARN("posting level %d dBm", level);
			}
			else
			{
				UERROR("Could not get signal level.");
			}

			close(sockfd);
		}
	}

private:
	std::string interfaceName_;
	float rate_;
};

#endif /* WIFITHREAD_H_ */
