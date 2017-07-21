//********************************************************************************************************
//
// © 2016 Regents of the University of California on behalf of the University of California at Berkeley
//       with rights granted for USDOT OSADP distribution with the ECL-2.0 open source license.
//
//*********************************************************************************************************
/* msgTransceiver.cpp
 * functions
 * 1. receive over-the-air DSRC messages (BSM and SRM), and forward to the MRP_DataMgr
 * 2. receive UDP messages (MAP, SPaT, SSM) from MRP_DataMgr, and manage to broadcast over-the-air
*/

#include <algorithm>
#include <csignal>
#include <cstddef>
#include <stdint.h>     /// c++11 <cstdint>
#include <csignal>
#include <cstring>
#include <fstream>
#include <iostream>
#include <poll.h>
#include <string>
#include <vector>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/types.h>

#include "libsocket.h"

#include "cnfUtils.h"
#include "timeUtils.h"
#include "msgTransceiver.h"

#define UNUSED(x) (void)(x)

static volatile std::sig_atomic_t terminate = 0;
static void sighandler(int signum) {terminate = signum;};

/// WME callback functions
struct savariwme_cbs wme_cbs;
/// handler of savari_wme_handler_t
savari_wme_handler_t handler[2] = {FAIL, FAIL};
/// trace registration status to the WME stack
std::vector<registration_t> registrationStatus;
/// broadcast MAC address
const uint8_t broadcast_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

/// socket on eth0 interface
int socket_fd;
struct sockaddr_in dest_addr;

timeUtils::fullTimeStamp_t fullTimeStamp;
bool log2file;
std::ofstream OS_Display;
std::ofstream OS_Rx;
std::ofstream OS_Tx;
unsigned long logrows_Rx = 0;
unsigned long logrows_Tx = 0;

void do_usage(const char* progname)
{
	std::cerr << progname << "Usage: " << std::endl;
	std::cerr << "\t-s full path to rsu.conf" << std::endl;
	std::cerr << "\t-? print this message" << std::endl;
	exit(EXIT_FAILURE);
}

int main (int argc, char** argv)
{
	int option;
	std::string cnfFile;

	while ((option = getopt(argc, argv, "s:?")) != EOF)
	{
		switch(option)
		{
		case 's':
			cnfFile = std::string(optarg);
			break;
		case '?':
		default:
			do_usage(argv[0]);
			break;
		}
	}
	if (cnfFile.empty())
		do_usage(argv[0]);

	timeUtils::getFullTimeStamp(fullTimeStamp);
	/// instance class ComponentCnf to read configuration file
	ComponentCnf* pmycnf = new ComponentCnf(cnfFile);
	if (!pmycnf->isInitiated())
	{
		std::cerr << fullTimeStamp.localDateTimeStamp.to_dateTimeStr('-', ':');
		std::cerr << ", failed initiating ComponentCnf " << cnfFile << std::endl;
		delete pmycnf;
		return(-1);
	}
	int fileInterval = pmycnf->getIntegerParaValue(std::string("logInterval")) * 60;   // in seconds
	std::string logPath = pmycnf->getStringParaValue(std::string("logPath"));
	ComponentCnf::Address_t addr = pmycnf->getAddr(std::string("DataMgr"));
	log2file = (fileInterval == 0) ? false : true;
	unsigned long long logInterval = ((log2file) ? fileInterval : 900) * 1000;
	delete pmycnf;

	/// open error log
	std::string errLog = logPath + std::string("/rsu.err");
	std::ofstream OS_ERR(errLog.c_str(), std::ofstream::app);
	if (!OS_ERR.is_open())
	{
		std::cerr << fullTimeStamp.localDateTimeStamp.to_dateTimeStr('-', ':');
		std::cerr << ", failed initiating err log, exit!" << std::endl;
		return(-1);
	}

	/// open display log (for debugging purpose, keep the latest 15 minutes record)
	std::string displayLog = logPath + std::string("/display.log");
	OS_Display.open(displayLog.c_str());
	if (!OS_Display.is_open())
	{
		OS_ERR << fullTimeStamp.localDateTimeStamp.to_dateTimeStr('-', ':');
		OS_ERR << ", failed initiating display log, exit!" << std::endl;
		OS_ERR.close();
		return(-1);
	}

  /// open log file
	std::string txLog = logPath + std::string("/tx.") + fullTimeStamp.localDateTimeStamp.to_fileName();
	std::string rxLog = logPath + std::string("/rx.") + fullTimeStamp.localDateTimeStamp.to_fileName();
	if (log2file)
	{
		OS_Tx.open(txLog.c_str(), std::ofstream::out | std::ofstream::binary);
		if (!OS_Tx.is_open())
		{
			OS_ERR << fullTimeStamp.localDateTimeStamp.to_dateTimeStr('-', ':');
			OS_ERR << ", failed open Tx log, exit!" << std::endl;
			OS_ERR.close();
			OS_Display.close();
			return(-1);
		}
		OS_Rx.open(rxLog.c_str(), std::ofstream::out | std::ofstream::binary);
		if (!OS_Rx.is_open())
		{
			OS_ERR << fullTimeStamp.localDateTimeStamp.to_dateTimeStr('-', ':');
			OS_ERR << ", failed open Rx log, exit!" << std::endl;
			OS_ERR.close();
			OS_Display.close();
			OS_Tx.close();
			return(-1);
		}
	}
	unsigned long long logfile_msec = fullTimeStamp.msec;

	/// creat socket
	int addr_len = (int)sizeof(struct sockaddr_in);
	uint16_t listen_port = (uint16_t)std::atoi(addr.listenPort.c_str());
	uint16_t send_port = (uint16_t)std::atoi(addr.sendPort.c_str());
	socket_fd = udps_init(const_cast<char*>("eth0"), const_cast<char*>(addr.listenIP.c_str()), listen_port, &addr_len);
	if (socket_fd == FAIL)
	{
		OS_ERR << fullTimeStamp.localDateTimeStamp.to_dateTimeStr('-', ':');
		OS_ERR << ", failed creating eth0 socket, exit!" << std::endl;
		OS_ERR.close();
		OS_Display.close();
		if (log2file)
		{
			OS_Tx.close();
			OS_Rx.close();
		}
		udps_deinit(socket_fd);
		return(-1);
	}
	dest_addr.sin_family = AF_INET;
	dest_addr.sin_addr.s_addr = inet_addr(addr.sendIP.c_str());
	dest_addr.sin_port = htons(send_port);

	/// assign WME callback functions
	wme_cbs.wme_provider_confirm = &savari_provider_confirm;
	wme_cbs.wme_user_confirm = &savari_user_confirm;
	wme_cbs.wme_wsm_indication = &savari_wsm_indication;

  /// connect to the WME stack
	bool has_error = false;
	struct savariwme_reg_req wmereq;
	init_wme_req(wmereq);
	int local_service_index = 0;
  for (int i = 0; i < 2; i++)
  {
		wmeUtils::ifaceType iface = ((i == 0) ? wmeUtils::ath0 : wmeUtils::ath1);
		std::string ifaceName = ((iface == wmeUtils::ath0) ? std::string("ath0") : std::string("ath1"));
		savari_wme_handler_t& hid = handler[i];
		if ((hid = wme_init(const_cast<char*>("::1"), const_cast<char*>(ifaceName.c_str()))) == FAIL)
    {
      OS_ERR << fullTimeStamp.localDateTimeStamp.to_dateTimeStr('-', ':');
      OS_ERR << ", failed wme_init for interface " << ifaceName << std::endl;
      OS_Display << fullTimeStamp.localDateTimeStamp.to_dateTimeStr('-', ':');
      OS_Display << ", failed wme_init for interface " << ifaceName << std::endl;
			has_error = true;
			break;
    }
		wmeUtils::appRole registered_role = wmeUtils::wme_iface_cfg[i].role;
		/// register PSIDs
		std::vector<wmeUtils::wme_channel_cfg_t>::const_iterator it;
		for (it = wmeUtils::wmeAppChannelMap.begin(); it != wmeUtils::wmeAppChannelMap.end(); ++it)
		{
			if (it->iface != iface)
				continue;
			/// assign tranceiving direction: outbound - SPaT, MAP & SSM; inbound - BSM & SRM
			wmeUtils::direction transm_direction = ((it->msgName == "SPaT") || (it->msgName == "MAP") || (it->msgName == "SSM"))
				? wmeUtils::TX : wmeUtils::RX;
			wmereq.channel = it->channel;
			wmereq.psid = wme_convert_psid_be(it->psid);
			wmereq.priority = it->priority;
			if (it->txMode == wmeUtils::continuous)
			{
				wmereq.request_type    = SAVARI1609_USER_AUTOACCESS_ONMATCH;
				wmereq.extended_access = 0xFFFF;
				wmereq.channel_access  = SAVARI1609_CHANNEL_ACCESS_CONTINUOUS;
			}
			else
			{
				wmereq.request_type    = SAVARI1609_USER_AUTOACCESS_UNCOND;
				wmereq.extended_access = 0;
				wmereq.channel_access  = SAVARI1609_CHANNEL_ACCESS_ALTERNATING;
			}
			wmereq.immediate_access = 0;
			wmereq.wsatype = SAVARI1609_WSA_UNSECURED;
			wmereq.local_service_index = ++local_service_index;
			wmereq.secondradio = i;
			registration_t mreg = {i, false, registered_role, transm_direction, wmereq};
			registrationStatus.push_back(mreg);
			/// register as user or provider
			int ret = ((registered_role == wmeUtils::user) ? wme_register_user(hid, &wmereq) : wme_register_provider(hid, &wmereq));
			if (ret == FAIL)
			{
				OS_ERR << fullTimeStamp.localDateTimeStamp.to_dateTimeStr('-', ':');
				OS_ERR << ", failed wme_register for message " << it->msgName << " on interface " << ifaceName << std::endl;
				OS_Display << fullTimeStamp.localDateTimeStamp.to_dateTimeStr('-', ':');
				OS_Display << ", failed wme_register for message " << it->msgName << " on interface " << ifaceName << std::endl;
				has_error = true;
				break;
			}
		}
		if (has_error)
			break;
	}
	if (has_error)
	{
		OS_ERR << fullTimeStamp.localDateTimeStamp.to_dateTimeStr('-', ':');
		OS_ERR << ", failed connect to WME stack, exit!" << std::endl;
		OS_Display << fullTimeStamp.localDateTimeStamp.to_dateTimeStr('-', ':');
		OS_Display << ", failed connect to WME stack, exit!" << std::endl;
		OS_ERR.close();
		OS_Display.close();
		if (log2file)
		{
			OS_Tx.close();
			OS_Rx.close();
		}
		udps_deinit(socket_fd);
		deinit_wme();
		return(-1);
	}

	/// intercepts signals
	std::signal(SIGABRT, sighandler);
	std::signal(SIGFPE,  sighandler);
	std::signal(SIGINT,  sighandler);
	std::signal(SIGSEGV, sighandler);
	std::signal(SIGTERM, sighandler);

	/// UDP receiving buffer on eth0
	std::vector<uint8_t> recvbuf(2000, 0);

	/// set up sockets poll structure
	nfds_t nfds = 3;
	struct pollfd ufds[3];
	ufds[0].fd = socket_fd;
	ufds[1].fd = handler[0];
	ufds[2].fd = handler[1];
	for (nfds_t i = 0; i < nfds; i++)
		ufds[i].events = POLLIN;

	while(terminate == 0)
	{ /// wait for events
		int retval = poll(ufds, nfds, -1);
		timeUtils::getFullTimeStamp(fullTimeStamp);
		if (retval > 0)
		{
			for (nfds_t i = 0; i < nfds; i++)
			{
				if ((ufds[i].revents & POLLIN) != POLLIN)
					continue;
				int fd = ufds[i].fd;
				if (fd == socket_fd)
				{ /// udp packet available on eth0 interface
					struct sockaddr_in their_addr;
					socklen_t len = sizeof(their_addr);
					ssize_t bytesReceived = udp_recv(socket_fd, (void*)&recvbuf[0], recvbuf.size(), (struct sockaddr *)&their_addr, &len);
					if (bytesReceived >= 9)
					{ /// MMITSS header + message body
						uint32_t psid;
						size_t offset = 0;
						msgUtils::mmitss_udp_header_t udpHeader;
						msgUtils::unpackHeader(recvbuf, offset, udpHeader);
						if ((udpHeader.msgheader == msgUtils::msg_header) && ((psid = wmeUtils::getpsidbymsgid(udpHeader.msgid)) > 0))
						{
							uint32_t psid_be = wme_convert_psid_be(psid);
							std::vector<registration_t>::const_iterator it;
							for (it = registrationStatus.begin(); it != registrationStatus.end(); ++it)
							{
								if ((it->registered_req.psid == psid_be) && (it->transm_direction == wmeUtils::TX))
								{
									std::vector<wmeUtils::wme_channel_cfg_t>::const_iterator ite;
									for (ite = wmeUtils::wmeAppChannelMap.begin(); ite != wmeUtils::wmeAppChannelMap.end(); ++ite)
									{
										if (ite->psid == psid)
										{
											savariwme_tx_req wmetx;
											wmetx.channel = ite->channel;
											wmetx.psid = wme_convert_psid_be(psid);
											wmetx.priority = ite->priority;
											wmetx.datarate = 6; //3Mbps
											wmetx.txpower = 15; //in dbM
											std::memcpy(wmetx.mac, broadcast_mac, SAVARI1609_IEEE80211_ADDR_LEN);
											wmetx.expiry_time = 0;
											wmetx.element_id = WAVE_ELEMID_WSMP;
											wmetx.tx_length = static_cast<int>(udpHeader.length);
											wmetx.supp_enable = 0;
											wmetx.safetysupp = 0;
											wme_wsm_tx(handler[static_cast<uint8_t>(ite->iface)], &wmetx, &recvbuf[offset]);
											if (log2file)
											{
												OS_Tx.write((char *)&recvbuf[0], offset);
												OS_Tx << std::endl;
												logrows_Tx++;
											}
											OS_Display << fullTimeStamp.localDateTimeStamp.to_dateTimeStr('-', ':');
											OS_Display << ", transmit outbound " << ite->msgName << std::endl;
											break;
										}
									}
									break;
								}
							}
						}
					}
				}
				else if ((fd == handler[0]) || (fd == handler[1]))
				{ /// data available on ath0 or ath1 interface, invoke WME callback functions
					int idx = (fd == handler[0]) ? 0 : 1;
					wme_rx(fd, &wme_cbs, &idx);
				}
			}
		}
		/// check reopen logfiles
		if (fullTimeStamp.msec > logfile_msec + logInterval)
		{
			if (log2file)
			{
				OS_Tx.close();
				OS_Rx.close();
				if (logrows_Tx == 0)
					std::remove(txLog.c_str());
				if (logrows_Rx == 0)
					std::remove(rxLog.c_str());
				txLog = logPath + std::string("/tx.") + fullTimeStamp.localDateTimeStamp.to_fileName();
				rxLog = logPath + std::string("/rx.") + fullTimeStamp.localDateTimeStamp.to_fileName();
				OS_Tx.open(txLog.c_str(), std::ofstream::out | std::ofstream::binary);
				OS_Rx.open(rxLog.c_str(), std::ofstream::out | std::ofstream::binary);
				logrows_Tx = 0;
				logrows_Rx = 0;
			}
			OS_Display.close();
			OS_Display.open(displayLog.c_str());
			logfile_msec = fullTimeStamp.msec;
		}
	}
	/// exit
	timeUtils::getFullTimeStamp(fullTimeStamp);
	OS_ERR << fullTimeStamp.localDateTimeStamp.to_dateTimeStr('-', ':');
	OS_ERR << ", received user termination signal, exit!" << std::endl;
	OS_Display << fullTimeStamp.localDateTimeStamp.to_dateTimeStr('-', ':');
	OS_Display << ", received user termination signal, exit!" << std::endl;
	OS_ERR.close();
	OS_Display.close();
	if (log2file)
	{
		OS_Tx.close();
		OS_Rx.close();
		if (logrows_Tx == 0)
			std::remove(txLog.c_str());
		if (logrows_Rx == 0)
			std::remove(rxLog.c_str());
	}
	udps_deinit(socket_fd);
	deinit_wme();
	return(0);
}

void init_wme_req(struct savariwme_reg_req& wme_req)
{
	std::string psc("MMITSS");
	std::memset(&wme_req, 0, sizeof(wme_req));
	std::memcpy(wme_req.destmacaddr, broadcast_mac, SAVARI1609_IEEE80211_ADDR_LEN);
	wme_req.psc_length = static_cast<int>(psc.size() + 1);
	std::memcpy(wme_req.psc, psc.c_str(), wme_req.psc_length);
}

void deinit_wme(void)
{
	std::vector<registration_t>::iterator it;
	for (it = registrationStatus.begin(); it != registrationStatus.end(); ++it)
	{
		if (it->registered_role == wmeUtils::user)
			wme_unregister_user(handler[it->handler_index], &(it->registered_req));
		else
			wme_unregister_provider(handler[it->handler_index], &(it->registered_req));
	}
	registrationStatus.clear();
	for (int i = 0; i < 2; i++)
	{
		savari_wme_handler_t hid = handler[i];
		if (hid != FAIL)
			wme_deinit(hid);
	}
}

/// savari_user_confirm - invoked by the WME layer to indicate the status of wme_register_user.
void savari_user_confirm(void* ctx, int confirm)
{
	int idx = *(int *)ctx;
	if (confirm != SAVARI1609_RC_ACCEPTED)
		return;
	std::vector<registration_t>::iterator it;
	for (it = registrationStatus.begin(); it != registrationStatus.end(); ++it)
	{
		if ((it->handler_index == idx) && (it->registered_role == wmeUtils::user) && !it->isConfirmed)
		{
			wme_user_service_confirm(handler[it->handler_index], SAVARI1609_ACTION_ADD, &(it->registered_req));
			it->isConfirmed = true;
			timeUtils::getFullTimeStamp(fullTimeStamp);
			OS_Display << fullTimeStamp.localDateTimeStamp.to_dateTimeStr('-', ':');
			OS_Display << ", user_confirm for pisd " << std::hex << std::uppercase;
			OS_Display << wme_convert_psid_be(it->registered_req.psid) << std::dec << std::endl;
			break;
		}
	}
}

/// savari_provider_confirm - invoked by the WME layer to indicate the status of wme_register_provider.
void savari_provider_confirm(void* ctx, int confirm)
{
	int idx = *(int *)ctx;
	if (confirm != SAVARI1609_RC_ACCEPTED)
		return;
	std::vector<registration_t>::iterator it;
	for (it = registrationStatus.begin(); it != registrationStatus.end(); ++it)
	{
		if ((it->handler_index == idx) && (it->registered_role == wmeUtils::provider) && !it->isConfirmed)
		{
			wme_provider_service_confirm(handler[it->handler_index], SAVARI1609_ACTION_ADD, &(it->registered_req));
			it->isConfirmed = true;
			timeUtils::getFullTimeStamp(fullTimeStamp);
			OS_Display << fullTimeStamp.localDateTimeStamp.to_dateTimeStr('-', ':');
			OS_Display << ", provider_confirm for pisd " << std::hex << std::uppercase;
			OS_Display << wme_convert_psid_be(it->registered_req.psid) << std::dec << std::endl;
			break;
		}
	}
}

/// savari_wsm_indication - invoked by the WME layer to indicate WSM packet matching based on psid.
void savari_wsm_indication(void* ctx, struct savariwme_rx_indication* rxind)
{
	UNUSED(ctx);
	static std::vector<uint8_t> sendbuf(2000, 0);
	static uint8_t msgId = 0;
	int psid_len = wme_getpsidlen(rxind->psid);
	uint32_t psid = 0;
	for (int i = 0; i < psid_len; i++)
		psid = (psid << 8) | rxind->psid[i];
	uint32_t psid_be = wme_convert_psid_be(psid);
	std::vector<registration_t>::const_iterator it;
	for (it = registrationStatus.begin(); it != registrationStatus.end(); ++it)
	{
		if ((it->registered_req.psid == psid_be) && (it->transm_direction == wmeUtils::RX)
			&& ((msgId = wmeUtils::getmsgidbypsid(psid)) > 0))
		{ /// pack message to send to MRP_DataMgr
			timeUtils::getFullTimeStamp(fullTimeStamp);
			size_t offset = 0;
			msgUtils::packHeader(sendbuf, offset, msgId, fullTimeStamp.localDateTimeStamp.msOfDay, (uint16_t)rxind->num_rx);
			std::memcpy(&sendbuf[offset], rxind->rx_buf, rxind->num_rx);
			udp_send(socket_fd, (void*)&sendbuf[0], offset + rxind->num_rx, (struct sockaddr*)&dest_addr, sizeof(dest_addr));
			if (log2file)
			{
				OS_Rx.write((char *)&sendbuf[0], offset);
				OS_Rx << std::endl;
				logrows_Rx++;
			}
			OS_Display << fullTimeStamp.localDateTimeStamp.to_dateTimeStr('-', ':');
			OS_Display << ", transmit inbound psid " << std::hex << std::uppercase;
			OS_Display << psid << std::dec << std::endl;
		}
	}
}
