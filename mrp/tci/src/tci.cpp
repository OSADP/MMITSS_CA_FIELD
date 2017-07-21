//********************************************************************************************************
//
// © 2016 Regents of the University of California on behalf of the University of California at Berkeley
//       with rights granted for USDOT OSADP distribution with the ECL-2.0 open source license.
//
//*********************************************************************************************************
/* tci.cpp - MRP_TrafficControllerInterface (TCI)
 * functions:
 * 1. poll controller's configuration data and populate Class Card data elements (poll_list ==> timing_card_t).
 *    Class Card is defined in timeCard.h and implemented in timeCard.cpp.
 *    Configuration of sending poll request and handling controller's response to poll is defined in cntlrPolls.h,
 *      and implemented in cntlrPolls.cpp
 *    a) Polling the full sets of controller's configuration data only need to perform once at the startup of MRP_TCI.
 *    b) When controller changed the control plan number (including coordination and running-free), MRP_TCI polls the
 *       parameters associated with the current control plan to ensure the current parameters stored in controller's
 *       memory are used by other MRP components for handling priority and MMITSS traffic control, e.g. MRP_Aware.
 * 2. receive controller's pushing out messages: signal_status_mess_t, status8e_mess_t, longstatus8e_mess_t
 * 3. trace the status of controller and signal, and estimate the remaining times of vehicular and pedestrian phases (controller_status_t)
 * 4. send UDP messages (msgid_cntrlstatus, msgid_detCnt & msgid_detPres) to MRP_DataMgr
 * 5. receive UDP messages (msgid_softcall) from MRP_DataMgr and manage sending soft-call request to the traffic controller
 * Structures for UDP messages are defined in msgUtils.h. For potability, all UDP messages are serialized.
 * Functions to pack and unpack UDP messages are defined in msgUtils.h, and implemented in msgUtils.cpp and ab3418msgs.cpp
 * Structures for AB3418 messages are defined in ab3418msgs.h. Functions to parse and form AB3418 messages are defined in
 * ab3418msgs.h, and implemented in ab3418msgs.cpp and ab3418fcs.cpp (FCS - error detection).
 * logs:
 * 1. simpleLog
 *    - received soft-call requests
 *    - soft-call request sent to the traffic controller
 * 2. detailLog - simpleLog plus
 *    - controller's pushing out signal status data
 *    - controller and signal status message sent to MRP_DataMgr
 *    - detector count/occupancy message sent to MRP_DataMgr
 *    - detector presence message sent to MRP_DataMgr
 * 3. polled controller's configuration data in intersectionName.timecard
 *
 */

#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <fcntl.h>
#include <poll.h>
#include <string>
#include <sys/stat.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <termios.h>
#include <thread>
#include <unistd.h>

#include "ab3418fcs.h"
#include "cnfUtils.h"
#include "cntlrPolls.h"
#include "logUtils.h"
#include "msgUtils.h"
#include "socketUtils.h"
#include "tci.h"

void do_usage(const char* progname)
{
	std::cerr << "Usage" << progname << std::endl;
	std::cerr << "\t-n intersection name" << std::endl;
	std::cerr << "\t-s full path to mrpTci.conf" << std::endl;
	std::cerr << "\t-v turn on verbose" << std::endl;
	std::cerr << "\t-? print this message" << std::endl;
	exit(EXIT_FAILURE);
}

auto barrier_phases_on = [](const std::bitset<8>& check_phases)->uint8_t
{
	std::bitset<8> left_barrier(std::string("00110011"));
	left_barrier &= check_phases;
	return((left_barrier.any()) ? 0 : 1);
};
auto ring_phase_on = [](int check_phase){return((check_phase <= 4) ? 0 : 1);};
auto next_ring     = [](uint8_t check_ring){return(static_cast<uint8_t>((check_ring + 1) % 2));};
auto next_barrier  = [](uint8_t check_barrier){return(static_cast<uint8_t>((check_barrier + 1) % 2));};

static volatile std::sig_atomic_t terminate = 0;
static void sighandler(int signum) {terminate = signum;};

int main(int argc, char** argv)
{
	int option;
	std::string intersectionName;
	std::string cnfFile;
	bool verbose = false;

	while ((option = getopt(argc, argv, "s:n:v?")) != EOF)
	{
		switch(option)
		{
		case 's':
			cnfFile = std::string(optarg);
			break;
		case 'n':
			intersectionName = std::string(optarg);
			break;
		case 'v':
			verbose = true;
			break;
		case '?':
		default:
			do_usage(argv[0]);
			break;
		}
	}
	if (cnfFile.empty() || intersectionName.empty())
		do_usage(argv[0]);

	/* ----------- preparation -------------------------------------*/
	timeUtils::fullTimeStamp_t fullTimeStamp;
	timeUtils::getFullTimeStamp(fullTimeStamp);
	timeUtils::dateStamp_t dateStamp = fullTimeStamp.localDateTimeStamp.dateStamp;

	/// instance class ComponentCnf to read configuration file
	ComponentCnf* pmycnf = new ComponentCnf(cnfFile);
	if (!pmycnf->isInitiated())
	{
		std::cerr << fullTimeStamp.localDateTimeStamp.to_dateTimeStr('-', ':');
		std::cerr << ", failed initiating ComponentCnf " << cnfFile << std::endl;
		delete pmycnf;
		return(-1);
	}
	bool send2controller = (pmycnf->getIntegerParaValue(std::string("sendCommand")) == 1) ? true : false;
	unsigned long long logInterval = pmycnf->getIntegerParaValue(std::string("logInterval")) * 60 * 1000;  // in milliseconds
	int logType = pmycnf->getIntegerParaValue(std::string("logType"));
	logUtils::logType log_type = ((logInterval == 0) || ((logType != 1) && (logType != 2)))
		? logUtils::logType::none : static_cast<logUtils::logType>(logType);
	std::string spatPort  = pmycnf->getStringParaValue(std::string("spatPort"));
	std::string spat2Port = pmycnf->getStringParaValue(std::string("spat2Port"));
	std::string logPath   = pmycnf->getStringParaValue(std::string("logPath"));
	std::string cardName  = pmycnf->getStringParaValue(std::string("timeCardPath"))
		+ std::string("/") + intersectionName + std::string(".timecard");
	unsigned long long logfile_msec = 0;

	/// open error log
	std::ofstream OS_ERR(logPath + std::string("/tci.err"), std::ofstream::app);
	if (!OS_ERR.is_open())
	{
		std::cerr << fullTimeStamp.localDateTimeStamp.to_dateTimeStr('-', ':');
		std::cerr << ", failed initiating err log" << std::endl;
		delete pmycnf;
		return(-1);
	}

	/// open log files
	std::vector<logUtils::Logfile_t> logFiles;
	if (log_type != logUtils::logType::none)
	{
		std::vector<std::string> logtypes;
		logtypes.push_back(std::string("sigRaw")); // log controller pushing out controller and signal status message
		logtypes.push_back(std::string("call"));   // log soft-call message send to controller
		logtypes.push_back(std::string("sig"));    // log controller status message send to MRP_DataMgr
		logtypes.push_back(std::string("cnt"));    // log count/volume message send to MRP_DataMgr
		logtypes.push_back(std::string("pres"));   // log detector presence message send to MRP_DataMgr
		logtypes.push_back(std::string("req"));    // log soft-call request message received from MRP_DataMgr
		std::string prefix = logPath + std::string("/") + intersectionName;
		for (auto& type : logtypes)
			{logFiles.push_back(logUtils::Logfile_t(prefix, type));}
		if (!logUtils::openLogFiles(logFiles, fullTimeStamp.localDateTimeStamp.to_fileName()))
		{
			OS_ERR << fullTimeStamp.localDateTimeStamp.to_dateTimeStr('-', ':');
			OS_ERR << ", failed openLogFiles" << std::endl;
			OS_ERR.close();
			logUtils::closeLogFiles(logFiles);
			delete pmycnf;
			return(-1);
		}
		logfile_msec = fullTimeStamp.msec;
	}

	/// open sockets (bidirectional from/to DataMgr)
	if (!pmycnf->connectAll())
	{
		OS_ERR << fullTimeStamp.localDateTimeStamp.to_dateTimeStr('-', ':');
		OS_ERR << ", failed initiating sockets" << std::endl;
		OS_ERR.close();
		if (log_type != logUtils::logType::none)
			logUtils::closeLogFiles(logFiles);
		delete pmycnf;
		return(-1);
	}
	socketUtils::Conn_t sendConn = pmycnf->getSocketConn(std::string("toDataMgr"));
	int fd_Listen = pmycnf->getSocketDescriptor(std::string("fromDataMgr"));

	/// open serial ports
	int fd_spat = open_port(spatPort, true);
	int fd_spat2 = open_port(spat2Port, false);
	if ((fd_spat < 0) || (fd_spat2 < 0))
	{
		OS_ERR << fullTimeStamp.localDateTimeStamp.to_dateTimeStr('-', ':');
		OS_ERR << ", failed open port" << ((fd_spat < 0) ? spatPort : std::string());
		OS_ERR << std::string(" ") << ((fd_spat2 < 0) ? spat2Port : std::string()) << std::endl;
		OS_ERR.close();
		if (log_type != logUtils::logType::none)
			logUtils::closeLogFiles(logFiles);
		pmycnf->disconnectAll();
		delete pmycnf;
		if (fd_spat > 0)
			close_port(fd_spat, true);
		if (fd_spat2 > 0)
			close_port(fd_spat2, false);
		return(-1);
	}

	/// instance Card class to store timing card data
	Card* pcard = new Card();
	Card::phaseflags_mess_t phaseFlags;
	Card::freeplan_mess_t   freePlan;
	std::vector<Card::phasetiming_mess_t> phaseTimings;
	std::vector<Card::coordplan_mess_t>   coordPlans;
	/// instance Polls class to build timing card
	bool pollTimeCard = true;
	const unsigned long long poll_interval = 500;   // in milliseconds
	const int maxpolls_per_request = 5;
	Polls* pPolls = new Polls(maxpolls_per_request, poll_interval);
	pPolls->getNextPoll(true);

	/* ----------- intercepts signals -------------------------------------*/
	std::signal(SIGABRT, sighandler);
	std::signal(SIGFPE,  sighandler);
	std::signal(SIGINT,  sighandler);
	std::signal(SIGSEGV, sighandler);
	std::signal(SIGTERM, sighandler);

	/* ----------- local variables -------------------------------------*/
	/// structure to trace controller status
	controller_status_t controller_status;
	std::memset(&controller_status,0,sizeof(controller_status));
	controller_status.isPlantimingReady = false;
	controller_status.mode = MsgEnum::controlMode::unavailable;
	controller_status.coordplan_index = -1;

	/// serial port read buffer
	const size_t maxAB3418msgSize = 512;
	std::vector<uint8_t> recvbuf_spat(maxAB3418msgSize, 0);
	std::vector<uint8_t> recvbuf_spat2(maxAB3418msgSize, 0);
	/// serial port write buffer
	std::vector<uint8_t> sendbuf_spat2(maxAB3418msgSize, 0);
	/// read all available bytes from serial port, variables to manage received half of a packet
	std::vector<uint8_t> msgbuf(maxAB3418msgSize, 0);
	std::vector< std::pair<size_t, size_t> > ab3418Frame_spat;
	std::vector< std::pair<size_t, size_t> > ab3418Frame_spat2;
	size_t bytenums_spat = 0;
	size_t bytenums_spat2 = 0;
	/// structure to hold inbound pushing out ab3418 messages on serial ports
	AB3418MSG::signal_status_mess_t  signal_status_mess;
	AB3418MSG::status8e_mess_t       status8e_mess;
	AB3418MSG::longstatus8e_mess_t   longstatus8e_mess;
	/// receive and send socket buffer
	const size_t maxUDPmsgSize = 2000;
	std::vector<uint8_t> recvbuf_socket(maxUDPmsgSize, 0);
	std::vector<uint8_t> sendbuf_socket(maxUDPmsgSize, 0);

	/// interval to send soft-call to the traffic controller
	unsigned long long softcall_interval = 20;  // in milliseconds
	/// structure to trace soft-call state
	softcall_state_t softcall_state;
	softcall_state.reset();

	/// set up serial ports and sockets poll structure
	nfds_t nfds = 3;
	struct pollfd ufds[3];
	ufds[0].fd = fd_spat;
	ufds[1].fd = fd_spat2;
	ufds[2].fd = fd_Listen;
	for (nfds_t i = 0; i < nfds; i++)
		ufds[i].events = POLLIN;
  int pollTimeout = 20; // in milliseconds, B38400 = 48 bytes in 10 milliseconds

	while(terminate == 0)
	{
		bool process_spat = false;
		bool process_spat2 = false;
		int retval = poll(ufds, nfds, pollTimeout);
		timeUtils::getFullTimeStamp(fullTimeStamp);
		if (dateStamp != fullTimeStamp.localDateTimeStamp.dateStamp)
		{ /// poll controller configuration data once per day
			dateStamp = fullTimeStamp.localDateTimeStamp.dateStamp;
			if (!pollTimeCard)
			{
				pollTimeCard = true;
				pPolls->resetPollReturn();
			}
		}

		if (retval > 0)
		{
			for (nfds_t i = 0; i < nfds; i++)
			{
				if ((ufds[i].revents & POLLIN) != POLLIN)
					continue;
				if (ufds[i].fd == fd_spat)
				{	/// events on fd_spat, read all available bytes
					ssize_t bytes_read = read(fd_spat, (void*)&recvbuf_spat[bytenums_spat], maxAB3418msgSize - bytenums_spat);
					if (bytes_read > 0)
					{
						bytenums_spat += bytes_read;
						processRecvBuff(recvbuf_spat, bytenums_spat, ab3418Frame_spat);
						if (!ab3418Frame_spat.empty())
							process_spat = true;
					}
				}
				else if (ufds[i].fd == fd_spat2)
				{	/// events on fd_spat2, read all available bytes
					ssize_t bytes_read = read(fd_spat2, (void*)&recvbuf_spat2[bytenums_spat2], maxAB3418msgSize - bytenums_spat2);
					if (bytes_read > 0)
					{
						bytenums_spat2 += bytes_read;
						processRecvBuff(recvbuf_spat2, bytenums_spat2, ab3418Frame_spat2);
						if (!ab3418Frame_spat2.empty())
							process_spat2 = true;
					}
				}
				else if (ufds[i].fd == fd_Listen)
				{	/// MMITSS header + message body
					ssize_t bytesReceived = recv(fd_Listen, (void*)&recvbuf_socket[0], recvbuf_socket.size(), 0);
					if (bytesReceived >= 9)
					{
						size_t offset = 0;
						msgUtils::mmitss_udp_header_t udpHeader;
						msgUtils::unpackHeader(recvbuf_socket, offset, udpHeader);
						if ((udpHeader.msgheader == msgUtils::msg_header) && (udpHeader.msgid == msgUtils::msgid_softcall))
						{
							msgDefs::softcall_request_t softcall_request;
							softcall_request.ms_since_midnight = udpHeader.ms_since_midnight;
							msgDefs::unpackMsg(recvbuf_socket, offset, softcall_request);
							updateSoftcallState(softcall_state, softcall_request);
							if (log_type != logUtils::logType::none)
								logUtils::logMsg(logFiles, std::string("req"), recvbuf_socket, (size_t)bytesReceived, fullTimeStamp.localDateTimeStamp.msOfDay);
						}
					}
				}
			}
		}

		bool isNewSpat = false;
		if (process_spat)
		{
			for (auto& item : ab3418Frame_spat)
			{
				if (item.second >= item.first + 7)
				{
					size_t frame_size = 0;
					bool fcs = AB3418checksum::processMsg(msgbuf, frame_size, recvbuf_spat, item);
					if ((msgbuf[4] == AB3418MSG::rawspatRes_messType) && fcs && (frame_size == AB3418MSG::rawspatRes_size))
					{ /// only expect rawspatRes_messType on fd_spat
						isNewSpat = true;
						AB3418MSG::parseMsg(signal_status_mess, msgbuf);
						if ((controller_status.controller_addr == 0x00) && (signal_status_mess.controller_addr != controller_status.controller_addr))
						{
							controller_status.controller_addr = signal_status_mess.controller_addr;
							pcard->setControllerAddr(controller_status.controller_addr);
						}
						if (log_type == logUtils::logType::detailLog)
						{
							size_t msgSize = AB3418MSG::packMsg(sendbuf_socket, signal_status_mess, msgUtils::msgid_signalraw, fullTimeStamp.localDateTimeStamp.msOfDay);
							logUtils::logMsg(logFiles, std::string("sigRaw"), sendbuf_socket, msgSize);
						}
					}
					else
					{
						OS_ERR << fullTimeStamp.localDateTimeStamp.to_dateTimeStr('-', ':');
						OS_ERR << ", failed spat processMsg:" << std::endl;
						OS_ERR << "  received: ";
						logUtils::logMsgHex(OS_ERR, &recvbuf_spat[item.first], item.second - item.first + 1);
						OS_ERR << "  parsed: ";
						logUtils::logMsgHex(OS_ERR, &msgbuf[0], frame_size);
					}
				}
			}
			/// reset bytenums_spat
			bool reset_spat = true;
			for (size_t i = ab3418Frame_spat.back().second + 1; i < bytenums_spat; i++)
			{ /// move the remaining array, starting with 0x7E, to the beginning of recvbuf_spat
				if (recvbuf_spat[i] == AB3418MSG::flag)
				{
					size_t offset = 0;
					for (size_t j = i; j < bytenums_spat; j++)
						recvbuf_spat[offset++] = recvbuf_spat[j];
					bytenums_spat = offset;
					reset_spat = false;
					break;
				}
			}
			if (reset_spat)
				bytenums_spat = 0;
		}

		if (process_spat2)
		{
			for (auto& item : ab3418Frame_spat2)
			{
				if (item.second >= item.first + 7)
				{
					size_t frame_size = 0;
					bool fcs = AB3418checksum::processMsg(msgbuf, frame_size, recvbuf_spat2, item);
					switch(msgbuf[4])
					{ /// check mess_type
					case AB3418MSG::status8eRes_messType:
						/// detector presences
						if (fcs && (frame_size == AB3418MSG::status8eRes_size))
						{
							AB3418MSG::parseMsg(status8e_mess, msgbuf);
							controller_status.status = status8e_mess.status;
							/// pack message and send to MRP_DataMgr
							size_t msgSize = AB3418MSG::packMsg(sendbuf_socket, status8e_mess, msgUtils::msgid_detPres, fullTimeStamp.localDateTimeStamp.msOfDay);
							bool sendFlag = socketUtils::sendall(sendConn, &sendbuf_socket[0], msgSize);
							/// log to file
							if (log_type == logUtils::logType::detailLog)
								logUtils::logMsg(logFiles, std::string("pres"), sendbuf_socket, msgSize);
							if (verbose)
							{
								std::cout << fullTimeStamp.localDateTimeStamp.to_dateTimeStr('-', ':');
								std::cout << ", sending msgid_detPres, " << std::boolalpha << sendFlag << std::endl;
							}
						}
						break;
					case AB3418MSG::longStatus8eRes_messType:
						/// detector count and occupancy
						if (fcs && (frame_size == AB3418MSG::longStatus8eRes_size))
						{
							AB3418MSG::parseMsg(longstatus8e_mess, msgbuf);
							/// pack message and send to MRP_DataMgr
							size_t msgSize = AB3418MSG::packMsg(sendbuf_socket, longstatus8e_mess, msgUtils::msgid_detCnt, fullTimeStamp.localDateTimeStamp.msOfDay);
							bool sendFlag = socketUtils::sendall(sendConn, &sendbuf_socket[0], msgSize);
							/// log to file
							if (log_type == logUtils::logType::detailLog)
								logUtils::logMsg(logFiles, std::string("cnt"), sendbuf_socket, msgSize);
							if (verbose)
							{
								std::cout << fullTimeStamp.localDateTimeStamp.to_dateTimeStr('-', ':');
								std::cout << ", sending msgid_detCnt, " << std::boolalpha << sendFlag << std::endl;
							}
						}
						break;
					case AB3418MSG::getBlockMsgRes_errMessType:
						/// getBlock request returned error (message includes pageId & blockId of getBlockMsg)
						if (pollTimeCard && fcs && (frame_size == AB3418MSG::errGetBlockRes_size))
						{
							std::string poll_desc = pPolls->getPollDesc(&msgbuf[5], msgbuf[4]);
							if (!poll_desc.empty())
							{
								OS_ERR << fullTimeStamp.localDateTimeStamp.to_dateTimeStr('-', ':');
								OS_ERR << ", getBlockMsg error: " << poll_desc;
								OS_ERR << ", err_num = " << static_cast<int>(msgbuf[7]);
								OS_ERR << ", err_code " << AB3418MSG::errCode(msgbuf[7]) << std::endl;
							}
						}
						break;
					case AB3418MSG::getTimingDataRes_errMessType:
						/// getTimingData request returned error (message includes error number and index number)
						if (pollTimeCard && fcs && (frame_size == AB3418MSG::errGetDataRes_size))
						{
							OS_ERR << fullTimeStamp.localDateTimeStamp.to_dateTimeStr('-', ':');
							OS_ERR << ", getTimingData err_num = " << static_cast<int>(msgbuf[5]);
							OS_ERR << ", err_code " << AB3418MSG::errCode(msgbuf[5]) << std::endl;
						}
						break;
					case AB3418MSG::setSoftcallRes_errMessType:
						/// setSoftcall request returned error (message includes error number and index number)
						if (fcs && (frame_size == AB3418MSG::errSetDataRes_size))
						{
							OS_ERR << fullTimeStamp.localDateTimeStamp.to_dateTimeStr('-', ':');
							OS_ERR << ", setSoftcall err_num = " << static_cast<int>(msgbuf[5]);
							OS_ERR << ", err_code " << AB3418MSG::errCode(msgbuf[5]) << std::endl;
						}
						break;
					case AB3418MSG::setSoftcallRes_messType:
						/// setSoftcall returned success
						break;
					case AB3418MSG::getTimingDataRes_messType:
						/// getTimingData request returned success
						if (pollTimeCard)
						{
							std::string poll_desc = pPolls->getPollDesc(&msgbuf[5], msgbuf[4], frame_size, fcs);
							if (!poll_desc.empty())
							{
								pcard->updateTimeCard(msgbuf, poll_desc);
								pPolls->setPollReturn(poll_desc);
								if (verbose)
								{
									std::cout << fullTimeStamp.localDateTimeStamp.to_dateTimeStr('-', ':');
									std::cout << ", " << poll_desc << " returned success" << std::endl;
								}
							}
						}
						break;
					case AB3418MSG::getBlockMsgRes_messType:
						/// getBlock request returned success
						if (pollTimeCard)
						{
							std::string poll_desc = pPolls->getPollDesc(&msgbuf[5], msgbuf[4], frame_size, fcs);
							if (!poll_desc.empty())
							{
								pcard->updateTimeCard(msgbuf, poll_desc);
								pPolls->setPollReturn(poll_desc);
								if (verbose)
								{
									std::cout << fullTimeStamp.localDateTimeStamp.to_dateTimeStr('-', ':');
									std::cout << ", " << poll_desc << " returned success" << std::endl;
								}
							}
						}
						break;
					default:
						OS_ERR << fullTimeStamp.localDateTimeStamp.to_dateTimeStr('-', ':');
						OS_ERR << ", failed spat2 processMsg:" << std::endl;
						OS_ERR << "  received: ";
						logUtils::logMsgHex(OS_ERR, &recvbuf_spat2[item.first], item.second - item.first + 1);
						OS_ERR << "  parsed: ";
						logUtils::logMsgHex(OS_ERR, &msgbuf[0], frame_size);
						break;
					}
				}
			}
			/// reset bytenums_spat2
			bool reset_spat2 = true;
			for (size_t i = ab3418Frame_spat2.back().second + 1; i < bytenums_spat2; i++)
			{
				if (recvbuf_spat2[i] == AB3418MSG::flag)
				{
					size_t offset = 0;
					for (size_t j = i; j < bytenums_spat2; j++)
						recvbuf_spat2[offset++] = recvbuf_spat2[j];
					bytenums_spat2 = offset;
					reset_spat2 = false;
					break;
				}
			}
			if (reset_spat2)
				bytenums_spat2 = 0;
		}

		/// handle controller polls
		if (pollTimeCard && pPolls->nextPoll()) /// move to the next poll
			pPolls->getNextPoll();
		if (pollTimeCard && pPolls->atEnd())
		{	/// have finished looping though the polling list
			if (pPolls->allReturned())
			{ /// all required polls have returned success
				if (verbose)
				{
					std::cout << fullTimeStamp.localDateTimeStamp.to_dateTimeStr('-', ':');
					std::cout << ", finished polling controller" << std::endl;
				}
				pollTimeCard = false;
				pcard->setInitiated();
				pcard->setFreePlanParameters();
				pcard->setCoordPlanParameters();
				std::string saveCardName = cardName + fullTimeStamp.localDateTimeStamp.to_dateStr('-');
				std::rename(cardName.c_str(), saveCardName.c_str());
				pcard->logTimeCard(cardName);
			}
			else if (!pPolls->setPollReturn())
			{ /// when failed polling controller data, read timing card instead
				OS_ERR << fullTimeStamp.localDateTimeStamp.to_dateTimeStr('-', ':');
				OS_ERR << ", reached maximum allowed polling cycles" << std::endl;
				pollTimeCard = false;
				if (!pcard->isInitiated() && !pcard->readTimeCard(cardName))
				{
					OS_ERR << fullTimeStamp.localDateTimeStamp.to_dateTimeStr('-', ':');
					OS_ERR << ", failed reading timing card, exit!" << std::endl;
					terminate = 0xFF;
					break;
				}
			}
			else
			{
				if (verbose)
				{
					std::cout << fullTimeStamp.localDateTimeStamp.to_dateTimeStr('-', ':');
					std::cout << ", repeat polling controller" << std::endl;
				}
				pPolls->getNextPoll(true);
			}
		}
		if (pollTimeCard && (controller_status.controller_addr != 0x00) && pPolls->sendPoll(fullTimeStamp.msec))
		{
			size_t nbyte = pPolls->packRequest(sendbuf_spat2, controller_status.controller_addr);
			if ((nbyte > 0) && (write(fd_spat2, (void*)&sendbuf_spat2[0], nbyte) > 0) && verbose)
			{
				std::string poll_desc = pPolls->getPollDesc();
				std::cout << fullTimeStamp.localDateTimeStamp.to_dateTimeStr('-', ':');
				std::cout	<< ", polling " << poll_desc << std::endl;
			}
		}
		if (!pollTimeCard && !controller_status.isPlantimingReady)
		{	/// get current timing parameters
			controller_status.msec = fullTimeStamp.msec;
			controller_status.status = status8e_mess.status;
			for (uint8_t ring = 0; ring < 2; ring++)
			{
				if (signal_status_mess.active_phases[ring] > 0)
					controller_status.active_force_off[signal_status_mess.active_phases[ring] - 1] = signal_status_mess.active_force_off[ring];
			}
			controller_status.mode = pcard->getControlMode(controller_status.status, signal_status_mess.preempt, signal_status_mess.pattern_num);
			if (!pcard->getPatnIdx(controller_status.coordplan_index, controller_status.mode, signal_status_mess.plan_num))
			{ /// should not be here as all plans have been polled
				OS_ERR << fullTimeStamp.localDateTimeStamp.to_dateTimeStr('-', ':');
				OS_ERR << ", failed find coordplan_index in time card for plan_num ";
				OS_ERR << static_cast<unsigned int>(signal_status_mess.plan_num);
				OS_ERR << ", re-poll coordination plans" << std::endl;
				pcard->resetPlans();
				pollTimeCard = true;
				pPolls->resetPlanPolls();
				pPolls->getNextPoll(true);
				continue;
			}
			/// initial controller status tracing
			pcard->getPermitPhases(controller_status.permitted_phases, controller_status.permitted_ped_phases, controller_status.coordplan_index);
			pcard->getSyncPhase(controller_status.coordinated_phases, controller_status.synch_phase, controller_status.coordplan_index);
			controller_status.cycle_length = pcard->getCycleLength(controller_status.coordplan_index);
			controller_status.curbarrier = barrier_phases_on(signal_status_mess.active_phase);
			controller_status.curbarrier_start_time = fullTimeStamp.msec;
			controller_status.timer_time[0] = fullTimeStamp.msec;
			controller_status.timer_time[1] = fullTimeStamp.msec;
			controller_status.cycle_start_time = fullTimeStamp.msec;
			controller_status.cycle_clock_time = fullTimeStamp.msec;
			/// phase_status
			for (uint8_t i = 0; i < 8; i++)
			{
				auto& phase_status = controller_status.phase_status[i];
				if (!controller_status.permitted_phases.test(i))
					phase_status.state = MsgEnum::phaseState::dark;
				else
				{
					phase_status.state = pcard->getPhaseState(controller_status.mode, signal_status_mess.active_phases[ring_phase_on(i+1)],
						signal_status_mess.active_interval[ring_phase_on(i+1)], (uint8_t)(i+1));
					phase_status.state_start_time = fullTimeStamp.msec;
					phase_status.call_status = MsgEnum::phaseCallType::none;
					phase_status.recall_status = pcard->getPhaseRecallType(controller_status.mode, controller_status.coordplan_index, i);
				}
				if (!controller_status.permitted_ped_phases.test(i))
					phase_status.pedstate = MsgEnum::phaseState::dark;
				else
				{
					phase_status.pedstate = pcard->getPedState(controller_status.mode, signal_status_mess.active_phases[ring_phase_on(i+1)],
						signal_status_mess.active_interval[ring_phase_on(i+1)], (uint8_t)(i+1));
					phase_status.pedstate_start_time = fullTimeStamp.msec;
				}
			}
			controller_status.signal_status = signal_status_mess;
			controller_status.isPlantimingReady = true;
			phaseFlags   = pcard->getPhaseFlags();
			freePlan     = pcard->getFreePlan();
			phaseTimings = pcard->getPhaseTiming();
			coordPlans   = pcard->getCoordPlans();
			if (verbose)
			{
				std::cout << fullTimeStamp.localDateTimeStamp.to_dateTimeStr('-', ':');
				std::cout << ", starting, signal is running mode " << static_cast<int>(controller_status.mode);
				if (controller_status.mode == MsgEnum::controlMode::coordination)
					std::cout << ", coordination plan_num " << static_cast<int>(coordPlans[controller_status.coordplan_index].plan_num);
				else
					std::cout << ", plan_num " << static_cast<int>(controller_status.signal_status.plan_num);
				std::cout << std::endl;
			}
		}
		if(pollTimeCard || !controller_status.isPlantimingReady)
			continue;

		/// finished all polls
		if (isNewSpat)
		{
			controller_status.msec = fullTimeStamp.msec;
			for (uint8_t ring = 0; ring < 2; ring++)
			{
				if (signal_status_mess.active_phases[ring] > 0)
					controller_status.active_force_off[signal_status_mess.active_phases[ring] - 1] = signal_status_mess.active_force_off[ring];
			}
			/// trace pattern_num change
			if (signal_status_mess.pattern_num != controller_status.signal_status.pattern_num)
			{	/// reset control plan parameters
				controller_status.isPlantimingReady = false;
				controller_status.mode = pcard->getControlMode(controller_status.status, signal_status_mess.preempt, signal_status_mess.pattern_num);
				if (!pcard->getPatnIdx(controller_status.coordplan_index, controller_status.mode, signal_status_mess.plan_num))
				{	/// should not be here as all plans have been polled
					OS_ERR << fullTimeStamp.localDateTimeStamp.to_dateTimeStr('-', ':');
					OS_ERR << ", plan change, failed find coordplan_index in time card for plan_num ";
					OS_ERR << static_cast<unsigned int>(signal_status_mess.plan_num);
					OS_ERR << ", re-poll coordination plans" << std::endl;
					pcard->resetPlans();
					pollTimeCard = true;
					pPolls->resetPlanPolls();
					pPolls->getNextPoll(true);
					continue;
				}
				/// reset permitted_phases & permitted_ped_phases
				pcard->getPermitPhases(controller_status.permitted_phases, controller_status.permitted_ped_phases, controller_status.coordplan_index);
				/// reset coordinated_phases & synch_phase
				pcard->getSyncPhase(controller_status.coordinated_phases, controller_status.synch_phase, controller_status.coordplan_index);
				controller_status.cycle_length = pcard->getCycleLength(controller_status.coordplan_index);
				/// reset phase recall_status
				for (uint8_t i = 0; i < 8; i++)
				{
					if (controller_status.permitted_phases.test(i))
						controller_status.phase_status[i].recall_status = pcard->getPhaseRecallType(controller_status.mode, controller_status.coordplan_index, i);
				}
				controller_status.isPlantimingReady = true;
			}
			/// trace barrier change
			uint8_t curbarrier = barrier_phases_on(signal_status_mess.active_phase);
			if (curbarrier != controller_status.curbarrier)
			{
				controller_status.curbarrier = curbarrier;
				controller_status.curbarrier_start_time = fullTimeStamp.msec;
			}
			/// trace local_cycle_clock change (local_cycle_clock stays 0 when running free)
			if (signal_status_mess.local_cycle_clock != controller_status.signal_status.local_cycle_clock)
				controller_status.cycle_clock_time = fullTimeStamp.msec;
			/// trace cycle change (under coordination)
			if ((signal_status_mess.local_cycle_clock < controller_status.signal_status.local_cycle_clock)
					&& (signal_status_mess.local_cycle_clock < 3))
				controller_status.cycle_start_time = (unsigned long long)(fullTimeStamp.msec - signal_status_mess.local_cycle_clock * 1000);
			/// trace active interval countdown timer change
			for (uint8_t ring = 0; ring < 2; ring++)
			{
				if ((signal_status_mess.active_phases[ring] != controller_status.signal_status.active_phases[ring])
						|| (signal_status_mess.active_interval[ring] != controller_status.signal_status.active_interval[ring])
						|| (signal_status_mess.interval_timer[ring] != controller_status.signal_status.interval_timer[ring]))
					controller_status.timer_time[ring] = fullTimeStamp.msec;
			}
			/// trace phase_status change
			for (uint8_t i = 0; i < 8; i++)
			{
				if (controller_status.permitted_phases.test(i))
				{
					auto& phase_status = controller_status.phase_status[i];
					MsgEnum::phaseState state = pcard->getPhaseState(controller_status.mode, signal_status_mess.active_phases[ring_phase_on(i+1)],
						signal_status_mess.active_interval[ring_phase_on(i+1)], (uint8_t)(i+1));
					/// state & state_start_time
					if (state != phase_status.state)
					{
						phase_status.state = state;
						phase_status.state_start_time = fullTimeStamp.msec;
					}
					/// call_status
					phase_status.call_status = MsgEnum::phaseCallType::none;
					if (signal_status_mess.veh_call.test(i))
						phase_status.call_status = MsgEnum::phaseCallType::vehicle;
					/// ped_call has higher priority than veh_call
					if (signal_status_mess.ped_call.test(i))
						phase_status.call_status = MsgEnum::phaseCallType::ped;
				}
				if (controller_status.permitted_ped_phases.test(i))
				{
					auto& phase_status = controller_status.phase_status[i];
					MsgEnum::phaseState pedstate = pcard->getPedState(controller_status.mode, signal_status_mess.active_phases[ring_phase_on(i+1)],
						signal_status_mess.active_interval[ring_phase_on(i+1)], (uint8_t)(i+1));
					if (pedstate != phase_status.pedstate)
					{
						phase_status.pedstate = pedstate;
						phase_status.pedstate_start_time = fullTimeStamp.msec;
					}
				}
			}

			/// update controller_status.phase_status.time2next & pedtime2next (no need for red flashing mode)
			predicted_bound_t time2start[2]; // phase green onset by ring
			if (controller_status.mode == MsgEnum::controlMode::runningFree)
			{	/// when running free, there is no local_cycle_clock (stays at 0) and no force-off logic
				/// phase green is terminated by either gap-out or max-out
				controller_status.cur_local_cycle_clock = 0;
				/// start with active phases
				for (uint8_t ring = 0; ring < 2; ring++)
				{
					uint8_t phaseOnRing = signal_status_mess.active_phases[ring];
					if (phaseOnRing > 0)
					{
						auto& phase_status = controller_status.phase_status[phaseOnRing-1];
						const auto& phasetiming = phaseTimings[phaseOnRing-1];
						updateActivePhaseTime2next(phase_status, time2start[ring], signal_status_mess,
							phasetiming, phaseFlags, ring, controller_status.timer_time[ring], fullTimeStamp.msec);
					}
				}
				/// determine start-barrier & start-phases for moving barrier-to-barrier, phase-to-phase
				uint8_t startbarrier = curbarrier;
				uint8_t startphases[2] = {signal_status_mess.active_phases[0], signal_status_mess.active_phases[1]};
				/// when next_phase is on (active phase in yellow or red clearance), update time2next for next_phase & time2start for the phases after
				if (signal_status_mess.next_phase.any())
				{	/// next_phase is on when at least one active phases is in yellow or red clearance
					uint8_t nextbarrier = barrier_phases_on(signal_status_mess.next_phase);
					if (nextbarrier != curbarrier)
					{
						barrierCrossAdjust(time2start);
						startbarrier = nextbarrier;
					}
					for (uint8_t ring = 0; ring < 2; ring++)
					{
						const uint8_t&  ringPhase = signal_status_mess.next_phases[ring];
						if (startbarrier != curbarrier)
							startphases[ring] = ringPhase;
						if (ringPhase > 0)
						{ /// update time2next (i.e., red to green) for next_phases
							startphases[ring] = ringPhase;
							auto& phase_status = controller_status.phase_status[ringPhase-1];
							auto& phasetiming = phaseTimings[ringPhase-1];
							if (phase_status.state == MsgEnum::phaseState::redLight)
							{
								phase_status.time2next.bound_L = time2start[ring].bound_L;
								phase_status.time2next.bound_U = time2start[ring].bound_U;
							}
							/// next_phase should be on
							if (phase_status.call_status == MsgEnum::phaseCallType::none)
								phase_status.call_status = MsgEnum::phaseCallType::vehicle;
							/// update time2start for phases after the next_phase
							getNextPhaseStartBound(time2start[ring], phasetiming, phaseFlags, (uint8_t)(ringPhase - 1), phase_status);
						}
					}
				}
				/// phase after start-phases and on start-barrier
				for (uint8_t ring = 0; ring < 2; ring++)
				{
					uint8_t lagphase = freePlan.leadlag_phases[startbarrier][ring][1];
					if ((lagphase > 0) && (lagphase != startphases[ring]) && (lagphase != signal_status_mess.active_phases[ring]))
					{
						auto& phase_status = controller_status.phase_status[lagphase-1];
						const auto& phasetiming = phaseTimings[lagphase-1];
						phase_status.time2next.bound_L = time2start[ring].bound_L;
						phase_status.time2next.bound_U = time2start[ring].bound_U;
						getNextPhaseStartBound(time2start[ring], phasetiming, phaseFlags, (uint8_t)(lagphase-1), phase_status);
					}
				}
				barrierCrossAdjust(time2start);
				if (startbarrier == curbarrier)
				{	/// for phases on the next barrier
					for (uint8_t ring = 0; ring < 2; ring++)
					{
						uint8_t leadphase = freePlan.leadlag_phases[next_barrier(startbarrier)][ring][0];
						uint8_t lagphase  = freePlan.leadlag_phases[next_barrier(startbarrier)][ring][1];
						if (leadphase > 0)
						{
							auto& phase_status = controller_status.phase_status[leadphase-1];
							const auto& phasetiming = phaseTimings[leadphase-1];
							phase_status.time2next.bound_L = time2start[ring].bound_L;
							phase_status.time2next.bound_U = time2start[ring].bound_U;
							getNextPhaseStartBound(time2start[ring], phasetiming, phaseFlags, (uint8_t)(leadphase-1), phase_status);
						}
						if ((lagphase > 0) && (lagphase != leadphase))
						{
							auto& phase_status = controller_status.phase_status[lagphase-1];
							const auto& phasetiming = phaseTimings[lagphase-1];
							phase_status.time2next.bound_L = time2start[ring].bound_L;
							phase_status.time2next.bound_U = time2start[ring].bound_U;
							getNextPhaseStartBound(time2start[ring], phasetiming, phaseFlags, (uint8_t)(lagphase-1), phase_status);
						}
					}
					barrierCrossAdjust(time2start);
					/// for remaining phases on the start-barrier (i.e. current barrier)
					for (uint8_t ring = 0; ring < 2; ring++)
					{
						uint8_t leadphase = freePlan.leadlag_phases[startbarrier][ring][0];
						uint8_t lagphase  = freePlan.leadlag_phases[startbarrier][ring][1];
						if (leadphase > 0)
						{
							auto& phase_status = controller_status.phase_status[leadphase-1];
							if (((leadphase == signal_status_mess.active_phases[ring]) && (phase_status.state == MsgEnum::phaseState::redLight))
								|| ((leadphase != signal_status_mess.active_phases[ring]) && (leadphase != startphases[ring])))
							{
								const auto& phasetiming = phaseTimings[leadphase-1];
								phase_status.time2next.bound_L = time2start[ring].bound_L;
								phase_status.time2next.bound_U = time2start[ring].bound_U;
								getNextPhaseStartBound(time2start[ring], phasetiming, phaseFlags, (uint8_t)(leadphase-1), phase_status);
							}
						}
						if ((lagphase > 0) && (lagphase != leadphase))
						{
							auto& phase_status = controller_status.phase_status[lagphase-1];
							if ((lagphase == signal_status_mess.active_phases[ring]) && (phase_status.state == MsgEnum::phaseState::redLight))
							{
								const auto& phasetiming = phaseTimings[lagphase-1];
								phase_status.time2next.bound_L = time2start[ring].bound_L;
								phase_status.time2next.bound_U = time2start[ring].bound_U;
								getNextPhaseStartBound(time2start[ring], phasetiming, phaseFlags, (uint8_t)(lagphase-1), phase_status);
							}
						}
					}
				}
				else
				{	/// start-phases on different barrier from active-phase
					/// for remaining phases on the current barrier
					for (uint8_t ring = 0; ring < 2; ring++)
					{
						uint8_t leadphase = freePlan.leadlag_phases[curbarrier][ring][0];
						uint8_t lagphase  = freePlan.leadlag_phases[curbarrier][ring][1];
						if (leadphase > 0)
						{
							auto& phase_status = controller_status.phase_status[leadphase-1];
							if (((leadphase == signal_status_mess.active_phases[ring]) && (phase_status.state == MsgEnum::phaseState::redLight))
								|| ((leadphase != signal_status_mess.active_phases[ring]) && (leadphase != startphases[ring])))
							{
								const auto& phasetiming = phaseTimings[leadphase-1];
								phase_status.time2next.bound_L = time2start[ring].bound_L;
								phase_status.time2next.bound_U = time2start[ring].bound_U;
								getNextPhaseStartBound(time2start[ring], phasetiming, phaseFlags, (uint8_t)(leadphase-1), phase_status);
							}
						}
						if ((lagphase > 0) && (lagphase != leadphase))
						{
							auto& phase_status = controller_status.phase_status[lagphase-1];
							if (((lagphase == signal_status_mess.active_phases[ring]) && (phase_status.state == MsgEnum::phaseState::redLight))
								|| (leadphase == signal_status_mess.active_phases[ring]))
							{
								const auto& phasetiming = phaseTimings[lagphase-1];
								phase_status.time2next.bound_L = time2start[ring].bound_L;
								phase_status.time2next.bound_U = time2start[ring].bound_U;
								getNextPhaseStartBound(time2start[ring], phasetiming, phaseFlags, (uint8_t)(lagphase-1), phase_status);
							}
						}
					}
					/// for remaining phase on start-barrier
					if ( ((startphases[0] > 0) && freePlan.lag_phases.test(startphases[0] - 1))
						|| ((startphases[1] > 0) && freePlan.lag_phases.test(startphases[1] - 1)) )
					{
						barrierCrossAdjust(time2start);
						for (uint8_t ring = 0; ring < 2; ring++)
						{
							uint8_t leadphase = freePlan.leadlag_phases[startbarrier][ring][0];
							if ((leadphase > 0) && (leadphase != startphases[ring]))
							{
								auto& phase_status = controller_status.phase_status[leadphase-1];
								const auto& phasetiming = phaseTimings[leadphase-1];
								phase_status.time2next.bound_L = time2start[ring].bound_L;
								phase_status.time2next.bound_U = time2start[ring].bound_U;
								getNextPhaseStartBound(time2start[ring], phasetiming, phaseFlags, (uint8_t)(leadphase-1), phase_status);
							}
						}
					}
				}
			}
			else if (controller_status.mode == MsgEnum::controlMode::coordination)
			{	/// when under coordination, phase green is terminated by gap-out, force-off or max-out
				const auto& coordplan  = coordPlans[controller_status.coordplan_index];
				/// get the cur_local_cycle_clock in deciseconds (0 < cur_local_cycle_clock <= cycle_length)
				controller_status.cur_local_cycle_clock = static_cast<uint16_t>(((fullTimeStamp.msec + 30 - controller_status.cycle_clock_time)/100
					+ signal_status_mess.local_cycle_clock * 10) % controller_status.cycle_length);
				/// get concurrent phase combination type (minorMinor, minorMajor, or majorMajor)
				Card::ConcurrentType concurrentType = pcard->getConcurrentPhaseType(signal_status_mess.active_phase, coordplan.sync_phases);
				/// active_force_off adjustment
				if (concurrentType == Card::ConcurrentType::minorMajor)
				{	/// lagging minorMajor should have the same force-off point
					const uint8_t& sync_ring = coordplan.sync_ring;
					uint8_t ring = next_ring(sync_ring);
					const auto& lagPhase  = signal_status_mess.active_phases[sync_ring];
					const auto& ringPhase = signal_status_mess.active_phases[ring];
					if ((lagPhase > 0) && (ringPhase > 0) && (coordplan.lag_phases.test(ringPhase-1))
						&& (signal_status_mess.active_force_off[0] != signal_status_mess.active_force_off[1]))
					{
						if (signal_status_mess.active_force_off[ring] < signal_status_mess.active_force_off[sync_ring])
							signal_status_mess.active_force_off[ring] = signal_status_mess.active_force_off[sync_ring];
					}
				}
				else if (concurrentType == Card::ConcurrentType::majorMajor)
				{	/// both coordinated phases have passed the yield point, they should have the same force-off point
					if ((coordplan.sync_phases.count() == 2)
						&& (signal_status_mess.active_force_off[0] > 0) && (signal_status_mess.active_force_off[1] > 0)
						&& (signal_status_mess.active_force_off[0] != signal_status_mess.active_force_off[1]))
					{
						if (signal_status_mess.active_force_off[0] > signal_status_mess.active_force_off[1])
							signal_status_mess.active_force_off[0] = signal_status_mess.active_force_off[1];
						signal_status_mess.active_force_off[1] = signal_status_mess.active_force_off[0];
					}
				}
				/// coordinated phases should be on
				for (uint8_t ring = 0; ring < 2; ring++)
				{
					const auto& ringPhase = signal_status_mess.active_phases[ring];
					if ((ringPhase > 0) && (ringPhase == coordplan.coordinated_phases[ring]))
					{
						auto& phase_status = controller_status.phase_status[ringPhase-1];
						if (phase_status.call_status == MsgEnum::phaseCallType::none)
							phase_status.call_status = MsgEnum::phaseCallType::vehicle;
					}
				}
				/// phase_status.time2next: start with active phases
				for (uint8_t ring = 0; ring < 2; ring++)
				{
					const uint8_t& phaseOnRing = signal_status_mess.active_phases[ring];
					if (phaseOnRing > 0)
					{
						auto& phase_status = controller_status.phase_status[phaseOnRing-1];
						const auto& phasetiming = phaseTimings[phaseOnRing-1];
						updateActivePhaseTime2next(phase_status, time2start[ring], coordplan, signal_status_mess, phasetiming,
							phaseFlags, ring, controller_status.cur_local_cycle_clock, controller_status.cycle_length, concurrentType,
							controller_status.timer_time[ring], fullTimeStamp.msec);
					}
				}
				/// determine start-barrier & start-phases for moving barrier-to-barrier, phase-to-phase
				uint8_t startbarrier = curbarrier;
				uint8_t startphases[2] = {signal_status_mess.active_phases[0],signal_status_mess.active_phases[1]};
				/// when next_phase is on (active phase in yellow or red clearance), update time2next for next_phase & time2start for the phases after
				if (signal_status_mess.next_phase.any())
				{	/// next_phase is on when at least one active phases is in yellow or red clearance
					uint8_t nextbarrier = barrier_phases_on(signal_status_mess.next_phase);
					if (nextbarrier != curbarrier)
					{
						barrierCrossAdjust(time2start);
						startbarrier = nextbarrier;
					}
					for (uint8_t ring = 0; ring < 2; ring++)
					{
						const uint8_t&  ringPhase = signal_status_mess.next_phases[ring];
						if (startbarrier != curbarrier)
							startphases[ring] = ringPhase;
						if (ringPhase > 0)
						{ /// update time2next (i.e., red to green) for next_phases
							startphases[ring] = ringPhase;
							auto& phase_status = controller_status.phase_status[ringPhase-1];
							const auto& phasetiming = phaseTimings[ringPhase-1];
							if (phase_status.state == MsgEnum::phaseState::redLight)
							{
								phase_status.time2next.bound_L = time2start[ring].bound_L;
								phase_status.time2next.bound_U = time2start[ring].bound_U;
							}
							/// next_phase should be on
							if (phase_status.call_status == MsgEnum::phaseCallType::none)
								phase_status.call_status = MsgEnum::phaseCallType::vehicle;
							/// update time2start for phases after the next_phase
							getNextPhaseStartBound(time2start[ring], phasetiming, phaseFlags, (uint8_t)(ringPhase-1),
								phase_status, coordplan, controller_status.cur_local_cycle_clock, controller_status.cycle_length);
						}
					}
				}
				/// phase after start-phases and on start-barrier
				for (uint8_t ring = 0; ring < 2; ring++)
				{
					uint8_t lagphase = coordplan.leadlag_phases[startbarrier][ring][1];
					if ((lagphase > 0) && (lagphase != startphases[ring]) && (lagphase != signal_status_mess.active_phases[ring]))
					{
						auto& phase_status = controller_status.phase_status[lagphase-1];
						const auto& phasetiming = phaseTimings[lagphase-1];
						phase_status.time2next.bound_L = time2start[ring].bound_L;
						phase_status.time2next.bound_U = time2start[ring].bound_U;
						getNextPhaseStartBound(time2start[ring], phasetiming, phaseFlags, (uint8_t)(lagphase-1),
							phase_status, coordplan, controller_status.cur_local_cycle_clock, controller_status.cycle_length);
					}
				}
				barrierCrossAdjust(time2start);
				if (startbarrier == curbarrier)
				{	/// for phases on the next barrier
					for (uint8_t ring = 0; ring < 2; ring++)
					{
						uint8_t leadphase = coordplan.leadlag_phases[next_barrier(startbarrier)][ring][0];
						uint8_t lagphase  = coordplan.leadlag_phases[next_barrier(startbarrier)][ring][1];
						if (leadphase > 0)
						{
							auto& phase_status = controller_status.phase_status[leadphase-1];
							const auto& phasetiming = phaseTimings[leadphase-1];
							phase_status.time2next.bound_L = time2start[ring].bound_L;
							phase_status.time2next.bound_U = time2start[ring].bound_U;
							getNextPhaseStartBound(time2start[ring], phasetiming, phaseFlags, (uint8_t)(leadphase-1),
								phase_status, coordplan, controller_status.cur_local_cycle_clock, controller_status.cycle_length);
						}
						if ((lagphase > 0) && (lagphase != leadphase))
						{
							auto& phase_status = controller_status.phase_status[lagphase-1];
							const auto& phasetiming = phaseTimings[lagphase-1];
							phase_status.time2next.bound_L = time2start[ring].bound_L;
							phase_status.time2next.bound_U = time2start[ring].bound_U;
							getNextPhaseStartBound(time2start[ring], phasetiming, phaseFlags, (uint8_t)(lagphase-1),
								phase_status, coordplan, controller_status.cur_local_cycle_clock, controller_status.cycle_length);
						}
					}
					barrierCrossAdjust(time2start);
					/// for remaining phases on the start-barrier (i.e. current barrier)
					for (uint8_t ring = 0; ring < 2; ring++)
					{
						uint8_t leadphase = coordplan.leadlag_phases[startbarrier][ring][0];
						uint8_t lagphase  = coordplan.leadlag_phases[startbarrier][ring][1];
						if (leadphase > 0)
						{
							auto& phase_status = controller_status.phase_status[leadphase-1];
							if (((leadphase == signal_status_mess.active_phases[ring]) && (phase_status.state == MsgEnum::phaseState::redLight))
								|| ((leadphase != signal_status_mess.active_phases[ring]) && (leadphase != startphases[ring])))
							{
								const auto& phasetiming = phaseTimings[leadphase-1];
								phase_status.time2next.bound_L = time2start[ring].bound_L;
								phase_status.time2next.bound_U = time2start[ring].bound_U;
								getNextPhaseStartBound(time2start[ring], phasetiming, phaseFlags, (uint8_t)(leadphase-1),
									phase_status, coordplan, controller_status.cur_local_cycle_clock, controller_status.cycle_length);
							}
						}
						if ((lagphase > 0) && (lagphase != leadphase))
						{
							auto& phase_status = controller_status.phase_status[lagphase-1];
							if ((lagphase == signal_status_mess.active_phases[ring]) && (phase_status.state == MsgEnum::phaseState::redLight))
							{
								const auto& phasetiming = phaseTimings[lagphase-1];
								phase_status.time2next.bound_L = time2start[ring].bound_L;
								phase_status.time2next.bound_U = time2start[ring].bound_U;
								getNextPhaseStartBound(time2start[ring], phasetiming, phaseFlags, (uint8_t)(lagphase-1),
									phase_status, coordplan, controller_status.cur_local_cycle_clock, controller_status.cycle_length);
							}
						}
					}
				}
				else
				{ /// start-phases on different barrier from active-phases
					/// for remaining phases on the current barrier
					for (uint8_t ring = 0; ring < 2; ring++)
					{
						uint8_t leadphase = coordplan.leadlag_phases[curbarrier][ring][0];
						uint8_t lagphase  = coordplan.leadlag_phases[curbarrier][ring][1];
						if (leadphase > 0)
						{
							auto& phase_status = controller_status.phase_status[leadphase-1];
							if (((leadphase == signal_status_mess.active_phases[ring]) && (phase_status.state == MsgEnum::phaseState::redLight))
								|| ((leadphase != signal_status_mess.active_phases[ring]) && (leadphase != startphases[ring])))
							{
								const auto& phasetiming = phaseTimings[leadphase-1];
								phase_status.time2next.bound_L = time2start[ring].bound_L;
								phase_status.time2next.bound_U = time2start[ring].bound_U;
								getNextPhaseStartBound(time2start[ring], phasetiming, phaseFlags, (uint8_t)(leadphase-1),
									phase_status, coordplan, controller_status.cur_local_cycle_clock, controller_status.cycle_length);
							}
						}
						if ((lagphase > 0) && (lagphase != leadphase))
						{
							auto& phase_status = controller_status.phase_status[lagphase-1];
							if (((lagphase == signal_status_mess.active_phases[ring]) && (phase_status.state == MsgEnum::phaseState::redLight))
								|| (leadphase == signal_status_mess.active_phases[ring]))
							{
								const auto& phasetiming = phaseTimings[lagphase-1];
								phase_status.time2next.bound_L = time2start[ring].bound_L;
								phase_status.time2next.bound_U = time2start[ring].bound_U;
								getNextPhaseStartBound(time2start[ring], phasetiming, phaseFlags, (uint8_t)(lagphase-1),
									phase_status, coordplan, controller_status.cur_local_cycle_clock, controller_status.cycle_length);
							}
						}
					}
					/// for remaining phase on start-barrier
					if ( ((startphases[0] > 0) && coordplan.lag_phases.test(startphases[0] - 1))
						|| ((startphases[1] > 0) && coordplan.lag_phases.test(startphases[1] - 1)) )
					{
						barrierCrossAdjust(time2start);
						for (uint8_t ring = 0; ring < 2; ring++)
						{
							uint8_t leadphase = coordplan.leadlag_phases[startbarrier][ring][0];
							if ((leadphase > 0) && (leadphase != startphases[ring]))
							{
								auto& phase_status = controller_status.phase_status[leadphase-1];
								const auto& phasetiming = phaseTimings[leadphase-1];
								phase_status.time2next.bound_L = time2start[ring].bound_L;
								phase_status.time2next.bound_U = time2start[ring].bound_U;
								getNextPhaseStartBound(time2start[ring], phasetiming, phaseFlags, (uint8_t)(leadphase-1),
									phase_status, coordplan, controller_status.cur_local_cycle_clock, controller_status.cycle_length);
							}
						}
					}
				}
			}

			/// phase_status.pedtime2next
			if (controller_status.mode != MsgEnum::controlMode::flashing)
			{
				for (uint8_t i = 0; i < 8; i++)
				{
					auto& phase_status = controller_status.phase_status[i];
					if (controller_status.permitted_ped_phases.test(i))
					{
						if (phase_status.pedstate != MsgEnum::phaseState::redLight)
						{	/// WALK or FLASH_DONOT_WALK
							phase_status.pedtime2next.bound_L = getPedIntervalLeft(signal_status_mess.interval_timer[ring_phase_on(i+1)],
								controller_status.timer_time[ring_phase_on(i+1)], fullTimeStamp.msec);
							phase_status.pedtime2next.bound_U = phase_status.pedtime2next.bound_L;
						}
						else if (phase_status.state == MsgEnum::phaseState::redLight)
						{
							phase_status.pedtime2next.bound_L = phase_status.time2next.bound_L;
							phase_status.pedtime2next.bound_U = phase_status.time2next.bound_U;
						}
						else if ((phase_status.state == MsgEnum::phaseState::permissiveYellow)
							&& ((uint8_t)(i+1) == signal_status_mess.next_phases[ring_phase_on(i+1)]))
						{
							const auto& phasetiming = phaseTimings[i];
							uint32_t red_clearance = ((phaseFlags.red_revert_interval > phasetiming.red_clearance) ?
									phaseFlags.red_revert_interval : phasetiming.red_clearance);
							phase_status.pedtime2next.bound_L = static_cast<uint16_t>(phase_status.time2next.bound_L + red_clearance);
							phase_status.pedtime2next.bound_U = static_cast<uint16_t>(phase_status.time2next.bound_U + red_clearance);
						}
						else
						{
							phase_status.pedtime2next.bound_U = time2start[ring_phase_on(i+1)].bound_U;
							phase_status.pedtime2next.bound_L = time2start[ring_phase_on(i+1)].bound_L;
						}
					}
				}
			}

			/// update controller_status.signal_status
			controller_status.signal_status = signal_status_mess;
			/// update softcall_state with current phase_status
			updateSoftcallState(softcall_state, controller_status.phase_status);
			/// pack massage and send to MRP_DataMgr
			size_t msgSize = packMsg(sendbuf_socket, controller_status, msgUtils::msgid_cntrlstatus, fullTimeStamp);
			bool sendFlag = socketUtils::sendall(sendConn, &sendbuf_socket[0], msgSize);
			if (log_type == logUtils::logType::detailLog)
				logUtils::logMsg(logFiles, std::string("sig"), sendbuf_socket, msgSize);
			if (verbose)
			{
				std::cout << fullTimeStamp.localDateTimeStamp.to_dateTimeStr('-', ':');
				std::cout << ", sending msgid_cntrlstatus, " << std::boolalpha << sendFlag << std::endl;
			}
		}

		/// check sending soft-call
		if ((fullTimeStamp.msec > softcall_state.msec + softcall_interval) && softcall_state.any())
		{
			std::bitset<8> pedCallPhases;
			std::bitset<8> vehCallPahses;
			std::bitset<8> prioCallPhases;
			for (int i = 0; i < 8; i++)
			{
				const auto& phase_status = controller_status.phase_status[i];
				bool phaseInGreen = ((phase_status.state == MsgEnum::phaseState::protectedGreen)
					|| (phase_status.state == MsgEnum::phaseState::permissiveGreen));
				if (softcall_state.ped_call.test(i))
					pedCallPhases.set(i);
				if (softcall_state.veh_call.test(i) && !phaseInGreen)
					vehCallPahses.set(i);
				if (softcall_state.veh_ext.test(i) && phaseInGreen)
					vehCallPahses.set(i);
				if (softcall_state.prio_call.test(i) && !phaseInGreen)
					prioCallPhases.set(i);
				if (softcall_state.prio_ext.test(i) && phaseInGreen)
					prioCallPhases.set(i);
			}
			if (pedCallPhases.any() || vehCallPahses.any() || prioCallPhases.any())
			{
				size_t nbyte = AB3418MSG::packRequest(sendbuf_spat2, controller_status.controller_addr, vehCallPahses, pedCallPhases, prioCallPhases);
				if (send2controller)
				{
					if (write(fd_spat2, (void*)&sendbuf_spat2[0], nbyte) > 0)
					{
						softcall_state.ped_call.reset();
						softcall_state.msec = fullTimeStamp.msec;
						if (verbose)
						{
							std::cout << fullTimeStamp.localDateTimeStamp.to_dateTimeStr('-', ':');
							std::cout << " send soft-call to controller: ped_call=" << pedCallPhases.to_string();
							std::cout << " veh_call=" << vehCallPahses.to_string();
							std::cout << " prio_call=" << prioCallPhases.to_string() << std::endl;
						}
						if (log_type != logUtils::logType::none)
						{
							size_t msgSize = AB3418MSG::packMsg(sendbuf_socket, vehCallPahses, pedCallPhases, prioCallPhases,
								msgUtils::msgid_placecall, fullTimeStamp.localDateTimeStamp.msOfDay);
							logUtils::logMsg(logFiles, std::string("call"), sendbuf_socket, msgSize);
						}
					}
					else
					{
						OS_ERR << fullTimeStamp.localDateTimeStamp.to_dateTimeStr('-', ':');
						OS_ERR << ", failed sending soft-call to controller: ped_call=" << pedCallPhases.to_string();
						OS_ERR << ", veh_call=" << vehCallPahses.to_string();
						OS_ERR << ", prio_call=" << prioCallPhases.to_string() << std::endl;
					}
				}
			}
		}

		/// check reopen log files
		if ((log_type != logUtils::logType::none) && (fullTimeStamp.msec > logfile_msec + logInterval))
		{
			logUtils::reOpenLogFiles(logFiles, fullTimeStamp.localDateTimeStamp.to_fileName());
			logfile_msec = fullTimeStamp.msec;
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(5));
	}
	/// exit
	timeUtils::getFullTimeStamp(fullTimeStamp);
	OS_ERR << fullTimeStamp.localDateTimeStamp.to_dateTimeStr('-', ':');
	OS_ERR << ", received user termination signal " << terminate << ", exit!" << std::endl;
	OS_ERR.close();
	if (log_type != logUtils::logType::none)
		logUtils::closeLogFiles(logFiles);
	pmycnf->disconnectAll();
	close_port(fd_spat, true);
	close_port(fd_spat2, true);
	delete pmycnf;
	delete pcard;
	delete pPolls;
	return(0);
}

int open_port(const std::string& port_name, bool isReadOnly)
{
	struct termios settings;
	speed_t baudrate = B38400;
	int fd;

	/// open port
	if (isReadOnly)
		fd = open(port_name.c_str(), O_RDONLY | O_NOCTTY | O_NDELAY);
	else
		fd = open(port_name.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd == -1)
	{
		std::cerr << "Failed to open serial port " << port_name << std::endl;
		return(-1);
	}
	if (!isatty(fd))
	{
		std::cerr << port_name << " is not a terminal" << std::endl;
		close_port(fd, isReadOnly);
		return(-1);
	}
	/// get serial interface attributes
	std::memset(&settings, 0, sizeof(settings));
	if (tcgetattr(fd,&settings) < 0)
	{
		std::cerr << "Failed tcgetattr on port " << port_name << std::endl;
		close_port(fd, isReadOnly);
		return(-1);
	}
	/// set to raw mode
	cfmakeraw(&settings);
	settings.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
	if (!isReadOnly)
		settings.c_oflag &= ~(OPOST | ONLCR);
	settings.c_lflag &= ~(ICANON | ECHO | ECHONL | ISIG | IEXTEN);
	settings.c_cflag &= ~(CSIZE | PARENB);
	settings.c_cflag |= CS8;
	/// immediate read return
	settings.c_cc[VMIN] = 1;
	settings.c_cc[VTIME] = 0;
	/// input/output baud rate
	if (cfsetspeed(&settings, baudrate) < 0)
	{
		std::cerr << "Failed cfsetspeed on port " << port_name << std::endl;
		close_port(fd, isReadOnly);
		return(-1);
	}
	/// set serial interface attributes
	if (isReadOnly)
		tcflush(fd, TCIFLUSH);
	else
		tcflush(fd, TCIOFLUSH);
	if (tcsetattr(fd, TCSANOW, &settings) < 0)
	{
		std::cerr << "Failed tcsetattr on port " << port_name << std::endl;
		close_port(fd, isReadOnly);
		return(-1);
	}
	return(fd);
}

void close_port(int fd, bool isReadOnly)
{
	if (isReadOnly)
		tcflush(fd, TCIFLUSH);
	else
		tcflush(fd, TCIOFLUSH);
	close(fd);
}

void processRecvBuff(std::vector<uint8_t>& buf, size_t& count, std::vector< std::pair<size_t, size_t> >& vec)
{	/// serial port receiving buffer
	vec.clear();
	std::vector<size_t> flagloc;
	for (size_t i = 0; i < count; i++)
	{
		if (buf[i] == AB3418MSG::flag)
			flagloc.push_back(i);
	}
	if (flagloc.empty())
	{ // no 0x7E found, discard bytes in buffer
		count = 0;
		return;
	}

	size_t flagnums = flagloc.size();
	if (flagnums == 1)
	{ // found 1 0x7E, move 0x7E to the beginning of buffer
		if (flagloc[0] > 0)
		{
			count = 0;
			for (size_t i = flagloc[0]; i < count; i++)
				buf[count++] = buf[i];
		}
		return;
	}
	if (flagnums == 2)
	{	// found 2 0x7E. two cases: ...0x7E 0x7E ... and ...0x7E...0x7E
		if (flagloc[1] == flagloc[0] + 1)
		{	// case 0x7E 0x7E, no completed ab3418 frame found, move 2nd 0x7E to the beginning of buffer
			count = 0;
			for (size_t i = flagloc[1]; i < count; i++)
				buf[count++] = buf[i];
		}
		else // case 0x7E...0x7E, found one completed ab3418 frame
			vec.push_back(std::make_pair(flagloc[0], flagloc[1]));
		return;
	}
	// found at least 3 0x7E, there must be a pair of consecutive 0x7Es
	bool foundpair = false;
	size_t pairIndex;
	for (size_t i = 0; i < flagnums - 1; i++)
	{ // find the first occurrence of consecutive 0x7E
		if (flagloc[i+1] == flagloc[i] + 1)
		{
			foundpair = true;
			pairIndex = i;  // this is the end of a completed ab3418 frame
			break;
		}
	}
	if(!foundpair)
	{	// error in received buff, move the last 0x7E to the beginning of buffer
		count = 0;
		for (size_t i = flagloc.back(); i < count; i++)
			buf[count++] = buf[i];
		return;
	}
	if (pairIndex > 0)
	{	// add start and end indexes of the previous completed ab3418 frame to vec
		vec.push_back(std::make_pair(flagloc[pairIndex - 1], flagloc[pairIndex]));
	}
	while(1)
	{ // add start and end indexes of the rest completed ab3418 frame to vec
		pairIndex += 2;
		if (pairIndex < flagnums)
			vec.push_back(std::make_pair(flagloc[pairIndex - 1], flagloc[pairIndex]));
		else
			break;
	}
}

uint16_t getPedIntervalLeft(uint8_t interval_timer, unsigned long long timer_time, unsigned long long msec)
{
	uint16_t timeinto = static_cast<uint16_t>((msec + 30 - timer_time) / 100);
	uint16_t timecountdown = static_cast<uint16_t>(interval_timer * 10);
	return(static_cast<uint16_t>((timecountdown > timeinto) ? (timecountdown - timeinto) : timecountdown));
}

uint8_t getPhaseWalkInterval(const Card::phasetiming_mess_t& phasetiming, const Card::phaseflags_mess_t& phaseflags, uint8_t phaseIdx)
{
	return(static_cast<uint8_t>((phaseflags.walk2_phases.test(phaseIdx)) ? phasetiming.walk2_interval : phasetiming.walk1_interval));
}

uint32_t getPhaseGreenLeft(uint8_t active_interval, uint8_t interval_timer, uint32_t intervalTimeInto, const Card::phasetiming_mess_t& phasetiming)
{
	uint32_t countdownTime = interval_timer * 10;
	if (active_interval == 0x00)  /// walk
		return(((countdownTime > intervalTimeInto) ? (countdownTime - intervalTimeInto) : countdownTime) + phasetiming.walk_clearance * 10);
	else if (active_interval < 0x05) /// walk clearance or minimum green or added initial
		return((countdownTime > intervalTimeInto) ? (countdownTime - intervalTimeInto) : countdownTime);
	else /// passage, map gap, min gap
		return(interval_timer);
}

uint32_t getPhaseGreen2Maxout(const Card::phasetiming_mess_t& phasetiming, const Card::phaseflags_mess_t& phaseflags, uint8_t phaseIdx)
{
	uint32_t maximum_extension = phasetiming.maximum_extensions[0];
	if (phaseflags.maxgreen2_phases.test(phaseIdx))
		maximum_extension = phasetiming.maximum_extensions[1];
	else if (phaseflags.maxgreen3_phases.test(phaseIdx))
		maximum_extension = phasetiming.maximum_extensions[2];
	return((phasetiming.minimum_green + maximum_extension) * 10);
}

uint32_t getPhaseGuaranteedGreen(const Card::phasetiming_mess_t& phasetiming, const Card::phaseflags_mess_t& phaseflags,
	uint8_t phaseIdx, uint8_t active_interval)
{ /// for active phase
	if (active_interval <= 0x01)
		return((getPhaseWalkInterval(phasetiming, phaseflags, phaseIdx) + phasetiming.walk_clearance) * 10);
	return((active_interval < 0x05) ? phasetiming.minimum_green * 10 : 0);
}

uint32_t getPhaseGuaranteedGreen(const Card::phasetiming_mess_t& phasetiming, const Card::phaseflags_mess_t& phaseflags,
	uint8_t phaseIdx, const phase_status_t& phase_status)
{ /// for phase is not active and not with continuous recall
	if ((phase_status.recall_status == MsgEnum::phaseRecallType::ped) || (phase_status.call_status == MsgEnum::phaseCallType::ped))
		return((getPhaseWalkInterval(phasetiming, phaseflags, phaseIdx) + phasetiming.walk_clearance) * 10);
	else
		return(phasetiming.minimum_green * 10);
}

void barrierCrossAdjust(predicted_bound_t (&time2start)[2])
{
	time2start[0].bound_L = static_cast<uint16_t>((time2start[0].bound_L < time2start[1].bound_L) ? time2start[1].bound_L : time2start[0].bound_L);
	time2start[1].bound_L = time2start[0].bound_L;
	time2start[0].bound_U = static_cast<uint16_t>((time2start[0].bound_U < time2start[1].bound_U) ? time2start[1].bound_U : time2start[0].bound_U);
	time2start[1].bound_U = time2start[0].bound_U;
}

void updateActivePhaseTime2next(phase_status_t& phase_status, predicted_bound_t& time2start,
	const AB3418MSG::signal_status_mess_t& signalstatus, const Card::phasetiming_mess_t& phasetiming,
	const Card::phaseflags_mess_t& phaseflags, uint8_t ring, unsigned long long timer_time, unsigned long long msec)
{ /// running free, no force-off constraint
	uint32_t intervalTimeInto = static_cast<uint32_t>((msec + 30 - timer_time) / 100);
	if ((phase_status.state == MsgEnum::phaseState::protectedGreen) || (phase_status.state == MsgEnum::phaseState::permissiveGreen))
	{
		uint32_t stateTimeInto = static_cast<uint32_t>((msec + 30 - phase_status.state_start_time) / 100);
		/// green left based on active_interval
		uint32_t timeleft = getPhaseGreenLeft(signalstatus.active_interval[ring], signalstatus.interval_timer[ring], intervalTimeInto, phasetiming);
		/// time2maxout (till max-out point)
		uint32_t maxgreen = getPhaseGreen2Maxout(phasetiming, phaseflags, (uint8_t)(signalstatus.active_phases[ring] - 1));
		uint32_t time2maxout = (maxgreen > stateTimeInto) ? (maxgreen - stateTimeInto) : 0;
		/// time2gapout (till end of the guaranteed green where the phase could be gapped-out)
		uint32_t guaranteedgreen = ((phase_status.recall_status == MsgEnum::phaseRecallType::maximum) ? maxgreen :
			getPhaseGuaranteedGreen(phasetiming, phaseflags, (uint8_t)(signalstatus.active_phases[ring] - 1), signalstatus.active_interval[ring]));
		uint32_t time2gapout = (guaranteedgreen > stateTimeInto) ? (guaranteedgreen - stateTimeInto) : 0;
		/// minimum green is guaranteed
		if (time2maxout < time2gapout)
			time2maxout = time2gapout;
		/// time2next
		phase_status.time2next.bound_L = static_cast<uint16_t>((time2gapout == 0) ? timeleft : time2gapout);
		phase_status.time2next.bound_U = static_cast<uint16_t>((time2maxout == 0) ? timeleft : time2maxout);
		/// time2start for phases after (next_phase not known yet)
		time2start.bound_L = static_cast<uint16_t>(phase_status.time2next.bound_L + phasetiming.yellow_interval + phasetiming.red_clearance);
		time2start.bound_U = static_cast<uint16_t>(phase_status.time2next.bound_U + phasetiming.yellow_interval + phasetiming.red_clearance);
	}
	else if (phase_status.state == MsgEnum::phaseState::protectedYellow)
	{	/// yellow interval is fixed
		uint32_t timeleft = signalstatus.interval_timer[ring];
		phase_status.time2next.bound_L = static_cast<uint16_t>((timeleft >= intervalTimeInto) ? (timeleft - intervalTimeInto) : timeleft);
		phase_status.time2next.bound_U = phase_status.time2next.bound_L;
		if (signalstatus.next_phases[ring] == signalstatus.active_phases[ring])
		{
			time2start.bound_L = static_cast<uint16_t>(phase_status.time2next.bound_L +
				((phaseflags.red_revert_interval > phasetiming.red_clearance) ? phaseflags.red_revert_interval : phasetiming.red_clearance));
		}
		else
			time2start.bound_L = static_cast<uint16_t>(phase_status.time2next.bound_L + phasetiming.red_clearance);
		time2start.bound_U = time2start.bound_L;
	}
	else
	{	/// red clearance or red revert intervals are fixed
		uint32_t timeleft = signalstatus.interval_timer[ring];
		phase_status.time2next.bound_L = static_cast<uint16_t>((timeleft >= intervalTimeInto) ? (timeleft - intervalTimeInto) : timeleft);
		phase_status.time2next.bound_U = phase_status.time2next.bound_U;
		/// time2start for the next_phase (next_phase already known)
		time2start.bound_L = phase_status.time2next.bound_L;
		time2start.bound_U = time2start.bound_L;
	}
}

void getNextPhaseStartBound(predicted_bound_t& time2start, const Card::phasetiming_mess_t& phasetiming,
	const Card::phaseflags_mess_t& phaseflags, uint8_t phaseIdx, const phase_status_t& phase_status)
{ /// running free, for phase is not active
	uint32_t maxgreen = getPhaseGreen2Maxout(phasetiming, phaseflags, phaseIdx);
	uint32_t guaranteedgreen = ((phase_status.recall_status == MsgEnum::phaseRecallType::maximum) ?
			maxgreen : getPhaseGuaranteedGreen(phasetiming, phaseflags, phaseIdx, phase_status));
	if (maxgreen < guaranteedgreen)
		maxgreen = guaranteedgreen;
	if ((phase_status.recall_status != MsgEnum::phaseRecallType::none) || (phase_status.call_status != MsgEnum::phaseCallType::none))
	{	/// phase should be on, move back time2start.bound_L
		time2start.bound_L = static_cast<uint16_t>(time2start.bound_L + guaranteedgreen + phasetiming.yellow_interval + phasetiming.red_clearance);
	}
	time2start.bound_U = static_cast<uint16_t>(time2start.bound_U + maxgreen + phasetiming.yellow_interval + phasetiming.red_clearance);
}

uint32_t getTime2Forceoff(uint8_t force_off, uint16_t local_cycle_clock, uint16_t cycle_length, bool is_sync_phase)
{	/// sync phase that terminates at yield point actually terminates at the offset of a second
	uint32_t forceoff = (is_sync_phase && (force_off == 0)) ? 10 : force_off * 10;
	return ((is_sync_phase) ? ((forceoff > local_cycle_clock) ? (forceoff - local_cycle_clock) : (forceoff + cycle_length - local_cycle_clock))
		: ((forceoff > local_cycle_clock) ? (forceoff - local_cycle_clock) : 0));
}

bool isPhaseForceoffOnly(Card::ConcurrentType type, bool is_sync_phase, bool is_lag_phase)
{
	return((type == Card::ConcurrentType::majorMajor) ||
		((type == Card::ConcurrentType::minorMajor) && (is_sync_phase || is_lag_phase)));
}

bool isPhaseForceoffOnly(bool is_sync_phase, bool is_leadlag_mode, bool is_minor_lagphase)
{
	return(is_sync_phase || (is_leadlag_mode && is_minor_lagphase));
}

uint32_t getTime2GreenEnd(uint32_t time2maxout, uint32_t time2forceoff, bool is_forceoff_Only)
{	/// green end at either max-out point or force-off point, whichever comes first
	return((is_forceoff_Only) ? time2forceoff : ((time2maxout > time2forceoff) ? time2forceoff : time2maxout));
}

void updateActivePhaseTime2next(phase_status_t& phase_status, predicted_bound_t& time2start, const Card::coordplan_mess_t& coordplan,
	const AB3418MSG::signal_status_mess_t& signalstatus, const Card::phasetiming_mess_t& phasetiming, const Card::phaseflags_mess_t& phaseflags,
	uint8_t ring, uint16_t local_cycle_clock, uint16_t cycle_length, Card::ConcurrentType concurrentType,
	unsigned long long timer_time, unsigned long long msec)
{ /// under coordination, needs to consider force-off constraint
	uint32_t intervalTimeInto = static_cast<uint32_t>((msec + 30 - timer_time)/100);
	if ((phase_status.state == MsgEnum::phaseState::protectedGreen) || (phase_status.state == MsgEnum::phaseState::permissiveGreen))
	{	/// stateTimeInto & intervalTimeInto
		uint32_t stateTimeInto = static_cast<uint32_t>((msec + 30 - phase_status.state_start_time)/100);
		/// green left based on active_interval
		uint32_t timeleft = getPhaseGreenLeft(signalstatus.active_interval[ring], signalstatus.interval_timer[ring], intervalTimeInto, phasetiming);
		/// time2maxout (till max-out point)
		uint32_t maxgreen = getPhaseGreen2Maxout(phasetiming, phaseflags, (uint8_t)(signalstatus.active_phases[ring]-1));
		uint32_t time2maxout = (maxgreen > stateTimeInto) ? (maxgreen - stateTimeInto) : 0;
		/// time2forceoff (till force-off point)
		uint32_t time2forceoff = getTime2Forceoff(signalstatus.active_force_off[ring], local_cycle_clock, cycle_length,
			(signalstatus.active_phases[ring] == coordplan.coordinated_phases[ring]));
		/// time2terminate (max-out or force-off whichever comes first)
		bool terminateByforceoffOnly = isPhaseForceoffOnly(concurrentType, (signalstatus.active_phases[ring] == coordplan.coordinated_phases[ring]),
			coordplan.lag_phases.test(signalstatus.active_phases[ring]-1));
		uint32_t time2terminate = getTime2GreenEnd(time2maxout, time2forceoff, terminateByforceoffOnly);
		/// time2gapout (till the end of the guaranteed green)
		uint32_t guaranteedgreen = ((phase_status.recall_status == MsgEnum::phaseRecallType::maximum) ? time2terminate :
			getPhaseGuaranteedGreen(phasetiming, phaseflags, (uint8_t)(signalstatus.active_phases[ring]-1), signalstatus.active_interval[ring]));
		uint32_t time2gapout = (guaranteedgreen > stateTimeInto) ? (guaranteedgreen - stateTimeInto) : 0;
		/// minimum green is guaranteed
		if (time2terminate < time2gapout)
			time2terminate = time2gapout;
		/// time2next
		phase_status.time2next.bound_U = static_cast<uint16_t>((time2terminate == 0) ? timeleft : time2terminate);
		if (terminateByforceoffOnly)
			phase_status.time2next.bound_L = phase_status.time2next.bound_U;
		else
			phase_status.time2next.bound_L = static_cast<uint16_t>((time2gapout == 0) ? timeleft : time2gapout);
		/// time2start for the next_phase (next_phase not known yet)
		time2start.bound_L = static_cast<uint16_t>(phase_status.time2next.bound_L + phasetiming.yellow_interval + phasetiming.red_clearance);
		time2start.bound_U = static_cast<uint16_t>(phase_status.time2next.bound_U + phasetiming.yellow_interval + phasetiming.red_clearance);
	}
	else if (phase_status.state == MsgEnum::phaseState::protectedYellow)
	{ /// yellow interval is fixed
		uint32_t timeleft = signalstatus.interval_timer[ring];
		phase_status.time2next.bound_L = static_cast<uint16_t>((timeleft >= intervalTimeInto) ? (timeleft - intervalTimeInto) : timeleft);
		phase_status.time2next.bound_U = phase_status.time2next.bound_L;
		if (signalstatus.next_phases[ring] == signalstatus.active_phases[ring])
		{
			time2start.bound_L = static_cast<uint16_t>(phase_status.time2next.bound_L +
				((phaseflags.red_revert_interval > phasetiming.red_clearance) ? phaseflags.red_revert_interval : phasetiming.red_clearance));
		}
		else
			time2start.bound_L = static_cast<uint16_t>(phase_status.time2next.bound_L + phasetiming.red_clearance);
		time2start.bound_U = time2start.bound_L;
	}
	else
	{ /// red clearance or red revert intervals are fixed
		uint32_t timeleft = signalstatus.interval_timer[ring];
		phase_status.time2next.bound_L = static_cast<uint16_t>((timeleft >= intervalTimeInto) ? (timeleft - intervalTimeInto) : timeleft);
		phase_status.time2next.bound_U = phase_status.time2next.bound_U;
		/// time2start for the next_phase (next_phase already known)
		time2start.bound_L = phase_status.time2next.bound_L;
		time2start.bound_U = time2start.bound_L;
	}
}

void getNextPhaseStartBound(predicted_bound_t& time2start, const Card::phasetiming_mess_t& phasetiming,
	const Card::phaseflags_mess_t& phaseflags, uint8_t phaseIdx, const phase_status_t& phase_status,
	const Card::coordplan_mess_t& coordplan, uint16_t local_cycle_clock, uint16_t cycle_length)
{ /// under coordination, for phase is not active
	uint32_t maxgreen = getPhaseGreen2Maxout(phasetiming, phaseflags, phaseIdx);
	bool terminateByforceoffOnly = isPhaseForceoffOnly(coordplan.sync_phases.test(phaseIdx),
		((coordplan.leadLagMode == Card::LeadLagType::leadLag) || (coordplan.leadLagMode == Card::LeadLagType::lagLead)),
		(coordplan.leadlag_phases[coordplan.sync_barrier][coordplan.sync_ring][1] == (uint8_t)(phaseIdx+1)));
	/// when phase green starts at time2start.bound_L
	uint16_t start_local_cycle_clock_L = static_cast<uint16_t>(time2start.bound_L + local_cycle_clock);
	if (start_local_cycle_clock_L > cycle_length)
		start_local_cycle_clock_L = static_cast<uint16_t>(start_local_cycle_clock_L - cycle_length);
	uint32_t time2forceoff_L = getTime2Forceoff(coordplan.force_off[phaseIdx], start_local_cycle_clock_L,
			cycle_length, coordplan.sync_phases.test(phaseIdx));
	uint32_t time2terminate_L = getTime2GreenEnd(maxgreen, time2forceoff_L, terminateByforceoffOnly);
	uint32_t guaranteedgreen_L = ((phase_status.recall_status == MsgEnum::phaseRecallType::maximum) ? time2terminate_L
			: getPhaseGuaranteedGreen(phasetiming, phaseflags, phaseIdx, phase_status));
	if (time2terminate_L < guaranteedgreen_L)
		time2terminate_L = guaranteedgreen_L;
	/// when phase green starts at time2start.bound_U
	uint16_t start_local_cycle_clock_U = static_cast<uint16_t>(time2start.bound_U + local_cycle_clock);
	if (start_local_cycle_clock_U > cycle_length)
		start_local_cycle_clock_U = static_cast<uint16_t>(start_local_cycle_clock_U - cycle_length);
	uint32_t time2forceoff_U = getTime2Forceoff(coordplan.force_off[phaseIdx], start_local_cycle_clock_U,
			cycle_length, coordplan.sync_phases.test(phaseIdx));
	uint32_t time2terminate_U = getTime2GreenEnd(maxgreen, time2forceoff_U, terminateByforceoffOnly);
	uint32_t guaranteedgreen_U = ((phase_status.recall_status == MsgEnum::phaseRecallType::maximum) ? time2terminate_U :
			getPhaseGuaranteedGreen(phasetiming, phaseflags, phaseIdx, phase_status));
	if (time2terminate_U < guaranteedgreen_U)
		time2terminate_U = guaranteedgreen_U;
	if (terminateByforceoffOnly)
	{
		time2start.bound_L = static_cast<uint16_t>(time2start.bound_L + time2terminate_L + phasetiming.yellow_interval + phasetiming.red_clearance);
		time2start.bound_U = static_cast<uint16_t>(time2start.bound_U + time2terminate_U + phasetiming.yellow_interval + phasetiming.red_clearance);
	}
	else
	{
		time2start.bound_U = static_cast<uint16_t>(time2start.bound_U + time2terminate_U + phasetiming.yellow_interval + phasetiming.red_clearance);
		if ((phase_status.recall_status != MsgEnum::phaseRecallType::none) || (phase_status.call_status != MsgEnum::phaseCallType::none))
			time2start.bound_L = static_cast<uint16_t>(time2start.bound_L + guaranteedgreen_L + phasetiming.yellow_interval + phasetiming.red_clearance);
	}
}

void softcallPhaseReset(std::bitset<8>& callPhases, const std::bitset<8>& resetPhases)
{
	for (int i = 0; i < 8; i++)
	{
		if (resetPhases.test(i) && callPhases.test(i))
			callPhases.reset(i);
	}
}

void softcallPhaseSet(std::bitset<8>& callPhases, const std::bitset<8>& setPhases)
{
	for (int i = 0; i < 8; i++)
	{
		if (setPhases.test(i) && !callPhases.test(i))
			callPhases.set(i);
	}
}

void updateSoftcallState(softcall_state_t& softcallState, const msgDefs::softcall_request_t& request)
{
	switch(request.calltype)
	{
	case MsgEnum::softCallType::cancel:
		if (request.callobj == MsgEnum::softCallObj::vehicle)
			softcallPhaseReset(softcallState.veh_ext, request.callphase);
		else if (request.callobj == MsgEnum::softCallObj::priority)
		{
			softcallPhaseReset(softcallState.prio_call, request.callphase);
			softcallPhaseReset(softcallState.prio_ext, request.callphase);
		}
		break;
	case MsgEnum::softCallType::call:
		if (request.callobj == MsgEnum::softCallObj::ped)
			softcallPhaseSet(softcallState.ped_call, request.callphase);
		else if (request.callobj == MsgEnum::softCallObj::vehicle)
			softcallPhaseSet(softcallState.veh_call, request.callphase);
		else if (request.callobj == MsgEnum::softCallObj::priority)
			softcallPhaseSet(softcallState.prio_call, request.callphase);
		break;
	case MsgEnum::softCallType::extension:
		if (request.callobj == MsgEnum::softCallObj::vehicle)
			softcallPhaseSet(softcallState.veh_ext, request.callphase);
		else if (request.callobj == MsgEnum::softCallObj::priority)
			softcallPhaseSet(softcallState.prio_ext, request.callphase);
		break;
	default:
		break;
	}
}

void updateSoftcallState(softcall_state_t& softcallState, const phase_status_t (&status)[8])
{
	for (int i = 0; i < 8; i++)
	{
		bool phaseInGreen = ((status[i].state == MsgEnum::phaseState::protectedGreen)
			|| (status[i].state == MsgEnum::phaseState::permissiveGreen));
		if (softcallState.veh_call.test(i) && phaseInGreen)
			softcallState.veh_call.reset(i);
		if (softcallState.veh_ext.test(i) && !phaseInGreen)
			softcallState.veh_ext.reset(i);
		if (softcallState.prio_call.test(i) && phaseInGreen)
			softcallState.prio_call.reset(i);
		if (softcallState.prio_ext.test(i) && !phaseInGreen)
			softcallState.prio_ext.reset(i);
	}
}

size_t packMsg(std::vector<uint8_t>& buf, const controller_status_t& cntrstatus, uint8_t msgid, const timeUtils::fullTimeStamp_t& fullTimeStamp)
{
	static uint8_t msgCnt = 0;
	/// msgCnt
	msgCnt = (uint8_t)((msgCnt + 1) % 127);
	/// status byte of traffic signal controller
	std::bitset<16> status;
	if ((cntrstatus.signal_status.active_interval[0] == 0x0A) || (cntrstatus.signal_status.active_interval[1] == 0x0A))
		status.set(1);  /// stopTimeIsActivated
	if (cntrstatus.mode == MsgEnum::controlMode::flashing)
		status.set(2);
	if (cntrstatus.mode == MsgEnum::controlMode::preemption)
		status.set(3);
	if(cntrstatus.signal_status.preempt.test(7))
		status.set(4);  /// signalPriorityIsActive
	status.set(6);   /// trafficDependentOperation
	/// byte 0 to 8 - MMITSS UDP header
	size_t offset = 9;
	/// pack SPaT data element
	buf[offset++] = msgCnt;
	msgUtils::pack4bytes(buf, offset, fullTimeStamp.utcDateTimeStamp.minuteOfYear);
	msgUtils::pack2bytes(buf, offset, fullTimeStamp.utcDateTimeStamp.msOfMinute);
	buf[offset++] = (uint8_t)cntrstatus.permitted_phases.to_ulong();
	buf[offset++] = (uint8_t)cntrstatus.permitted_ped_phases.to_ulong();
	msgUtils::pack2bytes(buf, offset, (uint16_t)status.to_ulong());
	for (int i = 0; i < 8; i++)
	{
		if (cntrstatus.permitted_phases.test(i))
		{
			const auto& phaseStatus = cntrstatus.phase_status[i];
			buf[offset++] = static_cast<uint8_t>(phaseStatus.state);
			uint16_t startTime, minEndTime, maxEndTime;
			if (phaseStatus.state == MsgEnum::phaseState::flashingRed)
			{
				startTime = 0;
				minEndTime = 0;
				maxEndTime = 0;
			}
			else
			{
				startTime = (uint16_t)round((double)(phaseStatus.state_start_time % 3600000) / 100);
				minEndTime = (uint16_t)round((double)((fullTimeStamp.msec + phaseStatus.time2next.bound_L * 100) % 3600000) / 100);
				maxEndTime = (uint16_t)round((double)((fullTimeStamp.msec + phaseStatus.time2next.bound_U * 100) % 3600000) / 100);
			}
			msgUtils::pack2bytes(buf, offset, startTime);
			msgUtils::pack2bytes(buf, offset, minEndTime);
			msgUtils::pack2bytes(buf, offset, maxEndTime);
		}
	}
	for (int i = 0; i < 8; i++)
	{
		if (!cntrstatus.permitted_ped_phases.test(i))
			continue;
		const auto& phaseStatus = cntrstatus.phase_status[i];
		buf[offset++] = static_cast<uint8_t>(phaseStatus.pedstate);
		uint16_t startTime, minEndTime, maxEndTime;
		if (phaseStatus.pedstate == MsgEnum::phaseState::flashingRed)
		{
			startTime = 0;
			minEndTime = 0;
			maxEndTime = 0;
		}
		else
		{
			startTime = (uint16_t)round((double)(phaseStatus.pedstate_start_time % 3600000) / 100);
			minEndTime = (uint16_t)round((double)((fullTimeStamp.msec + phaseStatus.pedtime2next.bound_L * 100) % 3600000) / 100);
			maxEndTime = (uint16_t)round((double)((fullTimeStamp.msec + phaseStatus.pedtime2next.bound_U * 100) % 3600000) / 100);
		}
		msgUtils::pack2bytes(buf, offset, startTime);
		msgUtils::pack2bytes(buf, offset, minEndTime);
		msgUtils::pack2bytes(buf, offset, maxEndTime);
	}
	buf[offset++] = static_cast<uint8_t>(cntrstatus.mode);
	buf[offset++] = cntrstatus.signal_status.pattern_num;
	buf[offset++] = cntrstatus.synch_phase;
	msgUtils::pack2bytes(buf, offset, cntrstatus.cycle_length);
	msgUtils::pack2bytes(buf, offset, cntrstatus.cur_local_cycle_clock);
	buf[offset++] = (uint8_t)cntrstatus.coordinated_phases.to_ulong();
	buf[offset++] = (uint8_t)cntrstatus.signal_status.preempt.to_ulong();
	buf[offset++] = (uint8_t)cntrstatus.signal_status.ped_call.to_ulong();
	buf[offset++] = (uint8_t)cntrstatus.signal_status.veh_call.to_ulong();
	for (int i = 0; i < 8; i++)
	{
		const auto& phaseStatus = cntrstatus.phase_status[i];
		buf[offset++] = static_cast<uint8_t>(phaseStatus.call_status);
		buf[offset++] = static_cast<uint8_t>(phaseStatus.recall_status);
	}
	/// add MMITSS header
	size_t header_offset = 0;
	msgUtils::packHeader(buf, header_offset, msgid, fullTimeStamp.localDateTimeStamp.msOfDay, (uint16_t)(offset - 9));
	return(offset);
}
