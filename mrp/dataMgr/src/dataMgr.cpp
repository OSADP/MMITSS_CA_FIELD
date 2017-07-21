//********************************************************************************************************
//
// © 2016 Regents of the University of California on behalf of the University of California at Berkeley
//       with rights granted for USDOT OSADP distribution with the ECL-2.0 open source license.
//
//*********************************************************************************************************
/* dataMgr.cpp - MRP_DataMgr
 * functions:
 * 1. send MAP to RSE_MessageTX and the pedestrian cloud server
 * 2. receive BSMs and SRMs from RSE_MessageRX and forward the messages to MRP_Aware
 * 3. receive PSRMs from the pedestrian cloud server and forward the messages to MRP_Aware
 * 4. receive UDP messages from MRP_TCI
 *    msgid_cntrlstatus   - encode SPaT, send SPaT to RSE_MessageTX and the pedestrian cloud server
 *                        - forward msgid_cntrlstatus message to MRP_Aware
 *    msgid_detCnt        - log to file, and save in memory for responding to poll request
 *    msgid_detPres       - log to file, and save in memory for responding to poll request
 * 5. receive UDP messages from MRP_Aware
 *    msgid_ssm           - forward to RSE_MessageTX (SSM is encoded by MRP_Aware)
 *    msgid_softcall      - forward to MRP_TCI
 *    msgid_traj          - log to file, and save in memory for calculating performance measures
 * 6. calculate performance measures, and send msgid_perm message to MRP_Aware
 * 7. receive poll request (msgid_pollReq) and send poll response back.
 *    Other MMITSS components can poll data regarding the traffic controller's timing card parameters,
 *    current control mode, performance measures, etc. The type of requested data is identified by
 *    requested_msgid byte inside the poll request message.
 *    Controller's timing card is populated by MRP_TCI and saved in text file.
 * Structures of UDP messages are defined in msgDefs.h. For potability all UDP messages are serialized.
 * Functions to pack and unpack of UDP messages are defined in msgUtils.h and implemented in msgUtils.cpp
 * logs:
 * 1. simpleLog
 *    - received BSM (WSM_PSID_BSM), SRM (WSM_PSID_SRM), PSRM (msgid_psrm)
 *    - received soft-call request (msgid_softcall)
 *    - received vehicle trajectory (msgid_traj)
 * 2. detailLog - simpleLog plus
 *    - SSM (WSM_PSID_SSM) and SPaT (WSM_PSID_SPAT) sent to RSE_MessageTX
 *    - detector count/occupancy (msgid_detCnt)
 *    - detector presence (msgid_detPres)
 *    - traffic controller and signal status (msgid_cntrlstatus)
 * MAP data is static therefor it is not logged.
 *
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstddef>
#include <fstream>
#include <iostream>
#include <numeric>
#include <poll.h>
#include <string>
#include <sys/stat.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <thread>
#include <unistd.h>
#include <cstring>   /// std::memcpy

#include "AsnJ2735Lib.h"
#include "locAware.h"
#include "cnfUtils.h"
#include "logUtils.h"
#include "msgUtils.h"
#include "socketUtils.h"
#include "timeUtils.h"
#include "timeCard.h"
#include "dsrcConsts.h"
#include "dataMgr.h"

void do_usage(const char* progname)
{
	std::cerr << "Usage" << progname << std::endl;
	std::cerr << "\t-n intersection name" << std::endl;
	std::cerr << "\t-s full path to dataMgr.conf" << std::endl;
	std::cerr << "\t-v turn on verbose" << std::endl;
	std::cerr << "\t-? print this message" << std::endl;
	exit(EXIT_FAILURE);
}

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

	/// instance class ComponentCnf to read configuration file
	ComponentCnf* pmycnf = new ComponentCnf(cnfFile);
	if (!pmycnf->isInitiated())
	{
		std::cerr << fullTimeStamp.localDateTimeStamp.to_dateTimeStr('-', ':');
		std::cerr << ", failed initiating ComponentCnf " << cnfFile << std::endl;
		delete pmycnf;
		return(-1);
	}
	unsigned long long logInterval = pmycnf->getIntegerParaValue(std::string("logInterval")) * 60 * 1000;   // in milliseconds
	unsigned long long permInterval = pmycnf->getIntegerParaValue(std::string("permInterval")) * 60 * 1000; // in milliseconds
	int logType = pmycnf->getIntegerParaValue(std::string("logType"));
	logUtils::logType log_type = ((logInterval == 0) || ((logType != 1) && (logType != 2)))
		? logUtils::logType::none : static_cast<logUtils::logType>(logType);
	std::string fnmap = pmycnf->getStringParaValue(std::string("nmapFile"));
	std::string logPath = pmycnf->getStringParaValue(std::string("logPath"));
	std::string cardName  = pmycnf->getStringParaValue(std::string("timeCardPath"))
		+ std::string("/") + intersectionName + std::string(".timecard");
	unsigned long long perm_msec = 0;
	unsigned long long logfile_msec = 0;

	/// open error log
	std::ofstream OS_ERR(logPath + std::string("/mgr.err"), std::ofstream::app);
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
		logtypes.push_back(std::string("sig"));      // log controller status message received from MRP_TCI
		logtypes.push_back(std::string("cnt"));      // log count/volume message received from MRP_TCI
		logtypes.push_back(std::string("pres"));     // log detector presence message received from MRP_TCI
		logtypes.push_back(std::string("req"));      // log soft-call request message received from MRP_Aware
		logtypes.push_back(std::string("traj"));     // log vehicle trajectory message received from MRP_Aware
		logtypes.push_back(std::string("perm"));     // log performance measures
		logtypes.push_back(std::string("payload"));  // log WSM payload received from
		std::string prefix = logPath + std::string("/") + intersectionName;
		for (auto& type : logtypes)
			{logFiles.push_back(logUtils::Logfile_t(prefix, type));}
		if (!openLogFiles(logFiles, fullTimeStamp.localDateTimeStamp.to_fileName()))
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

	/// instance class LocAware
	LocAware* plocAwareLib = new LocAware(fnmap);
	uint16_t intersectionId = plocAwareLib->getIntersectionIdByName(intersectionName);
	if (intersectionId == 0)
	{
		OS_ERR << fullTimeStamp.localDateTimeStamp.to_dateTimeStr('-', ':');
		OS_ERR << ", failed initiating locAwareLib " << fnmap << "(" << intersectionName << ")" << std::endl;
		OS_ERR.close();
		if (log_type != logUtils::logType::none)
			logUtils::closeLogFiles(logFiles);
		delete pmycnf;
		delete plocAwareLib;
		return(-1);
	}
	std::vector<uint8_t> mapPayload = plocAwareLib->getMapdataPayload(intersectionId);
	/// get speed limits (used for calculating performance measures)
	std::vector<uint8_t> speedLimits(8, 0); // in mph
	plocAwareLib->getSpeedLimits(speedLimits, intersectionId);
	delete plocAwareLib;

	/// form message buffer for sending MAP to RSE_MessageTX and Savari pedestrian cloud server
	const size_t bufSize = 2000;
	std::vector<uint8_t> map2RSE(bufSize, 0);
	std::vector<uint8_t> map2cloud(bufSize, 0);
	size_t map2RSE_size = packMapMsg(map2RSE, mapPayload, msgUtils::msgid_map);
	size_t map2cloud_size = packMapMsg(map2cloud, mapPayload, intersectionId, msgUtils::savari_cloud_map);
	map2RSE.resize(map2RSE_size);
	map2RSE.shrink_to_fit();
	map2cloud.resize(map2cloud_size);
	map2cloud.shrink_to_fit();
	unsigned long long sentMap_msec = 0;

	/// wait until time-card been populated by MRP_TCI
	if(verbose)
	{
		std::cout << fullTimeStamp.localDateTimeStamp.to_dateTimeStr('-', ':');
		std::cout << ", wait for timing card becoming ready" << std::endl;
	}
	std::ifstream IS_CARD(cardName);
	int readCnt = 0;
	bool card_exist = true;
	while (!IS_CARD.is_open())
	{
		std::this_thread::sleep_for(std::chrono::seconds(10));
		if (++readCnt > 30)
		{ /// MRP_TCI polling should be finished within 5 minutes
			card_exist = false;
			break;
		}
		IS_CARD.open(cardName);
	}
	timeUtils::getFullTimeStamp(fullTimeStamp);
	if (!card_exist)
	{
		OS_ERR << fullTimeStamp.localDateTimeStamp.to_dateTimeStr('-', ':');
		OS_ERR << ", timing card not exist" << std::endl;
		OS_ERR.close();
		if (log_type != logUtils::logType::none)
			logUtils::closeLogFiles(logFiles);
		delete pmycnf;
		return(-1);
	}
	IS_CARD.close();
	if(verbose)
	{
		std::cout << fullTimeStamp.localDateTimeStamp.to_dateTimeStr('-', ':');
		std::cout << ", timing card is ready" << std::endl;
	}

	/// instance class Card to hold static controller timing parameters
	Card* pcard = new Card();
	if (!pcard->readTimeCard(cardName))
	{
		OS_ERR << fullTimeStamp.localDateTimeStamp.to_dateTimeStr('-', ':');
		OS_ERR << ", failed reading timing card: " << cardName << std::endl;
		OS_ERR.close();
		if (log_type != logUtils::logType::none)
			logUtils::closeLogFiles(logFiles);
		delete pmycnf;
		return(-1);
	}

	/// open sockets
	if (!pmycnf->connectAll())
	{
		OS_ERR << fullTimeStamp.localDateTimeStamp.to_dateTimeStr('-', ':');
		OS_ERR << ", failed initiating sockets" << std::endl;
		OS_ERR.close();
		if (log_type != logUtils::logType::none)
			logUtils::closeLogFiles(logFiles);
		delete pmycnf;
		delete pcard;
		return(-1);
	}
	socketUtils::Conn_t wmeSend = pmycnf->getSocketConn(std::string("toWmeTx"));
	int fd_wmeListen = pmycnf->getSocketDescriptor(std::string("fromWmeRx"));
	socketUtils::Conn_t cloudSend = pmycnf->getSocketConn(std::string("toPedClound"));
	int fd_cloudListen = pmycnf->getSocketDescriptor(std::string("fromPedClound"));
	socketUtils::Conn_t tciSend = pmycnf->getSocketConn(std::string("toMrpTci"));
	socketUtils::Conn_t awareSend = pmycnf->getSocketConn(std::string("toMrpAware"));
	int fd_localhostListen = pmycnf->getSocketDescriptor(std::string("fromLocalhost"));

	/* ----------- intercepts signals -------------------------------------*/
	std::signal(SIGABRT, sighandler);
	std::signal(SIGFPE,  sighandler);
	std::signal(SIGINT,  sighandler);
	std::signal(SIGSEGV, sighandler);
	std::signal(SIGTERM, sighandler);

	/* ----------- local variables -------------------------------------*/
	/// receive & send UDP socket buffer
	std::vector<uint8_t> recvbuf(bufSize, 0);
	std::vector<uint8_t> sendbuf(bufSize, 0);

	/// structures to hold the latest received messages
	msgDefs::count_data_t det_cnt;
	msgDefs::pres_data_t  det_pres;
	msgDefs::vehTraj_t veh_traj;
  /// buffer to hold vehicle trajectory data used for calculating performance measures
	std::vector< std::vector<msgDefs::vehTraj_t> > a_vehTraj;
	a_vehTraj.resize(8);  /// by control phase that the trajectory is associated with
	msgDefs::intPerm_t intPerm;

	/// structure for encoding SPaT
	msgDefs::controller_state_t cntrl_state;
	cntrl_state.spatRaw.id = intersectionId;
	cntrl_state.signalStatus.mode = MsgEnum::controlMode::unavailable;

	/// set up sockets poll structure
	nfds_t nfds = 3;
	struct pollfd ufds[3];
	ufds[0].fd = fd_wmeListen;
	ufds[1].fd = fd_cloudListen;
	ufds[2].fd = fd_localhostListen;
	for (nfds_t i = 0; i < nfds; i++)
		ufds[i].events = POLLIN;
	int pollTimeout = 10;   // in milliseconds

	if (verbose)
	{
		std::cout << fullTimeStamp.localDateTimeStamp.to_dateTimeStr('-', ':');
		std::cout << ", start waiting on UDP messages..." << std::endl;
	}

	while(terminate == 0)
	{ /// wait for events
		int retval = poll(ufds, nfds, pollTimeout);
		timeUtils::getFullTimeStamp(fullTimeStamp);
		if (retval > 0)
		{
			for (nfds_t i = 0; i < nfds; i++)
			{
				if ((ufds[i].revents & POLLIN) != POLLIN)
					continue;
				ssize_t bytesReceived = recv(ufds[i].fd, (void*)&recvbuf[0], bufSize, 0);
				if (bytesReceived <= 0)
					continue;
				if ((ufds[i].fd == fd_wmeListen) || (ufds[i].fd == fd_localhostListen))
				{ /// MMITSS header + message body
					if (bytesReceived >= 9)
					{
						size_t offset = 0;
						msgUtils::mmitss_udp_header_t udpHeader;
						msgUtils::unpackHeader(recvbuf, offset, udpHeader);
						if (udpHeader.msgheader == msgUtils::msg_header)
						{ /// actions based on message ID
							if ((udpHeader.msgid == msgUtils::msgid_bsm) || (udpHeader.msgid == msgUtils::msgid_srm))
							{ /// received encoded BSM or SRM from RSE_MessageRX, forward the message to MRP_Aware
								socketUtils::sendall(awareSend, &recvbuf[0], (size_t)bytesReceived);
								if (log_type != logUtils::logType::none)
									logUtils::logMsg(logFiles, std::string("payload"), recvbuf, (size_t)bytesReceived, fullTimeStamp.localDateTimeStamp.msOfDay);
								if (verbose)
								{
									std::cout << fullTimeStamp.localDateTimeStamp.to_dateTimeStr('-', ':');
									std::cout << ", forward " << (udpHeader.msgid == msgUtils::msgid_bsm) ? std::string("BSM") : std::string("SRM");
									std::cout << " to MRP_Aware"	<< std::endl;
								}
							}
							else if (udpHeader.msgid == msgUtils::msgid_detCnt)
							{ /// received detector volume and occupancy message from MRP_TCI, save in memory
								det_cnt.ms_since_midnight = udpHeader.ms_since_midnight;
								msgDefs::unpackMsg(recvbuf, offset, det_cnt);
								if (log_type == logUtils::logType::detailLog)
									logUtils::logMsg(logFiles, std::string("cnt"), recvbuf, (size_t)bytesReceived, fullTimeStamp.localDateTimeStamp.msOfDay);
							}
							else if (udpHeader.msgid == msgUtils::msgid_detPres)
							{ /// received detector presence message from MRP_TCI, save in memory
								det_pres.ms_since_midnight = udpHeader.ms_since_midnight;
								msgDefs::unpackMsg(recvbuf, offset, det_pres);
								if (log_type == logUtils::logType::detailLog)
									logUtils::logMsg(logFiles, std::string("pres"), recvbuf, (size_t)bytesReceived, fullTimeStamp.localDateTimeStamp.msOfDay);
							}
							else if (udpHeader.msgid == msgUtils::msgid_cntrlstatus)
							{ /// encode SPaT
								cntrl_state.ms_since_midnight = udpHeader.ms_since_midnight;
								msgDefs::unpackMsg(recvbuf, offset, cntrl_state);
								ssize_t payload_size = AsnJ2735Lib::encode_spat_payload(cntrl_state.spatRaw, &sendbuf[9], bufSize);
								if (payload_size > 0)
								{	/// add MMITSS header and send to RSE_MessageTX
									size_t header_offset = 0;
									msgUtils::packHeader(sendbuf, header_offset, msgUtils::msgid_spat, fullTimeStamp.localDateTimeStamp.msOfDay, (uint16_t)payload_size);
									size_t msg_size = (size_t)payload_size + header_offset;
									socketUtils::sendall(wmeSend, &sendbuf[0], msg_size);
									if (log_type == logUtils::logType::detailLog)
										logUtils::logMsg(logFiles, std::string("payload"), sendbuf, msg_size);
									/// add Savari header and send to pedestrian cloud server
									header_offset = 0;
									msgUtils::packHeader(sendbuf, header_offset, msgUtils::savari_cloud_spat, intersectionId,
										fullTimeStamp.localDateTimeStamp.msOfDay, (uint16_t)payload_size);
									msg_size = (size_t)payload_size + header_offset;
									socketUtils::sendall(cloudSend, &sendbuf[0], msg_size);

									if (verbose)
									{
										std::cout << fullTimeStamp.localDateTimeStamp.to_dateTimeStr('-', ':');
										std::cout << ", sent SPaT" << std::endl;
									}
								}
								else
								{
									OS_ERR << fullTimeStamp.localDateTimeStamp.to_dateTimeStr('-', ':');
									OS_ERR << ", failed encode_spat_payload" << std::endl;
								}
								/// forward msgid_cntrlstatus message to MRP_Aware
								socketUtils::sendall(awareSend, &recvbuf[0], (size_t)bytesReceived);
								if (log_type == logUtils::logType::detailLog)
									logUtils::logMsg(logFiles, std::string("sig"), recvbuf, (size_t)bytesReceived, fullTimeStamp.localDateTimeStamp.msOfDay);
							}
							else if (udpHeader.msgid == msgUtils::msgid_ssm)
							{ /// received encoded SSM from MRP_Aware, forward to RSE_MessageTX
								socketUtils::sendall(wmeSend, &recvbuf[0], (size_t)bytesReceived);
								if (log_type == logUtils::logType::detailLog)
									logUtils::logMsg(logFiles, std::string("payload"), recvbuf, (size_t)bytesReceived, fullTimeStamp.localDateTimeStamp.msOfDay);
								if (verbose)
								{
									std::cout << fullTimeStamp.localDateTimeStamp.to_dateTimeStr('-', ':');
									std::cout << ", forward SSM to RSE_MessageTX" << std::endl;
								}
							}
							else if (udpHeader.msgid == msgUtils::msgid_softcall)
							{ /// received soft-call request from MRP_Aware, forward to MRP_TCI
								socketUtils::sendall(tciSend, &recvbuf[0], (size_t)bytesReceived);
								if (log_type == logUtils::logType::detailLog)
									logUtils::logMsg(logFiles, std::string("req"), recvbuf, (size_t)bytesReceived, fullTimeStamp.localDateTimeStamp.msOfDay);
								if (verbose)
								{
									std::cout << fullTimeStamp.localDateTimeStamp.to_dateTimeStr('-', ':');
									std::cout << ", forward soft-call request to MRP_TCI" << std::endl;
								}
							}
							else if (udpHeader.msgid == msgUtils::msgid_traj)
							{ /// received vehicle trajectory message from MRP_Aware, save in memory
								veh_traj.ms_since_midnight = udpHeader.ms_since_midnight;
								msgDefs::unpackMsg(recvbuf, offset, veh_traj);
								if ((veh_traj.entryControlPhase > 0) && (veh_traj.entryControlPhase <= 8))
									a_vehTraj[veh_traj.entryControlPhase - 1].push_back(veh_traj);
								if (log_type != logUtils::logType::none)
									logUtils::logMsg(logFiles, std::string("traj"), recvbuf, (size_t)bytesReceived, fullTimeStamp.localDateTimeStamp.msOfDay);
							}
							else
							{
								OS_ERR << fullTimeStamp.localDateTimeStamp.to_dateTimeStr('-', ':');
								OS_ERR << ", received unexpected MMITSS message ID " << static_cast<unsigned int>(udpHeader.msgid) << std::endl;
							}
						}
					}
				}
				else if (ufds[i].fd == fd_cloudListen)
				{	/// received message from pedestrian cloud server: Savari header + PSRM payload
					/// action: replace Savari header with MMITSS header and forward the message to MRP_Aware
					if (bytesReceived >= 9)
					{
						size_t offset = 0;
						msgUtils::savari_udp_header_t udpHeader;
						msgUtils::unpackHeader(recvbuf, offset, udpHeader);
						if ((udpHeader.type == msgUtils::savari_cloud_srm) && (udpHeader.intersectionID == intersectionId))
						{ /// received pedestrian SRM from Savari clod server,
							/// replace Savari header with MMITSS header, and send to MRP_Aware
							offset = 0;
							msgUtils::packHeader(recvbuf, offset, msgUtils::msgid_psrm, udpHeader.ms_since_midnight, udpHeader.length);
							socketUtils::sendall(awareSend, &recvbuf[0], (size_t)bytesReceived);
							if (log_type != logUtils::logType::none)
								logUtils::logMsg(logFiles, std::string("payload"), recvbuf, (size_t)bytesReceived, fullTimeStamp.localDateTimeStamp.msOfDay);
							if (verbose)
							{
								std::cout << fullTimeStamp.localDateTimeStamp.to_dateTimeStr('-', ':');
								std::cout << ", forward PSRM to MRP_Aware" << std::endl;
							}
						}
						else
						{
							OS_ERR << fullTimeStamp.localDateTimeStamp.to_dateTimeStr('-', ':');
							OS_ERR << ", received cloud message type " << static_cast<unsigned int>(udpHeader.type);
							OS_ERR << " request to intersection " << udpHeader.intersectionID;
							OS_ERR << " (this intersection " << intersectionId << " )" << std::endl;
						}
					}
					else
					{
						OS_ERR << fullTimeStamp.localDateTimeStamp.to_dateTimeStr('-', ':');
						OS_ERR << ", received unexpected message from cloud server, size=" << bytesReceived;
						OS_ERR << ", type=" << static_cast<unsigned int>(recvbuf[0]) << std::endl;
					}
				}
			}
		}

		/// check sending MAP
		if (fullTimeStamp.msec > sentMap_msec + MsgEnum::mapInterval)
		{	/// send MAP to RSE_MessageTX
			size_t offset = 3;
			msgUtils::pack4bytes(map2RSE, offset, fullTimeStamp.localDateTimeStamp.msOfDay);
			socketUtils::sendall(wmeSend, &map2RSE[0], map2RSE_size);
			/// send MAP to pedestrian cloud server
			offset = 3;
			msgUtils::pack4bytes(map2cloud, offset, fullTimeStamp.localDateTimeStamp.msOfDay);
			socketUtils::sendall(cloudSend, &map2cloud[0], map2cloud_size);

			/// reset sentMap_msec
			sentMap_msec = fullTimeStamp.msec;
			if (verbose)
			{
				std::cout << fullTimeStamp.localDateTimeStamp.to_dateTimeStr('-', ':');
				std::cout << ", sent MAP" << std::endl;
			}
		}

		/// check calculate performance measures
		if (fullTimeStamp.msec > perm_msec + permInterval)
		{
			if (cntrl_state.signalStatus.mode != MsgEnum::controlMode::unavailable)
			{
				intPerm.ms_since_midnight = fullTimeStamp.localDateTimeStamp.msOfDay;
				intPerm.mode = cntrl_state.signalStatus.mode;
				intPerm.patternNum = cntrl_state.signalStatus.patternNum;
				intPerm.permittedPhases = cntrl_state.spatRaw.permittedPhases;
				intPerm.observedPhases.reset();
				for (int i = 0; i < 8; i++)
				{
					if (getApchPerm(intPerm.apchPerm[i], a_vehTraj[i], speedLimits[i], intPerm.permittedPhases.test(i)))
						intPerm.observedPhases.set(i);
				}
				if (intPerm.observedPhases.any())
				{ /// send performance measures to MRP_Aware
					size_t msgSize = msgDefs::packMsg(sendbuf, intPerm, msgUtils::msgid_perm);
					socketUtils::sendall(awareSend, &sendbuf[0], msgSize);
					if (log_type != logUtils::logType::none)
						logUtils::logMsg(logFiles, std::string("perm"), sendbuf, msgSize);
				}
			}
			perm_msec = fullTimeStamp.msec;
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
	delete pmycnf;
	delete pcard;
	return(0);
}

size_t packMapMsg(std::vector<uint8_t>& buf, const std::vector<uint8_t>& payload, uint8_t msgid)
{ /// to RSE_MessageTX: MMITSS UDP header + encoded MAP payload
	size_t offset = 0;
	msgUtils::packHeader(buf, offset, msgid, 0, (uint16_t)payload.size());
	buf.insert(buf.begin() + offset, payload.begin(), payload.end());
	return(offset + payload.size());
}

size_t packMapMsg(std::vector<uint8_t>& buf, const std::vector<uint8_t>& payload, uint16_t intersectionID, uint8_t msgtype)
{ /// to pedestrian cloud server: Savari UDP header + encoded MAP payload
	size_t offset = 0;
	msgUtils::packHeader(buf, offset, msgtype, intersectionID, 0, (uint16_t)payload.size());
	buf.insert(buf.begin() + offset, payload.begin(), payload.end());
	return(offset + payload.size());
}

bool getApchPerm(msgDefs::apchPerm_t& apchPerm, std::vector<msgDefs::vehTraj_t>& vehTraj, uint8_t speedLimit, bool permitted)
{
	if (!permitted || vehTraj.empty() || (speedLimit == 0))
	{
		vehTraj.clear();
		return(false);
	}
	double freeflow_v = (double)speedLimit * DsrcConstants::mph2mps;
	std::vector<double> sample_v;
	std::vector<double> sample_tt;
	std::vector<double> sample_delay;
	size_t stopnums = std::count_if(vehTraj.begin(), vehTraj.end(),
		[](const msgDefs::vehTraj_t& item){return(item.stoppedTime >= 50);});
	for (auto& item : vehTraj)
	{
		double freeflow_tt = (double)item.inboundLaneLen / freeflow_v;       /// in deciseconds
		if ((item.distTraveled != 0) && (item.timeTraveled != 0))
		{
			double speed = (double)item.distTraveled / item.timeTraveled;  /// in mps
			double tt = (double)item.inboundLaneLen / speed;                   /// in deciseconds
			double delay = (tt > freeflow_tt) ? (tt - freeflow_tt) : 0.0;  /// in deciseconds
			sample_v.push_back(speed);
			sample_tt.push_back(tt);
			sample_delay.push_back(delay);
		}
	}
	if (sample_v.empty())
	{
		vehTraj.clear();
		return(false);
	}
	size_t sample_size = sample_v.size();
	double mean_v = std::accumulate(sample_v.begin(), sample_v.end(), 0.0) / (double)sample_size;
	double mean_tt = std::accumulate(sample_tt.begin(), sample_tt.end(), 0.0) / (double)sample_size;
	double mean_delay = std::accumulate(sample_delay.begin(), sample_delay.end(), 0.0) / (double)sample_size;
	std::vector<double> diff_v(sample_size);
	std::vector<double> diff_tt(sample_size);
	std::vector<double> diff_delay(sample_size);
	std::transform(sample_v.begin(), sample_v.end(), diff_v.begin(), [mean_v](double x){return(x - mean_v);});
	std::transform(sample_tt.begin(), sample_tt.end(), diff_tt.begin(), [mean_tt](double x){return(x - mean_tt);});
	std::transform(sample_delay.begin(), sample_delay.end(), diff_delay.begin(), [mean_delay](double x){return(x - mean_delay);});
	double std_v = std::sqrt(std::inner_product(diff_v.begin(), diff_v.end(), diff_v.begin(), 0.0) / (double)sample_size);
	double std_tt = std::sqrt(std::inner_product(diff_tt.begin(), diff_tt.end(), diff_tt.begin(), 0.0) / (double)sample_size);
	double std_delay = std::sqrt(std::inner_product(diff_delay.begin(), diff_delay.end(), diff_delay.begin(), 0.0) / (double)sample_size);
	apchPerm.sampleNums = static_cast<uint16_t>(sample_size);
	apchPerm.travelSpeed_mean = static_cast<uint16_t>(mean_v / DsrcConstants::mph2mps);
	apchPerm.travelSpeed_std = static_cast<uint16_t>(std_v / DsrcConstants::mph2mps);
	apchPerm.travelTime_mean = static_cast<uint16_t>(mean_tt);
	apchPerm.travelTime_std = static_cast<uint16_t>(std_tt);
	apchPerm.delay_mean = static_cast<uint16_t>(mean_delay);
	apchPerm.delay_std = static_cast<uint16_t>(std_delay);
	apchPerm.stoppedNums = static_cast<uint16_t>(stopnums);
	vehTraj.clear();
	return(true);
}
