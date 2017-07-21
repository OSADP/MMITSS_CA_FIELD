//********************************************************************************************************
//
// © 2016 Regents of the University of California on behalf of the University of California at Berkeley
//       with rights granted for USDOT OSADP distribution with the ECL-2.0 open source license.
//
//*********************************************************************************************************
/* mrpAware.cpp
 * functions:
 * 1. receive BSM (msgid_bsm), SRM (msgid_srm), PSRM (msgid_psrm) from MRP_DataMgr, decode and process the messages
 * 2. receive signal status messages (msgid_cntrlstatus) from MRP_DataMgr
 * 3. receive performance measures (msgid_perm) from MRP_DataMgr
 * 4. track BSMs on the MAP and identify those are on an approach at the intersection
 * 5. track SRM and associates with BSMs
 * 6. manage BSM, SRM & PSRM, and send soft-call requests (msgid_softcall) to MRP_TCI via MRP_DataMgr
 * 7. send encoded SSM (msgid_ssm) to RSE_MessageTX vi MRP_DataMgr
 * 8. send vehicle trajectory data (msgid_traj) to MRP_DataMgr
 * Structures of UDP messages are defined in msgDefs.h. For potability all UDP messages are serialized.
 * Functions to pack and unpack of UDP messages are defined in msgUtils.h and implemented in msgUtils.cpp
 * logs:
 * 1. display log for debugging purpose
 * 2. simpleLog
 *    - received BSM, SRM, PSRM
 *    - soft-call request message sent to MRP_DataMgr
 *    - vehicle trajectory message sent to MRP_DataMgr
 *    - SSM message send to MRP_DataMgr
 * 3. detailLog - simpleLog plus
 *
 * Soft-call logic:
 * 1. pedestrian phase call: request received from Savari pedestrian cloud server via MRP_DataMgr
 *    send soft-call request to MRP_TCI via MRP_DataMgr - if the requested pedestrian phase
 *    has not already been called (checked against ped_call)
 * 2. vehicle phase call: generated here based on received BSMs.
 *    each CV can make a phase call once per intersection approach, when:
 *    1) vehicle is on an approach to the intersection
 *    2) its control phase is not in green
 *    3) its control phase has not been called (veh_call)
 *    4) its time-to-intersection is less than maxTime2goPhaseCall (20 sec)
 *    soft vehicle phase call is determined at a time combining needs from all vehicles, so multiple phases
 *    can be called with one soft-call. soft-call on a particular phase can not be faster than vehPhaseCallInterval (1 sec)
 * 3. vehicle phase extension call:  generated here based on received BSMs.
 *    each CV can make a phase extension call once per intersection approach, when:
 *    1) signal is running free or under coordination
 *    2) controller is not serving a priority treatment
 *    3) vehicle is on an approach to the intersection
 *    4) its control phase is in green (not a coordinated phase when under coordination)
 *    5) its control phase is not serving an ongoing phase extension
 *    6) the vehicle is expected to arrive within maxTime2phaseExt (5 sec) after the remaining green
 *    7) control phase green is about to be terminated (within maxTime2changePhaseExt = 4 sec)
 *    soft vehicle phase extension call is determined at a time combining needs from all vehicles, so concurrent phases
 *    can be requested with one soft-call, and call on one phase could cover multiple vehicles.
 *    vehicle phase extension request is cancel when:
 *    1) the requested phase green has been terminated, or
 *    2) vehicles contributed to the phase extension request have passed the stop-bar
 * 4. priority call:  generated here based on received SRMs
 *    each priority-eligible CV can make a priority request once per intersection approach, when:
 *    1) signal is under coordination
 *    2) controller is not serving a priority treatment
 *    3) vehicle is on an approach to the intersection
 *    4) its control phase is a coordinated phase
 *    Early green
 *    EG 5) its control phase is in yellow and the vehicle can't pass during the first half of yellow, or
 *    EG 6) its control phase is in red and the vehicle is expected to arrive at the stop-bar within red
 *    Green extension
 *    GE 5) its control phase is in green and the vehicle is expecting to arrive within maxGreenExtenstion after the remaining green
 *    GE 6) sync phase green is about to be terminated (with maxTime2changePhaseExt)
 *    priority request is determined at a time combining needs from all vehicles, green extension has higher priority than early green
 *    priority request is cancel when:
 *    GE 1) the requested phase green has been terminated
 *    EG 1) the requested phase has turned green, or
 *       2) vehicle contributed to the priority request has passed the stop-bar
 *
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <fstream>
#include <iostream>
#include <string>
#include <sys/socket.h>
#include <sys/types.h>
#include <thread>
#include <unistd.h>

#include "AsnJ2735Lib.h"
#include "locAware.h"
#include "cnfUtils.h"
#include "logUtils.h"
#include "msgUtils.h"
#include "socketUtils.h"
#include "mrpAware.h"

void do_usage(const char* progname)
{
	std::cerr << "Usage" << progname << std::endl;
	std::cerr << "\t-n intersection name" << std::endl;
	std::cerr << "\t-s full path to mrpAwr.conf" << std::endl;
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
	unsigned long long timeouInterval = pmycnf->getIntegerParaValue(std::string("dsrcTimeout")) * 1000;     // in milliseconds
	int logType = pmycnf->getIntegerParaValue(std::string("logType"));
	logUtils::logType log_type = ((logInterval == 0) || ((logType != 1) && (logType != 2)))
		? logUtils::logType::none : static_cast<logUtils::logType>(logType);
	uint16_t maxGreenExtenstion     = (uint16_t)(pmycnf->getIntegerParaValue(std::string("maxGreenExtenstion")) * 10);  // in tenths of a second
	uint16_t maxTime2changePhaseExt = (uint16_t)(pmycnf->getIntegerParaValue(std::string("maxTime2change4Ext")) * 10);  // in tenths of a second
	uint16_t maxTime2goPhaseCall    = (uint16_t)(pmycnf->getIntegerParaValue(std::string("maxTime2goPhaseCall")) * 10); // in tenths of a second
	uint16_t maxTime2phaseExt       = (uint16_t)(pmycnf->getIntegerParaValue(std::string("maxTime4phaseExt")) * 10);    // in tenths of a second
	std::string fnmap = pmycnf->getStringParaValue(std::string("nmapFile"));
	std::string logPath = pmycnf->getStringParaValue(std::string("logPath"));
	unsigned long long logfile_msec = 0;

	/// open error log
	std::ofstream OS_ERR(logPath + std::string("/awr.err"), std::ofstream::app);
	if (!OS_ERR.is_open())
	{
		std::cerr << fullTimeStamp.localDateTimeStamp.to_dateTimeStr('-', ':');
		std::cerr << ", failed initiating err log" << std::endl;
		delete pmycnf;
		return(-1);
	}

	/// open display log (for debugging purpose, keep the latest 15 minutes record)
	std::string displayLog = logPath + std::string("/display.log");
	std::ofstream OS_Display(displayLog);
	if (!OS_Display.is_open())
	{
		OS_ERR << fullTimeStamp.localDateTimeStamp.to_dateTimeStr('-', ':');
		OS_ERR << ", failed initiating display log" << std::endl;
		OS_ERR.close();
		delete pmycnf;
		return(-1);
	}
	unsigned long long displayLogInterval = 15 * 60 * 1000;  /// 15 minutes in milliseconds
	unsigned long long displayfile_msec = fullTimeStamp.msec;

	/// open log files
	std::vector<logUtils::Logfile_t> logFiles;
	if (log_type != logUtils::logType::none)
	{
		std::vector<std::string> logtypes;
		logtypes.push_back(std::string("payload"));  // encoded payload of inbound BSM, SRM, PSRM and outbound SSM
		logtypes.push_back(std::string("req"));      // log soft-call request message send to MRP_DataMgr
		logtypes.push_back(std::string("traj"));     // log vehicle trajectory message sent to MRP_DataMgr
		std::string prefix = logPath + std::string("/") + intersectionName;
		for (auto& type : logtypes)
			{logFiles.push_back(logUtils::Logfile_t(prefix, type));}
		if (!openLogFiles(logFiles, fullTimeStamp.localDateTimeStamp.to_fileName()))
		{
			OS_ERR << fullTimeStamp.localDateTimeStamp.to_dateTimeStr('-', ':');
			OS_ERR << ", failed openLogFiles" << std::endl;
			OS_ERR.close();
			OS_Display.close();
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
		OS_Display.close();
		if (log_type != logUtils::logType::none)
			logUtils::closeLogFiles(logFiles);
		delete pmycnf;
		delete plocAwareLib;
		return(-1);
	}
	uint8_t intersectionIndex = plocAwareLib->getIndexByIntersectionId(intersectionId);
	GeoUtils::geoRefPoint_t intGeoRef = plocAwareLib->getIntersectionRefPoint(intersectionIndex);
	std::vector<uint8_t> speedLimits(8, 0); // in mph
	plocAwareLib->getSpeedLimits(speedLimits, intersectionId);
	SSM_element_t ssm;
	ssm.id = intersectionId;
	unsigned long long sentSSM_msec = 0;

	/// open sockets
	if (!pmycnf->connectAll())
	{
		OS_ERR << fullTimeStamp.localDateTimeStamp.to_dateTimeStr('-', ':');
		OS_ERR << ", failed initiating sockets" << std::endl;
		OS_ERR.close();
		OS_Display.close();
		if (log_type != logUtils::logType::none)
			logUtils::closeLogFiles(logFiles);
		delete pmycnf;
		delete plocAwareLib;
		return(-1);
	}
	socketUtils::Conn_t sendConn = pmycnf->getSocketConn(std::string("toDataMgr"));
	int fd_Listen = pmycnf->getSocketDescriptor(std::string("fromDataMgr"));

	/* ----------- intercepts signals -------------------------------------*/
	std::signal(SIGABRT, sighandler);
	std::signal(SIGFPE,  sighandler);
	std::signal(SIGINT,  sighandler);
	std::signal(SIGSEGV, sighandler);
	std::signal(SIGTERM, sighandler);

	/* ----------- local variables -------------------------------------*/
	/// control parameters:
	/// thresholds for whether or not to conduct locating vehicle on MAP,
	/// no need to re-do mapping when speed < stopSpeed & distance_traveled < stopDist.
	const double stopSpeed = 2;          // in m/s
	const double stopDist = 5.0;         // in meters
	unsigned long long vehPhaseCallInterval = 1000LL;  // in milliseconds
	/// receive & send UDP socket buffer
	const size_t bufSize = 2000;
	std::vector<uint8_t> recvbuf(bufSize, 0);
	std::vector<uint8_t> sendbuf(bufSize, 0);
	/// vehList to store BSMs and results of locating BSMs on MAP
	std::vector<cvStatusAware_t> vehList;
	/// srmList to store SRMs and status of priority request
	std::vector<srmStatus_t> srmList;
	/// awareStatus to trace status of MRP_Aware component
	aware_status_t awareStatus;
	awareStatus.reset();

	if (verbose)
	{
		std::cout << fullTimeStamp.localDateTimeStamp.to_dateTimeStr('-', ':');
		std::cout << ", start waiting on UDP messages..." << std::endl;
	}

	while(terminate == 0)
	{ /// receiving UDP message (non-blocking)
		uint32_t bsm_vehId = 0;  // set to BSM::TemporaryID when received an BSM
		ssize_t bytesReceived = recv(fd_Listen, &recvbuf[0], bufSize, 0);
		if (bytesReceived >= 9)
		{ /// MMITSS header + message body
			timeUtils::getFullTimeStamp(fullTimeStamp);
			size_t offset = 0;
			msgUtils::mmitss_udp_header_t udpHeader;
			msgUtils::unpackHeader(recvbuf, offset, udpHeader);
			if (udpHeader.msgheader == msgUtils::msg_header)
			{ /// actions based on message ID
				if ((udpHeader.msgid == msgUtils::msgid_bsm)
					&& (awareStatus.cntrlState.signalStatus.mode != MsgEnum::controlMode::unavailable))
				{ /// decode BSM
					BSM_element_t bsm;
					if (AsnJ2735Lib::decode_bsm_payload(&recvbuf[offset], udpHeader.length, bsm) > 0)
					{ /// when elevation is not included, use elevation of intersection reference point
						if (bsm.elevation == MsgEnum::unknown_elevation)
							bsm.elevation = intGeoRef.elevation;
						/// check whether it is an update BSM on vehList
						auto it = std::find_if(vehList.begin(), vehList.end(), [&bsm](cvStatusAware_t& obj){return(obj.bsm.id == bsm.id);});
						if (it == vehList.end())
						{ /// add BSM to vehList
							cvStatusAware_t cvStatusAware;
							cvStatusAware.reset();
							cvStatusAware.msec = fullTimeStamp.msec;
							cvStatusAware.bsm = bsm;
							vehList.push_back(cvStatusAware);
							bsm_vehId = bsm.id;
						}
						else if ((bsm.msgCnt > it->bsm.msgCnt) || (bsm.timeStampSec > it->bsm.timeStampSec))
						{ /// this is an update BSM on vehList
							it->msec = fullTimeStamp.msec;
							it->bsm = bsm;
							bsm_vehId = bsm.id;
						}
						if ((bsm_vehId > 0) && (log_type != logUtils::logType::none))
							logUtils::logMsg(logFiles, std::string("payload"), recvbuf, (size_t)bytesReceived, fullTimeStamp.localDateTimeStamp.msOfDay);
					}
					else
					{
						OS_ERR << fullTimeStamp.localDateTimeStamp.to_dateTimeStr('-', ':');
						OS_ERR << ", failed decode_bsm_payload, payload=";
						logUtils::logMsgHex(OS_ERR, &recvbuf[offset], udpHeader.length);
						OS_Display << fullTimeStamp.localDateTimeStamp.to_dateTimeStr('-', ':');
						OS_Display << ", failed decode_bsm_payload" << std::endl;
					}
				}
				else if ((udpHeader.msgid == msgUtils::msgid_srm)
					&& (awareStatus.cntrlState.signalStatus.mode != MsgEnum::controlMode::unavailable))
				{ /// decode SRM
					SRM_element_t srm;
					if (AsnJ2735Lib::decode_srm_payload(&recvbuf[offset], udpHeader.length, srm) > 0)
					{	/// validate SRM
						if (srm.intId == intersectionId)
						{ /// when elevation is not included, use elevation of intersection reference point
							if (srm.elevation == MsgEnum::unknown_elevation)
								srm.elevation = intGeoRef.elevation;
							if (srm.inApprochId == 0)
								srm.inApprochId = plocAwareLib->getApproachIdByLaneId(srm.intId, srm.inLaneId);
							if (srm.outApproachId == 0)
								srm.outApproachId = plocAwareLib->getApproachIdByLaneId(srm.intId, srm.outLaneId);
							uint8_t requestedPhase = plocAwareLib->getControlPhaseByIds(intersectionId, srm.inApprochId, srm.inLaneId);
							/// check whether it is an update SRM on srmList
							auto it = std::find_if(srmList.begin(), srmList.end(), [&srm](srmStatus_t obj){return(obj.srm.vehId == srm.vehId);});
							/// check whether the requesting vehicle has BSM on vehList
							auto it_veh = std::find_if(vehList.begin(), vehList.end(), [&srm](cvStatusAware_t& obj){return(obj.bsm.id == srm.vehId);});
							if (it_veh != vehList.end())
							{ /// the requesting vehicle should broadcast BSMs as well.
								/// SRMs are not required to send at a constant rate rather than need-basis,
								/// so BSMs from the requesting vehicles are used for locating vehicle on MAP.
								if (it == srmList.end())
								{ /// add SRM to srmList
									srmStatus_t srmStatus;
									srmStatus.msec = fullTimeStamp.msec;
									srmStatus.requestedPhase = requestedPhase;
									srmStatus.srm = srm;
									const auto& signalStatus = awareStatus.cntrlState.signalStatus;
									const auto& spat = awareStatus.cntrlState.spatRaw;
									/// determining initial requestStatus
									if (((srm.reqType == MsgEnum::requestType::priorityRequest) || (srm.reqType == MsgEnum::requestType::requestUpdate))
											&& (requestedPhase > 0) && spat.permittedPhases.test(requestedPhase - 1)
											&& (signalStatus.mode == MsgEnum::controlMode::coordination)  /// under coordination control
											&& signalStatus.coordinated_phases.test(requestedPhase - 1)   /// request on coordinated phase
											&& it_veh->isOnInbound)                                /// the requesting vehicle is onInbound
										srmStatus.status = MsgEnum::requestStatus::requested;
									else
										srmStatus.status = MsgEnum::requestStatus::rejected;
									srmList.push_back(srmStatus);
									awareStatus.requestStatusUpdated = true;
								}
								else if ((srm.msgCnt > it->srm.msgCnt) || (srm.timeStampSec > it->srm.timeStampSec))
								{ /// this is an update SRM on srmList, it should maintain requestedPhase
									it->msec = fullTimeStamp.msec;
									it->srm = srm;
									awareStatus.requestStatusUpdated = true;
								}
								if (log_type != logUtils::logType::none)
									logUtils::logMsg(logFiles, std::string("payload"), recvbuf, (size_t)bytesReceived, fullTimeStamp.localDateTimeStamp.msOfDay);
							}
						}
					}
					else
					{
						OS_ERR << fullTimeStamp.localDateTimeStamp.to_dateTimeStr('-', ':');
						OS_ERR << ", failed decode_srm_payload, payload=";
						logUtils::logMsgHex(OS_ERR, &recvbuf[offset], udpHeader.length);
						OS_Display << fullTimeStamp.localDateTimeStamp.to_dateTimeStr('-', ':');
						OS_Display << ", failed decode_srm_payload" << std::endl;
					}
				}
				else if ((udpHeader.msgid == msgUtils::msgid_psrm)
					&& (awareStatus.cntrlState.signalStatus.mode != MsgEnum::controlMode::unavailable))
				{ /// decode PSRM (same format as SRM)
					SRM_element_t psrm;
					if (AsnJ2735Lib::decode_srm_payload(&recvbuf[offset], udpHeader.length, psrm) > 0)
					{	/// validate requested pedestrian phase
						if (psrm.intId == intersectionId)
						{
							if (psrm.inApprochId == 0)
								psrm.inApprochId = plocAwareLib->getApproachIdByLaneId(psrm.intId, psrm.inLaneId);
							if (psrm.outApproachId == 0)
								psrm.outApproachId = plocAwareLib->getApproachIdByLaneId(psrm.intId, psrm.outLaneId);
							uint8_t requestedPhase = plocAwareLib->getControlPhaseByIds(intersectionId, psrm.inApprochId, psrm.inLaneId);
							if ((requestedPhase > 0) && (awareStatus.cntrlState.spatRaw.permittedPedPhases.test(requestedPhase - 1)))
							{
								OS_Display << fullTimeStamp.localDateTimeStamp.to_dateTimeStr('-', ':');
								OS_Display << ", received PSRM on phase " << static_cast<unsigned int>(requestedPhase) << std::endl;
								const auto& signalStatus = awareStatus.cntrlState.signalStatus;
								if (!signalStatus.ped_call.test(requestedPhase - 1)
									&& (signalStatus.call_status[requestedPhase - 1] != MsgEnum::phaseCallType::ped)
									&& (signalStatus.recall_status[requestedPhase - 1] != MsgEnum::phaseRecallType::ped))
								{ /// send pedestrian phase soft-call to MRP_DataMgr
									size_t msgSize = packMsg(sendbuf, requestedPhase, MsgEnum::softCallObj::ped, MsgEnum::softCallType::call,
										fullTimeStamp.localDateTimeStamp.msOfDay, msgUtils::msgid_softcall);
									socketUtils::sendall(sendConn, &sendbuf[0], msgSize);
									if (log_type != logUtils::logType::none)
										logUtils::logMsg(logFiles, std::string("req"), sendbuf, msgSize);
									OS_Display << fullTimeStamp.localDateTimeStamp.to_dateTimeStr('-', ':');
									OS_Display << ", sent pedestrian call on phase " << static_cast<unsigned int>(requestedPhase) << std::endl;
								}
								if (log_type != logUtils::logType::none)
									logUtils::logMsg(logFiles, std::string("payload"), recvbuf, (size_t)bytesReceived, fullTimeStamp.localDateTimeStamp.msOfDay);
							}
							else
							{
								OS_ERR << fullTimeStamp.localDateTimeStamp.to_dateTimeStr('-', ':');
								OS_ERR << ", unexpected PSRM requested phase, inApprochId=" << static_cast<unsigned int>(psrm.inApprochId);
								OS_ERR << ", inLaneId=" << static_cast<unsigned int>(psrm.inLaneId);
								OS_ERR << ", requestedPhase=" << static_cast<unsigned int>(requestedPhase) << std::endl;
							}
						}
						else
						{
							OS_ERR << fullTimeStamp.localDateTimeStamp.to_dateTimeStr('-', ':');
							OS_ERR << ", unexpected PSRM requested intersectionId=" << psrm.intId << std::endl;
						}
					}
					else
					{
						OS_ERR << fullTimeStamp.localDateTimeStamp.to_dateTimeStr('-', ':');
						OS_ERR << ", failed pedestrian decode_srm_payload, payload=";
						logUtils::logMsgHex(OS_ERR, &recvbuf[offset], udpHeader.length);
						OS_Display << fullTimeStamp.localDateTimeStamp.to_dateTimeStr('-', ':');
						OS_Display << ", failed pedestrian decode_srm_payload" << std::endl;
					}
				}
				else if (udpHeader.msgid == msgUtils::msgid_cntrlstatus)
				{
					awareStatus.cntrlState.ms_since_midnight = udpHeader.ms_since_midnight;
					msgDefs::unpackMsg(recvbuf, offset, awareStatus.cntrlState);
					/// update cycleCtn
					const auto& synch_phase = awareStatus.cntrlState.signalStatus.synch_phase;
					const auto& syncPhaseState = awareStatus.cntrlState.spatRaw.phaseState[synch_phase - 1].currState;
					if (awareStatus.syncPhaseState == MsgEnum::phaseState::unavailable)
						awareStatus.syncPhaseState = syncPhaseState;
					if (awareStatus.syncPhaseState != syncPhaseState)
					{
						if (syncPhaseState == MsgEnum::phaseState::protectedYellow)
						{
							awareStatus.cycleCtn = (uint8_t)(awareStatus.cycleCtn + 1 % 127);
							OS_Display << fullTimeStamp.localDateTimeStamp.to_dateTimeStr('-', ':');
							OS_Display << " cycleCtn = " << static_cast<unsigned int>(awareStatus.cycleCtn) << std::endl;
						}
						awareStatus.syncPhaseState = syncPhaseState;
					}
				}
				else if (udpHeader.msgid == msgUtils::msgid_perm)
				{
					awareStatus.intPerm.ms_since_midnight = udpHeader.ms_since_midnight;
					msgDefs::unpackMsg(recvbuf, offset, awareStatus.intPerm);
				}
			}
		}

		if (bsm_vehId > 0)
		{ /// received a new BSM, the vehicle has already been added to vehList
			auto it = std::find_if(vehList.begin(), vehList.end(), [&bsm_vehId](cvStatusAware_t& obj){return(obj.bsm.id == bsm_vehId);});
			const auto& bsm = it->bsm;
			/// construct GeoUtils::connectedVehicle_t from the newly received BSM
			GeoUtils::connectedVehicle_t cvIn;
			cvIn.reset();
			GeoUtils::geoRefPoint_t geoRef{bsm.latitude, bsm.longitude, bsm.elevation};
			GeoUtils::geoRefPoint2geoPoint(geoRef, cvIn.geoPoint);
			cvIn.id = bsm_vehId;
			cvIn.msec = it->msec;
			cvIn.motionState.speed = (double)bsm.speed * 0.02;
			cvIn.motionState.heading = (double)bsm.heading * 0.0125;
			bool first_bsm = (it->cvStatus.empty()) ? true : false;
			if (first_bsm)
				it->cvStatus.push_back(cvIn);
			/// get the latest result of locating vehicle on MAP
			auto cv = it->cvStatus.back();
			/// determining whether or not to conduct locating vehicle on MAP
			bool doMapping = (first_bsm) ? true : false;
			if (!first_bsm)
			{ /// get distance traveled from the last geoPoint
				double distTravelled = GeoUtils::distlla2lla(cvIn.geoPoint, cv.geoPoint);
				if ((cvIn.motionState.speed  >= stopSpeed) || (fabs(distTravelled) >= stopDist))
					doMapping = true;
			}
			if (!doMapping)
			{ /// keep the latest result of locating vehicle on MAP
				cvIn.isVehicleInMap = cv.isVehicleInMap;
				cvIn.vehicleTrackingState = cv.vehicleTrackingState;
				cvIn.vehicleLocationAware = cv.vehicleLocationAware;
			}
			else
			{ /// locate the vehicle on MAP
				cv.geoPoint = cvIn.geoPoint;
				cv.motionState = cvIn.motionState;
				cvIn.isVehicleInMap = plocAwareLib->locateVehicleInMap(cv, cvIn.vehicleTrackingState);
				/// update locationAware
				plocAwareLib->updateLocationAware(cvIn.vehicleTrackingState, cvIn.vehicleLocationAware);
			}
			/// update signalAware
			cvIn.vehicleSignalAware.reset();
			if (cvIn.isVehicleInMap
				&& (cvIn.vehicleTrackingState.intsectionTrackingState.intersectionIndex == intersectionIndex)
				&& (cvIn.vehicleLocationAware.controlPhase > 0)
				&& (awareStatus.cntrlState.signalStatus.mode != MsgEnum::controlMode::unavailable))
			{
				const auto& phaseState = awareStatus.cntrlState.spatRaw.phaseState[cvIn.vehicleLocationAware.controlPhase - 1];
				cvIn.vehicleSignalAware.currState = phaseState.currState;
				cvIn.vehicleSignalAware.startTime = phaseState.startTime;
				cvIn.vehicleSignalAware.minEndTime = phaseState.minEndTime;
				cvIn.vehicleSignalAware.maxEndTime = phaseState.maxEndTime;
			}

			/// update cvStatusAware with the current mapping result
			if (it->isOnInbound)
			{	/// the vehicle was onInbound of this intersection, possible current vehicleIntersectionStatus:
				/// case 1: the vehicle remains on the same inbound approach or moved inside intersection box
				if (cvIn.isVehicleInMap
					&& (cvIn.vehicleTrackingState.intsectionTrackingState.intersectionIndex == intersectionIndex)
					&& ((cvIn.vehicleTrackingState.intsectionTrackingState.vehicleIntersectionStatus == MsgEnum::mapLocType::onInbound)
						|| (cvIn.vehicleTrackingState.intsectionTrackingState.vehicleIntersectionStatus == MsgEnum::mapLocType::atIntersectionBox))
					&& (cvIn.vehicleTrackingState.intsectionTrackingState.approachIndex == cv.vehicleTrackingState.intsectionTrackingState.approachIndex))
				{ /// add tracked point to cvStatusAware::cvStatus
					if (doMapping)
						it->cvStatus.push_back(cvIn);
				}
				/// case 2: the vehicle moved to outbound approach of this intersection
				else if (cvIn.isVehicleInMap
					&& (cvIn.vehicleTrackingState.intsectionTrackingState.intersectionIndex == intersectionIndex)
					&& (cvIn.vehicleTrackingState.intsectionTrackingState.vehicleIntersectionStatus == MsgEnum::mapLocType::onOutbound))
				{
					if (it->cvStatus.size() >= 10)
					{ /// send msgid_traj message to MRP_dataMgr
						uint32_t laneLen = plocAwareLib->getLaneLength(intersectionId, it->cvStatus[0].vehicleLocationAware.laneId);
						size_t msgSize = packMsg(sendbuf, *it, fullTimeStamp.localDateTimeStamp.dateStamp,
							fullTimeStamp.localDateTimeStamp.msOfDay,	laneLen, stopSpeed, msgUtils::msgid_traj);
						socketUtils::sendall(sendConn, &sendbuf[0], msgSize);
						if (log_type != logUtils::logType::none)
							logUtils::logMsg(logFiles, std::string("traj"), sendbuf, msgSize);
					}
					/// reset cvStatusAware
					it->reset();
					it->cvStatus.push_back(cvIn);
					OS_Display << fullTimeStamp.localDateTimeStamp.to_dateTimeStr('-', ':');
					OS_Display << " vehicle " << cvIn.id << " entered outbound lane ";
					OS_Display << static_cast<unsigned int>(cvIn.vehicleLocationAware.laneId) << std::endl;
				}
				/// case 3: the vehicle changed intersection
				else if (cvIn.isVehicleInMap
					&& (cvIn.vehicleTrackingState.intsectionTrackingState.intersectionIndex != intersectionIndex))
				{
					if (it->cvStatus.size() >= 10)
					{ /// send msgid_traj message to MRP_dataMgr
						uint32_t laneLen = plocAwareLib->getLaneLength(intersectionId, it->cvStatus[0].vehicleLocationAware.laneId);
						size_t msgSize = packMsg(sendbuf, *it, fullTimeStamp.localDateTimeStamp.dateStamp,
							fullTimeStamp.localDateTimeStamp.msOfDay,	laneLen, stopSpeed, msgUtils::msgid_traj);
						socketUtils::sendall(sendConn, &sendbuf[0], msgSize);
						if (log_type != logUtils::logType::none)
							logUtils::logMsg(logFiles, std::string("traj"), sendbuf, msgSize);
					}
					/// reset cvStatusAware
					it->reset();
					it->cvStatus.push_back(cvIn);
					OS_Display << fullTimeStamp.localDateTimeStamp.to_dateTimeStr('-', ':');
					OS_Display << " vehicle " << cvIn.id << " entered intersection ";
					OS_Display << plocAwareLib->getIntersectionNameByIndex(cvIn.vehicleTrackingState.intsectionTrackingState.intersectionIndex);
					OS_Display << " on lane " << static_cast<unsigned int>(cvIn.vehicleLocationAware.laneId);
					OS_Display << ", phase " << static_cast<unsigned int>(cvIn.vehicleLocationAware.controlPhase) << std::endl;
				}
				/// case 4: the vehicle moved outside of the MAP area
				else if (!cvIn.isVehicleInMap)
				{
					if (it->cvStatus.size() >= 10)
					{ /// send msgid_traj message to MRP_dataMgr
						uint32_t laneLen = plocAwareLib->getLaneLength(intersectionId, it->cvStatus[0].vehicleLocationAware.laneId);
						size_t msgSize = packMsg(sendbuf, *it, fullTimeStamp.localDateTimeStamp.dateStamp,
							fullTimeStamp.localDateTimeStamp.msOfDay,	laneLen, stopSpeed, msgUtils::msgid_traj);
						socketUtils::sendall(sendConn, &sendbuf[0], msgSize);
						if (log_type != logUtils::logType::none)
							logUtils::logMsg(logFiles, std::string("traj"), sendbuf, msgSize);
					}
					/// reset cvStatusAware
					it->reset();
					it->cvStatus.push_back(cvIn);
					OS_Display << fullTimeStamp.localDateTimeStamp.to_dateTimeStr('-', ':');
					OS_Display << " vehicle " << cvIn.id << " left MAP area" << std::endl;
				}
			}
			/// the vehicle was not on any inbound approach and now entered on an inbound approach of this intersection
			else if (cvIn.isVehicleInMap
				&& (cvIn.vehicleTrackingState.intsectionTrackingState.intersectionIndex == intersectionIndex)
				&& (cvIn.vehicleTrackingState.intsectionTrackingState.vehicleIntersectionStatus == MsgEnum::mapLocType::onInbound))
			{	/// this is the first record of vehicle onInbound at this intersection
				it->isOnInbound = true;
				it->cvStatus[0] = cvIn;
				OS_Display << fullTimeStamp.localDateTimeStamp.to_dateTimeStr('-', ':');
				OS_Display << " vehicle " << cvIn.id << " entered inbound lane ";
				OS_Display << static_cast<unsigned int>(cvIn.vehicleLocationAware.laneId) << ", phase ";
				OS_Display << static_cast<unsigned int>(cvIn.vehicleLocationAware.controlPhase) << ", dist2go ";
				OS_Display << cvIn.vehicleLocationAware.dist2go.distLong << std::endl;
			}
			///  the vehicle neither was nor currently on any inbound approach of this intersection
			else if (doMapping)
				it->cvStatus[0] = cvIn;
		}

		/// wait until started receiving msgid_cntrlstatus messages (controller and signal status)
		if (awareStatus.cntrlState.signalStatus.mode == MsgEnum::controlMode::unavailable)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(5));
			continue;
		}

		timeUtils::getFullTimeStamp(fullTimeStamp);
		const auto& spat = awareStatus.cntrlState.spatRaw;
		const auto& signalStatus = awareStatus.cntrlState.signalStatus;
		auto& prioServingStatus = awareStatus.prioServingStatus;
		auto& grantingType = prioServingStatus.grantingType;

		/// check cancel of on-going priority treatment
		if (grantingType != prioGrantType::none)
		{
			const auto& grantingVehId = prioServingStatus.grantingVehId;
			const auto& grantingPhase = prioServingStatus.grantingPhase;
			const auto& phaseState = spat.phaseState[grantingPhase - 1];
			bool phaseInGreen = ((phaseState.currState == MsgEnum::phaseState::permissiveGreen)
				|| (phaseState.currState == MsgEnum::phaseState::protectedGreen));
			/// find grantVehicle on vehList
			auto it = std::find_if(vehList.begin(), vehList.end(), [&grantingVehId](cvStatusAware_t& obj){return(obj.bsm.id == grantingVehId);});
			/// find grantVehicle on srmList
			auto it_srm = std::find_if(srmList.begin(), srmList.end(), [&grantingVehId](srmStatus_t& obj){return(obj.srm.vehId == grantingVehId);});
			/// 1. cancel earlyGreen if grantingPhase turned green
			/// 2. cancel greenExtension if green on grantingPhase expired
			/// 3. cancel priority if grantVehicle has passed the intersection (not onInbound)
			/// 4. cancel priority if grantVehicle sent cancel SRM
			if (((grantingType == prioGrantType::earlyGreen) && phaseInGreen)
				|| ((grantingType == prioGrantType::greenExtension) && !phaseInGreen)
				|| ((it == vehList.end()) || !it->isOnInbound)
				|| ((it_srm != srmList.end()) && (it_srm->srm.reqType == MsgEnum::requestType::priorityCancellation)))
			{ /// reset grantingType
				grantingType = prioGrantType::none;
				/// remove grantVehicle from srmList
				if (it_srm != srmList.end())
					it_srm = srmList.erase(it_srm);
				/// send Cancel priority request to MRP_DataMgr
				size_t msgSize = packMsg(sendbuf, grantingPhase, MsgEnum::softCallObj::priority, MsgEnum::softCallType::cancel,
					fullTimeStamp.localDateTimeStamp.msOfDay, msgUtils::msgid_softcall);
				socketUtils::sendall(sendConn, &sendbuf[0], msgSize);
				if (log_type != logUtils::logType::none)
					logUtils::logMsg(logFiles, std::string("req"), sendbuf, msgSize);
				OS_Display << fullTimeStamp.localDateTimeStamp.to_dateTimeStr('-', ':');
				OS_Display << ", cancel priority on phase " << static_cast<unsigned int>(grantingPhase) << std::endl;
			}
		}

		/// check initializing priority treatment, when
		///   1) controller is not currently serving a priority
		///   2) signal is under coordination control
		///   3) no priority has been served in the current signal cycle
		if ((grantingType == prioGrantType::none)
			&& (signalStatus.mode == MsgEnum::controlMode::coordination)
			&& (awareStatus.cycleCtn != prioServingStatus.grantingCycleCnt))
		{ /// loop through srmList to get candidate SRMs for initializing a priority treatment.
			std::vector<prioRequestCadidate_t> greenExtensionCandidates;
			std::vector<prioRequestCadidate_t> earlyGreenCandidates;
			for (auto& srmStatus : srmList)
			{
				if ((srmStatus.status != MsgEnum::requestStatus::requested) || (srmStatus.status != MsgEnum::requestStatus::processing))
					continue;
				const auto& srm = srmStatus.srm;
				if ((srm.reqType != MsgEnum::requestType::priorityRequest) || (srm.reqType != MsgEnum::requestType::requestUpdate))
					continue;
				if (srmStatus.status == MsgEnum::requestStatus::requested)
				{
					srmStatus.status = MsgEnum::requestStatus::processing;
					awareStatus.requestStatusUpdated = true;
				}
				/// find the vehicle on vehList, and check whether it is onInbound
				auto it = std::find_if(vehList.begin(), vehList.end(), [&srm](cvStatusAware_t& obj){return(obj.bsm.id == srm.vehId);});
				if (it == vehList.end() || !it->isOnInbound)
					continue;
				/// get the latest result of locating vehicle on MAP
				const auto& cv = it->cvStatus.back();
				/// estimate time-to-arrival at the stop-bar (tenths of a second in the current or next hour)
				uint16_t time2go = getTime2Go(cv.vehicleLocationAware.dist2go.distLong, cv.motionState.speed, stopSpeed);
				uint16_t time2arrival = (uint16_t)((fullTimeStamp.msec / 100 + time2go) % 36000);
				/// get status of requesting phase and sync phase
				const auto& phaseState = spat.phaseState[srmStatus.requestedPhase - 1];
				const auto& syncPhaseState = spat.phaseState[signalStatus.synch_phase - 1];
				bool phaseInGreen = ((phaseState.currState == MsgEnum::phaseState::permissiveGreen)
					|| (phaseState.currState == MsgEnum::phaseState::protectedGreen));
				bool syncPhaseInGreen = ((syncPhaseState.currState == MsgEnum::phaseState::permissiveGreen)
					|| (syncPhaseState.currState == MsgEnum::phaseState::protectedGreen));
				/// required conditions to grant a greenExtension treatment:
				///   1) requested phase is green
				///   2) expected time2arrival is within allowable green extension window
				///   3) sync phase is green
				///   4) controller local cycle clock is close to the yield point (cycle-end)
				if (phaseInGreen && syncPhaseInGreen
					&& withinTimeWindow(phaseState.minEndTime, maxGreenExtenstion, time2arrival)
					&& ((uint16_t)(signalStatus.local_cycle_clock + maxTime2changePhaseExt) >= signalStatus.cycle_length))
				{
					uint16_t duration = getDuration(phaseState.minEndTime, time2arrival);
					prioRequestCadidate_t prioRequestCadidate{srm.vehId, srmStatus.requestedPhase, duration};
					greenExtensionCandidates.push_back(prioRequestCadidate);
				}
				/// required conditions to grant an earlyGreen treatment:
				///   1) requested phase is red
				///   2) expected time2arrival is before minEndTime of requested phase
				else if (!phaseInGreen && (phaseState.currState != MsgEnum::phaseState::protectedYellow)
					&& isTimeBefore(time2arrival, phaseState.minEndTime))
				{
					uint16_t duration = getDuration(time2arrival, phaseState.minEndTime);
					prioRequestCadidate_t prioRequestCadidate{srm.vehId, srmStatus.requestedPhase, duration};
					earlyGreenCandidates.push_back(prioRequestCadidate);
				}
			}
			/// greenExtension treatment has higher priority than earlyGreen treatment as greenExtension
			/// can eliminate unnecessary stops at the intersection.
			if (!greenExtensionCandidates.empty())
			{ /// find the index in greenExtensionCandidates that having maximum duration
				size_t indx = getCandidateIndex(greenExtensionCandidates);
				/// update prioServingStatus
				grantingType = prioGrantType::greenExtension;
				prioServingStatus.granting_msec = fullTimeStamp.msec;
				prioServingStatus.grantingPhase = signalStatus.synch_phase;
				prioServingStatus.grantingVehId = greenExtensionCandidates[indx].vehId;
				prioServingStatus.grantingCycleCnt = awareStatus.cycleCtn;
				/// update status of priority request for greenExtensionCandidates
				setGrantStatus(srmList, greenExtensionCandidates);
				awareStatus.requestStatusUpdated = true;
			}
			else if (!earlyGreenCandidates.empty())
			{ /// find the index in greenExtensionCandidates that having maximum duration
				size_t indx = getCandidateIndex(earlyGreenCandidates);
				/// update prioServingStatus
				grantingType = prioGrantType::earlyGreen;
				prioServingStatus.granting_msec = fullTimeStamp.msec;
				prioServingStatus.grantingPhase = earlyGreenCandidates[indx].requestedPhase;
				prioServingStatus.grantingVehId = earlyGreenCandidates[indx].vehId;
				prioServingStatus.grantingCycleCnt = awareStatus.cycleCtn;
			}
			if (grantingType != prioGrantType::none)
			{ /// send priority request to MRP_DataMgr
				size_t msgSize = packMsg(sendbuf, prioServingStatus.grantingPhase, MsgEnum::softCallObj::priority,
					(grantingType == prioGrantType::greenExtension) ? MsgEnum::softCallType::extension : MsgEnum::softCallType::call,
					fullTimeStamp.localDateTimeStamp.msOfDay, msgUtils::msgid_softcall);
				socketUtils::sendall(sendConn, &sendbuf[0], msgSize);
				if (log_type != logUtils::logType::none)
					logUtils::logMsg(logFiles, std::string("req"), sendbuf, msgSize);
				OS_Display << fullTimeStamp.localDateTimeStamp.to_dateTimeStr('-', ':');
				if (grantingType == prioGrantType::greenExtension)
					OS_Display << ", call greenExtension on phase ";
				else
					OS_Display << ", call earlyGreen on phase ";
				OS_Display << static_cast<unsigned int>(prioServingStatus.grantingPhase) << std::endl;
			}
		}

		/// check vehicle phase call, place a vehicular phase call when
		///   1) vehicle in onInbound and has not requested a phase call
		///   2) phase that controls vehicle's movement is permitted, not in green
		///   3) phase has not been called and not set for recall
		///   4) vehicle's travel time to the stop-bar is within maxTime2goPhaseCall (20 seconds)
		std::bitset<8> phases2call;
		for (auto& cv : vehList)
		{ /// loop through all BSMs to get every vehicular phase that should be called
			if (cv.isOnInbound && !cv.isPhaseCalled)
			{ /// get the latest result of locating vehicle on MAP
				const auto& cvStatus = cv.cvStatus.back();
				uint16_t time2go = getTime2Go(cvStatus.vehicleLocationAware.dist2go.distLong, cvStatus.motionState.speed, stopSpeed);
				uint8_t controlPhase = (uint8_t)(cvStatus.vehicleLocationAware.controlPhase - 1);
				bool phaseInGreen = ((cvStatus.vehicleSignalAware.currState == MsgEnum::phaseState::permissiveGreen)
					|| (cvStatus.vehicleSignalAware.currState == MsgEnum::phaseState::protectedGreen));
				auto& phasecall_msec = awareStatus.phaseCall_mec[controlPhase];
				if (!phaseInGreen && spat.permittedPhases.test(controlPhase)
					&& !signalStatus.veh_call.test(controlPhase)
					&& (signalStatus.call_status[controlPhase] == MsgEnum::phaseCallType::none)
					&& (signalStatus.recall_status[controlPhase] == MsgEnum::phaseRecallType::none)
					&& (time2go < maxTime2goPhaseCall) && (fullTimeStamp.msec > phasecall_msec + vehPhaseCallInterval))
				{
					phases2call.set(controlPhase);
					cv.isPhaseCalled = true;
					phasecall_msec = fullTimeStamp.msec;
				}
			}
		}
		if (phases2call.any())
		{	/// send vehicle phase call to MRP_DataMgr
			size_t msgSize = packMsg(sendbuf, phases2call, MsgEnum::softCallObj::vehicle, MsgEnum::softCallType::call,
				fullTimeStamp.localDateTimeStamp.msOfDay, msgUtils::msgid_softcall);
			socketUtils::sendall(sendConn, &sendbuf[0], msgSize);
			if (log_type != logUtils::logType::none)
				logUtils::logMsg(logFiles, std::string("req"), sendbuf, msgSize);
			OS_Display << fullTimeStamp.localDateTimeStamp.to_dateTimeStr('-', ':');
			OS_Display << ", call vehicle phases " << phases2call.to_string() << std::endl;
			phases2call.reset();
		}

		/// check cancel of on-going non-TSP phase extension
		for (int i = 0; i < 8; i++)
		{ /// check phases that have been called phase extension
			auto& phaseExtStatus = awareStatus.phaseExtStatus[i];
			if (phaseExtStatus.extType != phaseExtType::called)
				continue;
			const auto& phaseState = spat.phaseState[i];
			bool phaseInGreen = ((phaseState.currState == MsgEnum::phaseState::permissiveGreen)
				|| (phaseState.currState == MsgEnum::phaseState::protectedGreen));
			if (!phaseInGreen)
			{ /// green on extension phase expired
				phaseExtStatus.extType = phaseExtType::cancelled;
				phaseExtStatus.servingVehIds.clear();
			}
			else
			{	/// remove vehicle from servingVehIds if the vehicle is no long onInbound
				auto& servingVehIds = phaseExtStatus.servingVehIds;
				servingVehIds.erase(std::remove_if(servingVehIds.begin(), servingVehIds.end(), [&vehList](uint32_t& vehId)->bool
					{	auto it = std::find_if(vehList.begin(), vehList.end(), [&vehId](cvStatusAware_t& obj){return(obj.bsm.id == vehId);});
						return((it == vehList.end()) || !it->isOnInbound);}), servingVehIds.end());
				if (phaseInGreen  && servingVehIds.empty())
				{
					phases2call.set(i);
					phaseExtStatus.extType = phaseExtType::none;
				}
			}
		}
		if (phases2call.any())
		{	/// send cancel phase extension request to MRP_DataMgr
			size_t msgSize = packMsg(sendbuf, phases2call, MsgEnum::softCallObj::vehicle, MsgEnum::softCallType::cancel,
				fullTimeStamp.localDateTimeStamp.msOfDay, msgUtils::msgid_softcall);
			socketUtils::sendall(sendConn, &sendbuf[0], msgSize);
			if (log_type != logUtils::logType::none)
				logUtils::logMsg(logFiles, std::string("req"), sendbuf, msgSize);
			OS_Display << fullTimeStamp.localDateTimeStamp.to_dateTimeStr('-', ':');
			OS_Display << ", cancel vehicle extension on phases " << phases2call.to_string() << std::endl;
			phases2call.reset();
		}

		/// check initializing non-TSP phase extension call:
		///   1) controller is not serving a priority treatment
		///   2) vehicle in onInbound and has not requested a phase extension
		///   3) phase that controls vehicle's movement is permitted and has not be called phase extension
		///   4) signal is neither red-flashing nor preemption
		///   5) when under coordinated control, phase is not a coordinated phase
		///   6) phase is green, and expected time2arrival is within allowable green extension window
		///   7) time-to-green end is within maxTime2changePhaseExt
		if (grantingType == prioGrantType::none)
		{
			for (auto& cv : vehList)
			{ /// loop through all BSMs to get every vehicular phase that could be extended
				if (cv.isOnInbound && !cv.isExtensionCalled)
				{ /// get the latest result of locating vehicle on MAP
					const auto& cvStatus = cv.cvStatus.back();
					uint16_t time2go = getTime2Go(cvStatus.vehicleLocationAware.dist2go.distLong, cvStatus.motionState.speed, stopSpeed);
					uint16_t time2arrival = (uint16_t)((fullTimeStamp.msec / 100 + time2go) % 36000);
					uint16_t curr_time = (uint16_t)(fullTimeStamp.msec / 100 % 36000);
					uint8_t controlPhase = (uint8_t)(cvStatus.vehicleLocationAware.controlPhase - 1);
					const auto& phaseState = spat.phaseState[controlPhase];
					bool phaseInGreen = ((phaseState.currState == MsgEnum::phaseState::permissiveGreen)
						|| (phaseState.currState == MsgEnum::phaseState::protectedGreen));
					auto& phaseExtStatus = awareStatus.phaseExtStatus[controlPhase];
					if (spat.permittedPhases.test(controlPhase) && (phaseExtStatus.extType != phaseExtType::called)
						&& (signalStatus.mode != MsgEnum::controlMode::flashing) && (signalStatus.mode != MsgEnum::controlMode::preemption)
						&& ((signalStatus.mode != MsgEnum::controlMode::coordination) || !signalStatus.coordinated_phases.test(controlPhase))
						&& phaseInGreen && withinTimeWindow(phaseState.minEndTime, maxTime2phaseExt, time2arrival)
						&& (getDuration(curr_time, phaseState.minEndTime) < maxTime2changePhaseExt))
					{
						phases2call.set(controlPhase);
						cv.isExtensionCalled = true;
						if (phaseExtStatus.extType != phaseExtType::called)
						{
							phaseExtStatus.extType = phaseExtType::called;
							phaseExtStatus.extCall_msec = fullTimeStamp.msec;
						}
						phaseExtStatus.servingVehIds.push_back(cv.bsm.id);
					}
				}
			}
		}
		if (phases2call.any())
		{ /// send non-TSP phase extension request to MRP_DataMgr
			size_t msgSize = packMsg(sendbuf, phases2call, MsgEnum::softCallObj::vehicle, MsgEnum::softCallType::extension,
				fullTimeStamp.localDateTimeStamp.msOfDay, msgUtils::msgid_softcall);
			socketUtils::sendall(sendConn, &sendbuf[0], msgSize);
			if (log_type != logUtils::logType::none)
				logUtils::logMsg(logFiles, std::string("req"), sendbuf, msgSize);
			OS_Display << fullTimeStamp.localDateTimeStamp.to_dateTimeStr('-', ':');
			OS_Display << ", call non-TSP extension on phases " << phases2call.to_string() << std::endl;
			phases2call.reset();
		}

		/// check whether need to send SSM
		if ((fullTimeStamp.msec > sentSSM_msec +  MsgEnum::ssmInterval) && !srmList.empty())
		{	/// pack new SSM only when priority list got updated
			if (awareStatus.requestStatusUpdated)
				packMsg(ssm, srmList, fullTimeStamp.utcDateTimeStamp);
			/// reset update flag
			awareStatus.requestStatusUpdated = false;
			sentSSM_msec = fullTimeStamp.msec;
			/// encode SSM
			ssize_t payload_size = AsnJ2735Lib::encode_ssm_payload(ssm, &sendbuf[9], bufSize);
			if (payload_size > 0)
			{ /// add MMITSS header and send to MRP_DataMgr
				size_t offset = 0;
				msgUtils::packHeader(sendbuf, offset, msgUtils::msgid_ssm, fullTimeStamp.localDateTimeStamp.msOfDay, (uint16_t)payload_size);
				socketUtils::sendall(sendConn, &sendbuf[0], (size_t)payload_size + offset);
				if (log_type == logUtils::logType::detailLog)
					logUtils::logMsg(logFiles, std::string("payload"), sendbuf, (size_t)payload_size + offset);
			}
			else
			{
				OS_ERR << fullTimeStamp.localDateTimeStamp.to_dateTimeStr('-', ':');
				OS_ERR << ", failed encode_spat_payload" << std::endl;
			}
		}

		/// remove outdated entries from vehList & srmList
		vehList.erase(std::remove_if(vehList.begin(), vehList.end(), [&fullTimeStamp, &timeouInterval](cvStatusAware_t& obj)
			{return(obj.msec > fullTimeStamp.msec + timeouInterval);}), vehList.end());
		srmList.erase(std::remove_if(srmList.begin(), srmList.end(), [&fullTimeStamp, &timeouInterval](srmStatus_t& obj)
			{return(obj.msec > fullTimeStamp.msec + timeouInterval);}), srmList.end());

		/// check reopen log files
		if ((log_type != logUtils::logType::none) && (fullTimeStamp.msec > logfile_msec + logInterval))
		{
			logUtils::reOpenLogFiles(logFiles, fullTimeStamp.localDateTimeStamp.to_fileName());
			logfile_msec = fullTimeStamp.msec;
		}

		/// check reopen display log file
		if (fullTimeStamp.msec > displayfile_msec + displayLogInterval)
		{
			OS_Display.close();
			OS_Display.open(displayLog);
			displayfile_msec = fullTimeStamp.msec;
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(5));
	}
	/// exit
	timeUtils::getFullTimeStamp(fullTimeStamp);
	OS_ERR << fullTimeStamp.localDateTimeStamp.to_dateTimeStr('-', ':');
	OS_ERR << ", received user termination signal " << terminate << ", exit!" << std::endl;
	OS_Display << fullTimeStamp.localDateTimeStamp.to_dateTimeStr('-', ':');
	OS_Display << ", received user termination signal " << terminate << ", exit!" << std::endl;
	OS_ERR.close();
	OS_Display.close();
	if (log_type != logUtils::logType::none)
		logUtils::closeLogFiles(logFiles);
	pmycnf->disconnectAll();
	delete pmycnf;
	delete plocAwareLib;
	return(0);
}

size_t packMsg(std::vector<uint8_t>& buf, uint8_t callPhase, MsgEnum::softCallObj callObj,
	MsgEnum::softCallType callType, uint32_t msOfDay, uint8_t msgid)
{
	msgDefs::softcall_request_t softReq{msOfDay, std::bitset<8>(1 << (callPhase - 1)), callObj, callType};
	return(msgDefs::packMsg(buf, softReq, msgid));
}

size_t packMsg(std::vector<uint8_t>& buf, std::bitset<8> callPhases, MsgEnum::softCallObj callObj,
	MsgEnum::softCallType callType, uint32_t msOfDay, uint8_t msgid)
{
	msgDefs::softcall_request_t softReq{msOfDay, callPhases, callObj, callType};
	return(msgDefs::packMsg(buf, softReq, msgid));
}

size_t packMsg(std::vector<uint8_t>& buf, const cvStatusAware_t& cvStatusAware, const timeUtils::dateStamp_t& curDateStamp,
	uint32_t msOfDay, uint32_t laneLen, double stopSpeed, uint8_t msgid)
{
	static uint32_t trojCnt = 0;
	static timeUtils::dateStamp_t dateStamp{0, 0, 0};
	if (curDateStamp != dateStamp)
	{
		trojCnt = 0;
		dateStamp = curDateStamp;
	}
	trojCnt++;
	msgDefs::vehTraj_t vehTraj;
	const auto& cvStatus = cvStatusAware.cvStatus;
	vehTraj.ms_since_midnight = msOfDay;
	vehTraj.count = trojCnt;
	vehTraj.vehId = cvStatus[0].id;
	vehTraj.entryLaneId = cvStatus[0].vehicleLocationAware.laneId;
	vehTraj.entryControlPhase = cvStatus[0].vehicleLocationAware.controlPhase;
	vehTraj.leaveLaneId = cvStatus.back().vehicleLocationAware.laneId;
	vehTraj.leaveControlPhase = cvStatus.back().vehicleLocationAware.controlPhase;
	vehTraj.distTraveled = static_cast<uint16_t>((cvStatus[0].vehicleLocationAware.dist2go.distLong
		- cvStatus.back().vehicleLocationAware.dist2go.distLong) * 10);
	vehTraj.timeTraveled = static_cast<uint16_t>((cvStatus.back().msec - cvStatus[0].msec) / 100);
	size_t stoppedCnt = std::count_if(cvStatus.begin(), cvStatus.end(), [&stopSpeed](const GeoUtils::connectedVehicle_t& item)
		{return(item.motionState.speed <= stopSpeed);});
	vehTraj.stoppedTime = static_cast<uint16_t>(stoppedCnt);
	vehTraj.inboundLaneLen = static_cast<uint16_t>(laneLen / 10);
	return(msgDefs::packMsg(buf, vehTraj, msgid));
}

void packMsg(SSM_element_t& ssm, const std::vector<srmStatus_t> list, const timeUtils::dateTimeStamp_t utcDateTimeStamp)
{
	static uint8_t msgCnt = 0;
	static uint8_t updateCnt = 0;
	msgCnt = (uint8_t)((msgCnt + 1) % 127);
	updateCnt = (uint8_t)((updateCnt + 1) % 127);
	ssm.timeStampMinute = utcDateTimeStamp.minuteOfYear;
	ssm.timeStampSec = utcDateTimeStamp.msOfMinute;
	ssm.msgCnt = msgCnt;
	ssm.updateCnt	= updateCnt;
	ssm.mpSignalRequetStatus.resize(list.size());
	int indx = 0;
	for (const auto& item : list)
	{
		const auto& srm = item.srm;
		SignalRequetStatus_t SignalRequetStatus{srm.vehId, srm.reqId, srm.msgCnt,
			srm.inApprochId, srm.inLaneId, srm.outApproachId, srm.outLaneId, srm.ETAminute,
			srm.ETAsec, srm.duration, srm.vehRole, item.status};
		ssm.mpSignalRequetStatus[indx++] = SignalRequetStatus;
	}
}

uint16_t getTime2Go(double dist2go, double speed, double stopSpeed)
{
	double speed2go = (speed > stopSpeed) ? speed : stopSpeed;
	return((uint16_t)(fabs(dist2go) / speed2go * 10));
}

bool withinTimeWindow(uint16_t windowStartTime, uint16_t windowLength, uint16_t arrivalTime)
{ /// windowStartTime, windowEndTime and arrivalTime are time points in tenths of a second
	/// in the current or next hour. return whether arrivalTime is within the given time window
	uint16_t windowEndTime = (uint16_t)((windowStartTime + windowLength) % 36000);
	if (windowEndTime > windowStartTime) /// time window does not crossing the hour
		return((arrivalTime >= windowStartTime) && (arrivalTime <= windowEndTime));
	else
		return((arrivalTime >= windowStartTime) || (arrivalTime <= windowEndTime));
}

bool isTimeBefore(uint16_t timePoint_1, uint16_t timePoint_2)
{ /// return TRUE if timePoint_1 is before timePoint_2.
	/// timePoint_1 and timePoint_2 are tenths of a second in the current or next hour.
	/// determination of timePoint_1 prior to timePoint_2 needs to consider the crossing-hour case.
	if (timePoint_1 == timePoint_2)
		return(false);
	uint16_t fromP1_to_P2, fromP2_to_P1;
	if (timePoint_1 < timePoint_2)
	{
		fromP1_to_P2 = (uint16_t)(timePoint_2 - timePoint_1);
		fromP2_to_P1 = (uint16_t)(36000 + timePoint_1 - timePoint_2);
	}
	else
	{
		fromP1_to_P2 = (uint16_t)(36000 + timePoint_2 - timePoint_1);
		fromP2_to_P1 = (uint16_t)(timePoint_1 - timePoint_2);
	}
	return(fromP1_to_P2 < fromP2_to_P1);
}

uint16_t getDuration(uint16_t fromTimePoint, uint16_t toTimePoint)
{ /// fromTimePoint and toTimePoint are tenths of a second in the current or next hour.
	uint16_t fromP1_to_P2, fromP2_to_P1;
	if (fromTimePoint < toTimePoint)
	{
		fromP1_to_P2 = (uint16_t)(toTimePoint - fromTimePoint);
		fromP2_to_P1 = (uint16_t)(36000 + fromTimePoint - toTimePoint);
	}
	else
	{
		fromP1_to_P2 = (uint16_t)(36000 + toTimePoint - fromTimePoint);
		fromP2_to_P1 = (uint16_t)(fromTimePoint - toTimePoint);
	}
	return((fromP1_to_P2 < fromP2_to_P1) ? fromP1_to_P2 : fromP2_to_P1);
}

size_t getCandidateIndex(const std::vector<prioRequestCadidate_t>& candidates)
{
	size_t retn = 0;
	uint16_t maxDuration = candidates[0].duration;
	for (size_t i = 1, j = candidates.size(); i < j; i++)
	{
		if (candidates[i].duration > maxDuration)
		{
			maxDuration = candidates[i].duration;
			retn = i;
		}
	}
	return(retn);
}

void setGrantStatus(std::vector<srmStatus_t>& list, const std::vector<prioRequestCadidate_t>& candidates)
{
	for (const auto& item : candidates)
	{
		auto it = std::find_if(list.begin(), list.end(), [&item](srmStatus_t obj){return(obj.srm.vehId == item.vehId);});
		if (it != list.end())
			it->status = MsgEnum::requestStatus::granted;
	}
}
