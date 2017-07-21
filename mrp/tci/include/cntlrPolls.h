//********************************************************************************************************
//
// © 2016 Regents of the University of California on behalf of the University of California at Berkeley
//       with rights granted for USDOT OSADP distribution with the ECL-2.0 open source license.
//
//*********************************************************************************************************
#ifndef _CONTROLLER_POLL_H
#define _CONTROLLER_POLL_H

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

class Polls
{
	public:
		/// polls to get controller configuration data
		struct poll_conf_t
		{
			uint8_t poll_type;      // 1/2 - getBlockMsg/getTimingData
			std::string poll_desc;  // unique string per poll
			uint8_t poll_controlByte;
			uint8_t poll_messType;
			uint8_t poll_data1;     // pageId for getBlockMsg, memory_msb for getTimingData
			uint8_t poll_data2;     // blockId for getBlockMsg, memory_lsb for getTimingData
			uint8_t poll_data3;     // not used for getBlockMsg, num_bytes for getTimingData
			uint8_t res_messType;
			uint8_t err_messType;
			uint8_t res_size;
			bool    pollRequired;
			bool    fcsRequired;
			bool    pollReturned;
		};

		/// trace and manage controller polls
		struct poll_trace_t
		{
			size_t index;   // index in poll_list
			Polls::poll_conf_t* pcurPoll;
			int  maxpolls;  // maximum number of polls per request
			int  numspolled;
			int  cyclenums;
			unsigned long long poll_msec;
			unsigned long long poll_interval;
			void set(int maxpolls_, unsigned long long poll_interval_)
			{
				index = 0;
				pcurPoll = nullptr;
				cyclenums = 0;
				numspolled = 0;
				poll_msec = 0;
				maxpolls = maxpolls_;
				poll_interval = poll_interval_;
			};
		};

	private:
		Polls::poll_trace_t poll_trace;
		std::vector<Polls::poll_conf_t> poll_list = {
			{2, std::string("red revert"),          0x33, 0x89, 0x72, 0x00, 1, 0xC9, 0xE9, 12, true, true, false},
			{1, std::string("phase flags"),         0x33, 0x87, 2,  1, 0, 0xC7, 0xE7, 31, true, true, false},
			{1, std::string("ped flags"),           0x33, 0x87, 2,  3, 0, 0xC7, 0xE7, 18, true, true, false},
			{1, std::string("phase timing 1"),      0x33, 0x87, 3,  1, 0, 0xC7, 0xE7, 31, true, true, false},
			{1, std::string("phase timing 2"),      0x33, 0x87, 3,  2, 0, 0xC7, 0xE7, 31, true, true, false},
			{1, std::string("phase timing 3"),      0x33, 0x87, 3,  3, 0, 0xC7, 0xE7, 31, true, true, false},
			{1, std::string("phase timing 4"),      0x33, 0x87, 3,  4, 0, 0xC7, 0xE7, 31, true, true, false},
			{1, std::string("phase timing 5"),      0x33, 0x87, 3,  5, 0, 0xC7, 0xE7, 31, true, true, false},
			{1, std::string("phase timing 6"),      0x33, 0x87, 3,  6, 0, 0xC7, 0xE7, 31, true, true, false},
			{1, std::string("phase timing 7"),      0x33, 0x87, 3,  7, 0, 0xC7, 0xE7, 31, true, true, false},
			{1, std::string("phase timing 8"),      0x33, 0x87, 3,  8, 0, 0xC7, 0xE7, 31, true, true, false},
			{1, std::string("coord plan 1"),        0x33, 0x87, 4,  1, 0, 0xC7, 0xE7, 36, true, true, false},
			{1, std::string("coord plan 2"),        0x33, 0x87, 4,  2, 0, 0xC7, 0xE7, 36, true, true, false},
			{1, std::string("coord plan 3"),        0x33, 0x87, 4,  3, 0, 0xC7, 0xE7, 36, true, true, false},
			{1, std::string("coord plan 4"),        0x33, 0x87, 4,  4, 0, 0xC7, 0xE7, 36, true, true, false},
			{1, std::string("coord plan 5"),        0x33, 0x87, 4,  5, 0, 0xC7, 0xE7, 36, true, true, false},
			{1, std::string("coord plan 6"),        0x33, 0x87, 4,  6, 0, 0xC7, 0xE7, 36, true, true, false},
			{1, std::string("coord plan 7"),        0x33, 0x87, 4,  7, 0, 0xC7, 0xE7, 36, true, true, false},
			{1, std::string("coord plan 8"),        0x33, 0x87, 4,  8, 0, 0xC7, 0xE7, 36, true, true, false},
			{1, std::string("coord plan 9"),        0x33, 0x87, 4,  9, 0, 0xC7, 0xE7, 36, true, true, false},
			{1, std::string("free plan"),           0x33, 0x87, 4, 10, 0, 0xC7, 0xE7, 28, true, true, false},
			{1, std::string("coord plan 11"),       0x33, 0x87, 5,  1, 0, 0xC7, 0xE7, 36, false, true, false},
			{1, std::string("coord plan 12"),       0x33, 0x87, 5,  2, 0, 0xC7, 0xE7, 36, false, true, false},
			{1, std::string("coord plan 13"),       0x33, 0x87, 5,  3, 0, 0xC7, 0xE7, 36, false, true, false},
			{1, std::string("coord plan 14"),       0x33, 0x87, 5,  4, 0, 0xC7, 0xE7, 36, false, true, false},
			{1, std::string("coord plan 15"),       0x33, 0x87, 5,  5, 0, 0xC7, 0xE7, 36, false, true, false},
			{1, std::string("coord plan 16"),       0x33, 0x87, 5,  6, 0, 0xC7, 0xE7, 36, false, true, false},
			{1, std::string("coord plan 17"),       0x33, 0x87, 5,  7, 0, 0xC7, 0xE7, 36, false, true, false},
			{1, std::string("coord plan 18"),       0x33, 0x87, 5,  8, 0, 0xC7, 0xE7, 36, false, true, false},
			{1, std::string("coord plan 19"),       0x33, 0x87, 5,  9, 0, 0xC7, 0xE7, 36, false, true, false},
			{1, std::string("coord plan 21"),       0x33, 0x87, 6,  1, 0, 0xC7, 0xE7, 36, false, true, false},
			{1, std::string("coord plan 22"),       0x33, 0x87, 6,  2, 0, 0xC7, 0xE7, 36, false, true, false},
			{1, std::string("coord plan 23"),       0x33, 0x87, 6,  3, 0, 0xC7, 0xE7, 36, false, true, false},
			{1, std::string("coord plan 24"),       0x33, 0x87, 6,  4, 0, 0xC7, 0xE7, 36, false, true, false},
			{1, std::string("coord plan 25"),       0x33, 0x87, 6,  5, 0, 0xC7, 0xE7, 36, false, true, false},
			{1, std::string("coord plan 26"),       0x33, 0x87, 6,  6, 0, 0xC7, 0xE7, 36, false, true, false},
			{1, std::string("coord plan 27"),       0x33, 0x87, 6,  7, 0, 0xC7, 0xE7, 36, false, true, false},
			{1, std::string("coord plan 28"),       0x33, 0x87, 6,  8, 0, 0xC7, 0xE7, 36, false, true, false},
			{1, std::string("coord plan 29"),       0x33, 0x87, 6,  9, 0, 0xC7, 0xE7, 36, false, true, false},
			{1, std::string("detector group 1"),    0x33, 0x87, 7,  1, 0, 0xC7, 0xE7, 38, false, true, false},
			{1, std::string("detector group 2"),    0x33, 0x87, 7,  2, 0, 0xC7, 0xE7, 38, false, true, false},
			{1, std::string("detector group 3"),    0x33, 0x87, 7,  3, 0, 0xC7, 0xE7, 38, false, true, false},
			{1, std::string("detector group 4"),    0x33, 0x87, 7,  4, 0, 0xC7, 0xE7, 38, false, true, false},
			{1, std::string("detector group 5"),    0x33, 0x87, 7,  5, 0, 0xC7, 0xE7, 38, false, true, false},
			{1, std::string("detector group 6"),    0x33, 0x87, 7,  6, 0, 0xC7, 0xE7, 38, false, true, false},
			{1, std::string("detector group 7"),    0x33, 0x87, 7,  7, 0, 0xC7, 0xE7, 38, false, true, false},
			{1, std::string("detector group 8"),    0x33, 0x87, 7,  8, 0, 0xC7, 0xE7, 38, false, true, false},
			{1, std::string("detector group 9"),    0x33, 0x87, 7,  9, 0, 0xC7, 0xE7, 38, false, true, false},
			{1, std::string("detector group 10"),   0x33, 0x87, 7, 10, 0, 0xC7, 0xE7, 38, false, true, false},
			{1, std::string("detector group 11"),   0x33, 0x87, 7, 11, 0, 0xC7, 0xE7, 38, false, true, false},
			{1, std::string("system detector"),     0x33, 0x87, 7, 12, 0, 0xC7, 0xE7, 34, false, true, false},
			{1, std::string("CIC plan"),            0x33, 0x87, 7, 13, 0, 0xC7, 0xE7, 35, false, true, false},
			{1, std::string("TOD table 1"),         0x33, 0x87, 8,  1, 0, 0xC7, 0xE7, 26, false, true, false},
			{1, std::string("TOD table 2"),         0x33, 0x87, 8,  2, 0, 0xC7, 0xE7, 26, false, true, false},
			{1, std::string("TOD table 3"),         0x33, 0x87, 8,  3, 0, 0xC7, 0xE7, 26, false, true, false},
			{1, std::string("TOD table 4"),         0x33, 0x87, 8,  4, 0, 0xC7, 0xE7, 26, false, true, false},
			{1, std::string("TOD table 5"),         0x33, 0x87, 8,  5, 0, 0xC7, 0xE7, 26, false, true, false},
			{1, std::string("TOD table 6"),         0x33, 0x87, 8,  6, 0, 0xC7, 0xE7, 26, false, true, false},
			{1, std::string("TOD table 7"),         0x33, 0x87, 8,  7, 0, 0xC7, 0xE7, 26, false, true, false},
			{1, std::string("TOD table 8"),         0x33, 0x87, 8,  8, 0, 0xC7, 0xE7, 26, false, true, false},
			{1, std::string("TOD table 9"),         0x33, 0x87, 8,  9, 0, 0xC7, 0xE7, 26, false, true, false},
			{1, std::string("TOD table 10"),        0x33, 0x87, 8, 10, 0, 0xC7, 0xE7, 26, false, true, false},
			{1, std::string("TOD table 11"),        0x33, 0x87, 8, 11, 0, 0xC7, 0xE7, 26, false, true, false},
			{1, std::string("TOD table 12"),        0x33, 0x87, 8, 12, 0, 0xC7, 0xE7, 26, false, true, false},
			{1, std::string("TOD table 13"),        0x33, 0x87, 8, 13, 0, 0xC7, 0xE7, 26, false, true, false},
			{1, std::string("TOD table 14"),        0x33, 0x87, 8, 14, 0, 0xC7, 0xE7, 26, false, true, false},
			{1, std::string("TOD table 15"),        0x33, 0x87, 8, 15, 0, 0xC7, 0xE7, 26, false, true, false},
			{1, std::string("TOD table 16"),        0x33, 0x87, 8, 16, 0, 0xC7, 0xE7, 26, false, true, false},
			{1, std::string("TOD table 17"),        0x33, 0x87, 8, 17, 0, 0xC7, 0xE7, 26, false, true, false},
			{1, std::string("TOD table 18"),        0x33, 0x87, 8, 18, 0, 0xC7, 0xE7, 26, false, true, false},
			{1, std::string("TOD table 19"),        0x33, 0x87, 8, 19, 0, 0xC7, 0xE7, 26, false, true, false},
			{1, std::string("TOD table 20"),        0x33, 0x87, 8, 20, 0, 0xC7, 0xE7, 26, false, true, false},
			{1, std::string("TOD table 21"),        0x33, 0x87, 8, 21, 0, 0xC7, 0xE7, 26, false, true, false},
			{1, std::string("TOD table 22"),        0x33, 0x87, 8, 22, 0, 0xC7, 0xE7, 26, false, true, false},
			{1, std::string("TOD table 23"),        0x33, 0x87, 8, 23, 0, 0xC7, 0xE7, 26, false, true, false},
			{1, std::string("TOD table 24"),        0x33, 0x87, 8, 24, 0, 0xC7, 0xE7, 26, false, true, false},
			{1, std::string("weekday"),             0x33, 0x87, 8, 25, 0, 0xC7, 0xE7, 17, false, true, false},
			{1, std::string("TOD Function 1"),      0x33, 0x87, 9,  6, 0, 0xC7, 0xE7, 38, false, true, false},
			{1, std::string("TOD Function 2"),      0x33, 0x87, 9,  7, 0, 0xC7, 0xE7, 38, false, true, false},
			{1, std::string("TOD Function 3"),      0x33, 0x87, 9,  8, 0, 0xC7, 0xE7, 38, false, true, false},
			{1, std::string("TOD Function 4"),      0x33, 0x87, 9,  9, 0, 0xC7, 0xE7, 38, false, true, false},
			{1, std::string("RR1 phase flags"),     0x33, 0x87, 11, 1, 0, 0xC7, 0xE7, 22, false, true, false},
			{1, std::string("RR1 ped flags"),       0x33, 0x87, 11, 2, 0, 0xC7, 0xE7, 22, false, true, false},
			{1, std::string("RR1 overlap flags"),   0x33, 0x87, 11, 3, 0, 0xC7, 0xE7, 22, false, true, false},
			{1, std::string("RR1 exit parameters"), 0x33, 0x87, 11, 4, 0, 0xC7, 0xE7, 14, false, true, false},
			{1, std::string("RR1 Configuration"),   0x33, 0x87, 11, 5, 0, 0xC7, 0xE7, 22, false, true, false},
			{1, std::string("RR2 phase flags"),     0x33, 0x87, 11, 6, 0, 0xC7, 0xE7, 22, false, true, false},
			{1, std::string("RR2 ped flags"),       0x33, 0x87, 11, 7, 0, 0xC7, 0xE7, 22, false, true, false},
			{1, std::string("RR2 overlap flags"),   0x33, 0x87, 11, 8, 0, 0xC7, 0xE7, 22, false, true, false},
			{1, std::string("RR2 exit parameters"), 0x33, 0x87, 11, 9, 0, 0xC7, 0xE7, 14, false, true, false},
			{1, std::string("RR2 Configuration"),   0x33, 0x87, 11,10, 0, 0xC7, 0xE7, 22, false, true, false},
			{1, std::string("EVA"),                 0x33, 0x87, 11,11, 0, 0xC7, 0xE7, 18, true,  true, false},
			{1, std::string("EVB"),                 0x33, 0x87, 11,12, 0, 0xC7, 0xE7, 18, true,  true, false},
			{1, std::string("EVC"),                 0x33, 0x87, 11,13, 0, 0xC7, 0xE7, 18, true,  true, false},
			{1, std::string("EVD"),                 0x33, 0x87, 11,14, 0, 0xC7, 0xE7, 18, true,  true, false},
			{1, std::string("TSP plan group 1"),    0x33, 0x87, 13, 2, 0, 0xC7, 0xE7, 43, true,  true, false}, // plan 1 - 3
			{1, std::string("TSP plan group 2"),    0x33, 0x87, 13, 3, 0, 0xC7, 0xE7, 43, true,  true, false}, // plan 4 - 6
			{1, std::string("TSP plan group 3"),    0x33, 0x87, 13, 4, 0, 0xC7, 0xE7, 43, true,  true, false}, // plan 7 - 9
			{1, std::string("TSP plan group 4"),    0x33, 0x87, 13, 5, 0, 0xC7, 0xE7, 43, false, true, false}, // plan 11 - 13
			{1, std::string("TSP plan group 5"),    0x33, 0x87, 13, 6, 0, 0xC7, 0xE7, 43, false, true, false}, // plan 14 - 16
			{1, std::string("TSP plan group 6"),    0x33, 0x87, 13, 7, 0, 0xC7, 0xE7, 43, false, true, false}, // plan 17 - 19
			{1, std::string("TSP enable plans"),    0x33, 0x87, 13, 8, 0, 0xC7, 0xE7, 28, true,  true, false}
		};

	public:
		Polls(int maxpolls_per_request, unsigned long long poll_per_interval);
		~Polls(void){};

		void getNextPoll(bool fromStart=false);
		void resetPollReturn(void);
		bool setPollReturn(void);
		void setPollReturn(const std::string& poll_desc);
		void resetPlanPolls(void);
		bool sendPoll(unsigned long long msec);

		bool nextPoll(void) const;
		bool atEnd(void) const;
		bool allReturned(void) const;
		size_t packRequest(std::vector<uint8_t>& buf, uint8_t addr) const;
		std::string getPollDesc(void) const;
		std::string getPollDesc(const uint8_t* pResp, uint8_t messType) const;
		std::string getPollDesc(const uint8_t* pResp, uint8_t messType, size_t frameSize, bool fcsValidated) const;
};

#endif
