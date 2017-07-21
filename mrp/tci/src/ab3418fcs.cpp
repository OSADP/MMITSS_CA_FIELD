//********************************************************************************************************
//
// © 2016 Regents of the University of California on behalf of the University of California at Berkeley
//       with rights granted for USDOT OSADP distribution with the ECL-2.0 open source license.
//
//*********************************************************************************************************

#include "ab3418fcs.h"

uint16_t pppfcs(uint16_t fcs, const std::vector<uint8_t>& buf, size_t len)
{ /// 0x7E ...
	for (size_t i = 1; i < len; i++)
		fcs = static_cast<uint16_t>((fcs >> 8) ^ AB3418checksum::fcstab[(fcs ^ (uint16_t)buf[i]) & 0xFF]);
	return(fcs);
}

bool AB3418checksum::processMsg(std::vector<uint8_t>& buf, size_t& frame_size, const std::vector<uint8_t>& msg, const std::pair<size_t, size_t>& frame)
{ /// 0x7E ... 0x7E
	size_t offset = frame.first;
	buf[frame_size++] = msg[offset++];
	while (1)
	{
		if ((frame_size >= buf.size()) || (offset > frame.second))
			return(false);
		if (offset == frame.second)
		{
			buf[frame_size] = msg[offset];
			break;
		}
		else if (msg[offset] == 0x7D)
		{ /// must be either 0x7D5E or 0x7D5D
			if (++offset > frame.second)
				return(false);
			/// convert 0x7D5E to 0x7E
			if (msg[offset] == 0x5E)
			{
				buf[frame_size++] = 0x7E;
				offset++;
			}
			else if (msg[offset] == 0x5D)
			{ /// convert 0x7D5D to 0x7D
				buf[frame_size++] = 0x7D;
				offset++;
			}
			else
				return(false);
		}
		else
			buf[frame_size++] = msg[offset++];
	}
	if (frame_size < 7)
		return(false);
	/// checksum
	uint16_t oldfcs = static_cast<uint16_t>((~(uint16_t)(buf[frame_size - 2] << 8)) | (~(uint16_t)buf[frame_size - 1]));
	uint16_t newfcs = pppfcs(oldfcs, buf, frame_size);
	frame_size++;
	return(newfcs == AB3418checksum::PPPGOODFCS);
}

void AB3418checksum::append_FCS(std::vector<uint8_t>& buf, size_t& len)
{ // 0x7E ...
	uint16_t oldfcs = AB3418checksum::PPPINITFCS;
	uint16_t newfcs = pppfcs(oldfcs, buf, len); /// get FCS
	buf[len++] = (uint8_t)(~newfcs);            /// get ms byte FCS
	buf[len++] = (uint8_t)(~(newfcs >> 8));     /// get ls byte FCS
}

void AB3418checksum::get_byte_stuffing(std::vector<uint8_t>& buf, size_t& len)
{ /// 0x7E ..., replaces 0x7E to 0x7D5E, and replace 0x7D to 0x7D5D
	for(size_t i = 1; i < len; i++)
	{
		if (buf[i] == 0x7E)
		{
			buf[i] = 0x7D;
			buf.insert(buf.begin() + i + 1, 0x5E);
			len++;
			i++;
		}
		if (buf[i] == 0x7D)
		{
			buf.insert(buf.begin() + i + 1, 0x5D);
			len++;
			i++;
		}
	}
}

