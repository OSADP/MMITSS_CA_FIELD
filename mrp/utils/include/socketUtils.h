//********************************************************************************************************
//
// © 2016 Regents of the University of California on behalf of the University of California at Berkeley
//       with rights granted for USDOT OSADP distribution with the ECL-2.0 open source license.
//
//*********************************************************************************************************
#ifndef _SOCKET_UTILS_H
#define _SOCKET_UTILS_H

#include <cstddef>
#include <cstdint>
#include <string>
#include <sys/socket.h>

namespace socketUtils
{
	struct Conn_t
	{
		int fd;
		int socktype;
		struct sockaddr ai_addr;
		socklen_t ai_addrlen;
	};

	struct Address_t
	{
		std::string id;       // destination OR source
		std::string role;     // server OR client
		std::string protocol; // UDP OR TCP
		std::string name;     // hostname OR IP address
		std::string port;     // port number
		socketUtils::Conn_t conn;
		bool valid(void)
		{
			return(!id.empty() && !role.empty() && !protocol.empty() && !name.empty()
				&& !port.empty() && ((protocol == "UDP") || (protocol == "TCP")));
		};
		void reset(void)
		{
			conn.fd = - 1;
			id.clear();
			role.clear();
			protocol.clear();
			name.clear();
			port.clear();
		};
	};

	bool setAddress(const std::string& str, socketUtils::Address_t& addr);
	bool create(socketUtils::Address_t& addr);
	bool sendall(socketUtils::Conn_t conn, const uint8_t* buf, size_t len);
	bool destroy(int fid);
}

#endif
