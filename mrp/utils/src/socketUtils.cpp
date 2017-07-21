//********************************************************************************************************
//
// © 2016 Regents of the University of California on behalf of the University of California at Berkeley
//       with rights granted for USDOT OSADP distribution with the ECL-2.0 open source license.
//
//*********************************************************************************************************
#include <algorithm>
#include <cerrno>
#include <cstring>
#include <iostream>
#include <sstream>
#include <sys/fcntl.h>
#include <sys/types.h>
#include <sys/select.h>
#include <unistd.h>
#include <netdb.h>

#include "socketUtils.h"

bool socketUtils::setAddress(const std::string& str, socketUtils::Address_t& addr)
{ // str formate: destination/source protocol name port
	addr.reset();
	std::istringstream iss(str);
	iss >> std::skipws >> addr.id >> addr.protocol >> addr.name >> addr.port;
	iss.clear();
	addr.role = (str.find("from") == 0) ? std::string("server") : std::string("client");
	if (!addr.valid())
	{
		std::cerr << "setAddress failed" << std::endl;
		addr.reset();
		return(false);
	}
	return(true);
}

bool socketUtils::create(socketUtils::Address_t& addr)
{ // construct the socket address
	struct addrinfo hints, *res, *p;
	int status;
	std::memset(&hints, 0, sizeof(hints));
	hints.ai_family = AF_UNSPEC;
	if (addr.role == "server")
		hints.ai_flags = AI_PASSIVE;
	hints.ai_socktype = (addr.protocol == "TCP") ? SOCK_STREAM : SOCK_DGRAM;
	if ((status = getaddrinfo(addr.name.c_str(), addr.port.c_str(), &hints, &res)) != 0)
	{
		perror("socketUtils.create.getaddrinfo");
		return(false);
	}
	// loop through the result to find the first one works
	int socketfd;
	for(p = res; p != NULL; p = p->ai_next)
	{ // create the socket
		if ((socketfd = socket(p->ai_family, p->ai_socktype, p->ai_protocol)) == -1)
		{
			perror("socketUtils.create.socket");
			continue;
		}
		if (addr.role == "server")
		{	// server role, bind the socket
			int reuseaddr = 1;
			if ((setsockopt(socketfd, SOL_SOCKET, SO_REUSEADDR, &reuseaddr, sizeof(reuseaddr))  == -1)
				|| (bind(socketfd, p->ai_addr, p->ai_addrlen) == -1))
			{
				perror("socketUtils.create.bind");
				close(socketfd);
				continue;
			}
			// set the socket to be non-blocking
			int fileflags = fcntl(socketfd, F_GETFL, 0);
			if ((fileflags == -1) || (fcntl(socketfd, F_SETFL, fileflags|O_NONBLOCK) == -1))
			{
				perror("socketUtils.create.non-blocking");
				close(socketfd);
				continue;
			}
			// set TCP socket to listen for connection
			if ((addr.protocol == "TCP") && (listen(socketfd, 5) == -1))
			{
				perror("socketUtils.create.listen");
				close(socketfd);
				continue;
			}
		}
		else
		{ // client role, connect TCP socket
			if ((addr.protocol == "TCP") && (connect(socketfd, p->ai_addr, p->ai_addrlen)  == -1))
			{
				if (errno != EINPROGRESS)
				{
					perror("socketUtils.create.connect");
					close(socketfd);
					continue;
				}
				// check writability
				fd_set wfds;
				FD_ZERO(&wfds);
				FD_SET(socketfd, &wfds);
				struct timeval timeout = {5, 0};
				if ((select(socketfd+1, NULL, &wfds, NULL, &timeout)  <= 0) || !FD_ISSET(socketfd, &wfds))
				{
					perror("socketUtils.create.select");
					close(socketfd);
					continue;
				}
				// check whether connect() completed successfully
				int optval = 0;
				socklen_t optlen = (socklen_t)sizeof(optval);
				if ((getsockopt(socketfd, SOL_SOCKET, SO_ERROR, &optval, &optlen) == -1) || (optval > 0))
				{
					perror("socketUtils.create.getsockopt");
					close(socketfd);
					continue;
				}
			}
		}
		break;
	}
	if (p == NULL)
		addr.conn.fd = -1;
	else
	{
		addr.conn.fd = socketfd;
		addr.conn.socktype = p->ai_socktype;
		addr.conn.ai_addr = *(p->ai_addr);
		addr.conn.ai_addrlen = p->ai_addrlen;
	}
	freeaddrinfo(res);
	return(addr.conn.fd != -1);
}

bool socketUtils::destroy(int fid)
	{return(close(fid) == 0);}

auto tcpSendAll = [](int fd, const uint8_t* buf, size_t len)->bool
{
	size_t totalSent = 0;
	ssize_t bytesSent = -1;
	while (1)
	{
		if ((bytesSent = send(fd, buf + totalSent, len - totalSent, 0)) == -1)
		{
			perror("socketUtils.sendall");
			break;
		}
		totalSent += (size_t)bytesSent;
		if (totalSent >= len)
			break;
	}
	return(totalSent == len);
};

auto udpSendAll = [](int fd, const uint8_t* buf, size_t len, const struct sockaddr* to, socklen_t tolen)->bool
{
	size_t totalSent = 0;
	ssize_t bytesSent = -1;
	while (1)
	{
		if ((bytesSent = sendto(fd, buf + totalSent, len - totalSent, 0, to, tolen)) == -1)
		{
			perror("socketUtils.sendall");
			break;
		}
		totalSent += (size_t)bytesSent;
		if (totalSent >= len)
			break;
	}
	return(totalSent == len);
};

bool socketUtils::sendall(socketUtils::Conn_t conn, const uint8_t* buf, size_t len)
{
	if ((conn.fd < 0) || (buf == NULL) || (len == 0))
		return(false);
	return ((conn.socktype == SOCK_STREAM) ? tcpSendAll(conn.fd, buf, len)
		: udpSendAll(conn.fd, buf, len, &conn.ai_addr, conn.ai_addrlen));
}
