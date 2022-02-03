/*
 * Copyright (c) 1995, 1996, 1997 Kungliga Tekniska Högskolan
 * (Royal Institute of Technology, Stockholm, Sweden).
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *      This product includes software developed by the Kungliga Tekniska
 *      Högskolan and its contributors.
 *
 * 4. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include "unix_compat.h"
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#include "lwip/ip_addr.h"

static const char *const h_errlist[] = {
    "Resolver Error 0 (no error)",
    "Unknown host",		/* 1 HOST_NOT_FOUND */
    "Host name lookup failure",	/* 2 TRY_AGAIN */
    "Unknown server error",	/* 3 NO_RECOVERY */
    "No address associated with name", /* 4 NO_ADDRESS */
};

char *
hstrerror(int herr)
{
    if (0 <= herr && herr < (sizeof h_errlist / sizeof h_errlist[0]))
	return (char *) h_errlist[herr];
    else if(herr == -17)
	return "unknown error";
    else
	return "Error number out of range (hstrerror)";
}


int
getnameinfo(const struct sockaddr *sa, socklen_t salen,
		    char *host, size_t hostlen,
			char *serv, size_t servlen, int flags)
{
	 if (flags & ~(NI_NUMERICHOST | NI_NUMERICSERV)) {
		return EAI_BADFLAGS;
	}

	const struct sockaddr_in *sinp = (const struct sockaddr_in *) sa;

	switch (sa->sa_family) {
	case AF_INET:
		if (flags & NI_NUMERICHOST) {
			if (inet_ntop (AF_INET, &sinp->sin_addr, host, hostlen) == NULL) {
				return EAI_OVERFLOW;
			}
		}

		if (flags & NI_NUMERICSERV) {
			if (snprintf(serv, servlen, "%d", ntohs (sinp->sin_port)) < 0) {
				return EAI_OVERFLOW;
			}
		}

		break;
	}


	return 0;
}


const char *
gai_strerror(int errnum)
{
	switch (errnum) {
	case 0:
		return "no error";
	case EAI_BADFLAGS:
		return "invalid value for ai_flags";
	case EAI_NONAME:
		return "name or service is not known";
	case EAI_AGAIN:
		return "temporary failure in name resolution";
	case EAI_FAIL:
		return "non-recoverable failure in name resolution";
	case EAI_NODATA:
		return "no address associated with name";
	case EAI_FAMILY:
		return "ai_family not supported";
	case EAI_SOCKTYPE:
		return "ai_socktype not supported";
	case EAI_SERVICE:
		return "service not supported for ai_socktype";
	case EAI_ADDRFAMILY:
		return "address family for name not supported";
	case EAI_MEMORY:
		return "memory allocation failure";
	case EAI_SYSTEM:
		return "system error";
	case EAI_BADHINTS:
		return "invalid value for hints";
	case EAI_PROTOCOL:
		return "resolved protocol is unknown";
	case EAI_OVERFLOW:
		return "argument buffer overflow";
	default:
		return "unknown/invalid error";
	}
}

extern int h_errno;

struct hostent *gethostbyaddr(const void *addr, socklen_t len, int type) {
	h_errno = HOST_NOT_FOUND;

	return NULL;
}

// Current host name
static char *host_name = NULL;

int gethostname(char *name, size_t len) {
	if (name == NULL) {
		errno = EFAULT;
		return -1;
	}

	if ((int)len < 0) {
		errno = EINVAL;
		return -1;
	}

	// If hostname is not set, create the default hostname
	if (host_name == NULL) {
		host_name = strdup("ardupilot.local");
	}

	if (strlen(host_name) > len - 1) {
		errno = ENAMETOOLONG;
		return -1;
	}

	strncpy(name, host_name, len - 1);

	return 0;
}
#endif
