#ifndef UNIX_COMPAT_H
#define UNIX_COMPAT_H
#include <AP_HAL/AP_HAL_Boards.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS

#include "lwip/sockets.h"
#include "lwip/netdb.h"
#define IPV6_UNICAST_HOPS 16

// struct ipv6_mreq {
// 	/* IPv6 multicast address of group */
// 	struct in6_addr ipv6mr_multiaddr;

// 	/* local IPv6 address of interface */
// 	int		ipv6mr_ifindex;
// };

// Lua RTOS additions
struct hostent *gethostbyname(const char *name);
struct hostent *gethostbyaddr(const void *addr, socklen_t len, int type);
char *hstrerror(int herr);


#define EAI_OVERFLOW 208

 struct servent {
      char  *s_name;       /* official service name */
      char **s_aliases;    /* alias list */
      int    s_port;       /* port number */
      char  *s_proto;      /* protocol to use */
   };

/*
 * Error return codes from getaddrinfo()
 */
#ifndef ESHUTDOWN
#define ESHUTDOWN      108  /* Cannot send after transport endpoint shutdown */
#endif

#ifndef EAI_SOCKTYPE
#define	EAI_SOCKTYPE	10	/* ai_socktype not supported */
#endif

#ifndef EAI_AGAIN
#define	EAI_AGAIN	     2   /* temporary failure in name resolution */
#endif

#ifndef EAI_BADFLAGS
#define	EAI_BADFLAGS	 3	 /* invalid value for ai_flags */
#endif

 #define EAI_ADDRFAMILY   1      /* address family for hostname not supported */
 #define EAI_NODATA       7      /* no address associated with hostname */
 #define EAI_SOCKTYPE    10      /* ai_socktype not supported */
 #define EAI_SYSTEM      11      /* system error returned in errno */
 #define EAI_BADHINTS    12
 #define EAI_PROTOCOL    13
 #define EAI_MAX         14

/*
 * Constants for getnameinfo()
 */
#define    NI_MAXHOST  1025
#define    NI_MAXSERV  32

/*
 * Flag values for getnameinfo()
 */
#define    NI_NOFQDN   0x00000001
#define    NI_NUMERICHOST  0x00000002
#define    NI_NAMEREQD 0x00000004
#define    NI_NUMERICSERV  0x00000008
#define    NI_DGRAM    0x00000010
#define NI_WITHSCOPEID 0x00000020

#ifndef AF_UNIX
#define    AF_UNIX     1
#endif

int
 getnameinfo(const struct sockaddr *sa, socklen_t salen,
           char *host, size_t hostlen,
           char *serv, size_t servlen, int flags);

const char *gai_strerror(int ecode);

struct servent *getservbyname(const char *name, const char *proto);


#define IN6_IS_ADDR_UNSPECIFIED(a) \
	(((__const uint32_t *) (a))[0] == 0				      \
	 && ((__const uint32_t *) (a))[1] == 0				      \
	 && ((__const uint32_t *) (a))[2] == 0				      \
	 && ((__const uint32_t *) (a))[3] == 0)
#define IN6_IS_ADDR_LOOPBACK(a) \
	(((__const uint32_t *) (a))[0] == 0				      \
	 && ((__const uint32_t *) (a))[1] == 0				      \
	 && ((__const uint32_t *) (a))[2] == 0				      \
	 && ((__const uint32_t *) (a))[3] == htonl (1))
#define IN6_IS_ADDR_MULTICAST(a) (((__const uint8_t *) (a))[0] == 0xff)
#define IN6_IS_ADDR_LINKLOCAL(a) \
	((((__const uint32_t *) (a))[0] & htonl (0xffc00000))		      \
	 == htonl (0xfe800000))
#define IN6_IS_ADDR_SITELOCAL(a) \
	((((__const uint32_t *) (a))[0] & htonl (0xffc00000))		      \
	 == htonl (0xfec00000))
#define IN6_IS_ADDR_V4MAPPED(a) \
	((((__const uint32_t *) (a))[0] == 0)				      \
	 && (((__const uint32_t *) (a))[1] == 0)			      \
	 && (((__const uint32_t *) (a))[2] == htonl (0xffff)))
#define IN6_IS_ADDR_V4COMPAT(a) \
	((((__const uint32_t *) (a))[0] == 0)				      \
	 && (((__const uint32_t *) (a))[1] == 0)			      \
	 && (((__const uint32_t *) (a))[2] == 0)			      \
	 && (ntohl (((__const uint32_t *) (a))[3]) > 1))
#define IN6_ARE_ADDR_EQUAL(a,b) \
	((((__const uint32_t *) (a))[0] == ((__const uint32_t *) (b))[0])     \
	 && (((__const uint32_t *) (a))[1] == ((__const uint32_t *) (b))[1])  \
	 && (((__const uint32_t *) (a))[2] == ((__const uint32_t *) (b))[2])  \
	 && (((__const uint32_t *) (a))[3] == ((__const uint32_t *) (b))[3]))

#define IN6_IS_ADDR_MC_NODELOCAL(a) \
	(IN6_IS_ADDR_MULTICAST(a)					      \
	 && ((((__const uint8_t *) (a))[1] & 0xf) == 0x1))
#define IN6_IS_ADDR_MC_LINKLOCAL(a) \
	(IN6_IS_ADDR_MULTICAST(a)					      \
	 && ((((__const uint8_t *) (a))[1] & 0xf) == 0x2))
#define IN6_IS_ADDR_MC_SITELOCAL(a) \
	(IN6_IS_ADDR_MULTICAST(a)					      \
	 && ((((__const uint8_t *) (a))[1] & 0xf) == 0x5))
#define IN6_IS_ADDR_MC_ORGLOCAL(a) \
	(IN6_IS_ADDR_MULTICAST(a)					      \
	 && ((((__const uint8_t *) (a))[1] & 0xf) == 0x8))
#define IN6_IS_ADDR_MC_GLOBAL(a) \
	(IN6_IS_ADDR_MULTICAST(a)					      \
	 && ((((__const uint8_t *) (a))[1] & 0xf) == 0xe))

int res_init(void);

#endif // UNIX_COMPAT_H
#endif
