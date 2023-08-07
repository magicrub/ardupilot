#ifndef USOCKET_H
#define USOCKET_H
/*=========================================================================*\
* Socket compatibilization module for Unix
* LuaSocket toolkit
\*=========================================================================*/

/*=========================================================================*\
* BSD include files
\*=========================================================================*/
/* error codes */
#include <AP_HAL/AP_HAL_Boards.h>

#include <errno.h>
/* close function */
#include <unistd.h>
/* fnctnl function and associated constants */
#include <fcntl.h>
/* struct sockaddr */
#include <sys/types.h>
/* socket function */
#if CONFIG_HAL_BOARD != HAL_BOARD_CHIBIOS
#include <sys/socket.h>
/* struct timeval */
#include <sys/time.h>
/* gethostbyname and gethostbyaddr functions */
#include <netdb.h>
/* sigpipe handling */
#include <signal.h>
/* IP stuff*/
#include <netinet/in.h>
#include <arpa/inet.h>
/* TCP options (nagle algorithm disable) */
#include <netinet/tcp.h>
#include <net/if.h>
#else
#include <lwip/err.h>
#include <lwip/sockets.h>
#include <lwip/sys.h>
#include <lwip/netdb.h>
#include <lwip/dns.h>
#include <lwip/inet.h>
#include "unix_compat.h"
#endif
#ifndef SO_REUSEPORT
#define SO_REUSEPORT SO_REUSEADDR
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#define SOCKET_SELECT
#endif

/* Some platforms use IPV6_JOIN_GROUP instead if
 * IPV6_ADD_MEMBERSHIP. The semantics are same, though. */
#ifndef IPV6_ADD_MEMBERSHIP
#ifdef IPV6_JOIN_GROUP
#define IPV6_ADD_MEMBERSHIP IPV6_JOIN_GROUP
#endif /* IPV6_JOIN_GROUP */
#endif /* !IPV6_ADD_MEMBERSHIP */

/* Same with IPV6_DROP_MEMBERSHIP / IPV6_LEAVE_GROUP. */
#ifndef IPV6_DROP_MEMBERSHIP
#ifdef IPV6_LEAVE_GROUP
#define IPV6_DROP_MEMBERSHIP IPV6_LEAVE_GROUP
#endif /* IPV6_LEAVE_GROUP */
#endif /* !IPV6_DROP_MEMBERSHIP */

typedef int t_socket;
typedef t_socket *p_socket;
typedef struct sockaddr_storage t_sockaddr_storage;

#define SOCKET_INVALID (-1)

#endif /* USOCKET_H */
