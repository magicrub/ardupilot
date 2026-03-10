/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_Networking/AP_Networking_Config.h>
#include <AP_Filesystem/AP_Filesystem_config.h>

// WebServer disabled by default, enable in hwdef for boards that need it
#ifndef AP_WEBSERVER_ENABLED
#define AP_WEBSERVER_ENABLED 1
#endif

#if AP_WEBSERVER_ENABLED && !AP_NETWORKING_ENABLED
#error AP_WEBSERVER_ENABLED is enabled but AP_NETWORKING_ENABLED is disabled
#endif

// Default port for the web server
#ifndef AP_WEBSERVER_DEFAULT_PORT
#define AP_WEBSERVER_DEFAULT_PORT 80
#endif

// Maximum number of concurrent clients
#ifndef AP_WEBSERVER_MAX_CLIENTS
#define AP_WEBSERVER_MAX_CLIENTS 4
#endif

// Size of receive buffer for HTTP requests
#ifndef AP_WEBSERVER_RECV_BUF_SIZE
#define AP_WEBSERVER_RECV_BUF_SIZE 1024
#endif

// Size of send buffer for HTTP responses (not used - inline buffers are smaller)
#ifndef AP_WEBSERVER_SEND_BUF_SIZE
#define AP_WEBSERVER_SEND_BUF_SIZE 512
#endif

// Timeout for inactive clients (milliseconds)
#ifndef AP_WEBSERVER_CLIENT_TIMEOUT_MS
#define AP_WEBSERVER_CLIENT_TIMEOUT_MS 10000
#endif

// Stack size for the web server thread
#ifndef AP_WEBSERVER_STACK_SIZE
#define AP_WEBSERVER_STACK_SIZE 8192
#endif

// Root path for serving files from SD card
#ifndef AP_WEBSERVER_SD_ROOT
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#define AP_WEBSERVER_SD_ROOT "."
#else
#define AP_WEBSERVER_SD_ROOT "/APM"
#endif
#endif
