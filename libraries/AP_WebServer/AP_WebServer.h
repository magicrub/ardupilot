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

#include "AP_WebServer_Config.h"

#if AP_WEBSERVER_ENABLED

#include <AP_Param/AP_Param.h>
#include <AP_HAL/Semaphores.h>

class SocketAPM;

class AP_WebServer {
public:
    AP_WebServer();

    /* Do not allow copies */
    CLASS_NO_COPY(AP_WebServer);

    // Initialize the web server
    void init();

    static AP_WebServer *get_singleton() { return singleton; }

    static const struct AP_Param::GroupInfo var_info[];

private:
    static AP_WebServer *singleton;

    // Parameters
    AP_Int8 enabled;
    AP_Int16 port;

    // Listening socket
    SocketAPM *listen_sock;

    // Client connection state
    struct Client {
        SocketAPM *sock;
        uint32_t last_activity_ms;
        char recv_buf[AP_WEBSERVER_RECV_BUF_SIZE];
        uint16_t recv_len;
        bool header_complete;
        
        // Parsed request info
        enum class Method {
            UNKNOWN,
            GET,
            HEAD
        } method;
        char path[128];

        // State machine for non-blocking response handling
        enum class State {
            RECEIVING,      // Receiving request headers
            SENDING_HEADER, // About to send response header
            SENDING_BODY,   // Sending response body incrementally
            DONE            // Response complete, close connection
        } state;

        // State for ongoing transfers
        int file_fd;            // File descriptor for file transfers (-1 if not used)
        uint32_t bytes_sent;    // Bytes sent so far
        uint32_t bytes_total;   // Total bytes to send
        uint32_t transfer_start_ms; // For timing stats
        
        void reset();
        bool is_active() const { return sock != nullptr; }
    };
    
    Client clients[AP_WEBSERVER_MAX_CLIENTS];

    // Thread management
    bool thread_started;
    HAL_Semaphore sem;

    // Main server thread
    void server_thread();

    // Accept new client connections
    void accept_new_clients();

    // Process existing clients
    bool process_clients();

    // Process a single client
    void process_client(Client &client);

    // Parse HTTP request header
    bool parse_request(Client &client);

    // Send HTTP response
    void send_response(Client &client);

    // Send file content
    void send_file(Client &client, const char *filepath);

    // Send directory listing
    void send_directory_listing(Client &client, const char *dirpath, const char *urlpath);

    // Send download test (100MB of zeros for throughput testing)
    void send_speedtest(Client &client);

    // Send @SYS virtual file (threads.txt, tasks.txt, memory.txt, etc.)
    void send_sys_file(Client &client, const char *filename);

    // Send @SYS directory index
    void send_sys_index(Client &client);

    // Send thread monitor UI page (auto-updating with progress bars)
    void send_threads_ui(Client &client);

    // Continue incremental body transfer (for non-blocking I/O)
    void continue_body_transfer(Client &client);

    // Send error response
    void send_error(Client &client, uint16_t code, const char *message);

    // Send redirect response (adds trailing slash to path)
    void send_redirect(Client &client, const char *path);

    // Get MIME type for file extension
    static const char *get_mime_type(const char *filename);

    // URL decode a string in place
    static void url_decode(char *str);

    // Check if path is safe (no directory traversal)
    static bool is_safe_path(const char *path);

    // Close a client connection
    void close_client(Client &client);
};

namespace AP {
    AP_WebServer *webserver();
};

#endif // AP_WEBSERVER_ENABLED
