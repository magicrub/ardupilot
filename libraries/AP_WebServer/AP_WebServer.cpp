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

#include "AP_WebServer.h"

#if AP_WEBSERVER_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/Socket.h>
#include <AP_Filesystem/AP_Filesystem.h>
#include <AP_Networking/AP_Networking.h>
#include <GCS_MAVLink/GCS.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <new>

extern const AP_HAL::HAL &hal;

AP_WebServer *AP_WebServer::singleton;

const AP_Param::GroupInfo AP_WebServer::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Web Server Enable
    // @Description: Enable the built-in web server for file access
    // @Values: 0:Disabled,1:Enabled
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE", 1, AP_WebServer, enabled, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: PORT
    // @DisplayName: Web Server Port
    // @Description: TCP port for the web server
    // @Range: 1 65535
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("PORT", 2, AP_WebServer, port, AP_WEBSERVER_DEFAULT_PORT),

    AP_GROUPEND
};

// MIME type mapping
struct MimeType {
    const char *extension;
    const char *mime_type;
};

static const MimeType mime_types[] = {
    { "html", "text/html" },
    { "htm", "text/html" },
    { "css", "text/css" },
    { "js", "text/javascript" },
    { "json", "application/json" },
    { "txt", "text/plain" },
    { "log", "text/plain" },
    { "parm", "text/plain" },
    { "param", "text/plain" },
    { "lua", "text/x-lua" },
    { "xml", "text/xml" },
    { "png", "image/png" },
    { "jpg", "image/jpeg" },
    { "jpeg", "image/jpeg" },
    { "gif", "image/gif" },
    { "ico", "image/x-icon" },
    { "svg", "image/svg+xml" },
    { "bin", "application/octet-stream" },
    { "BIN", "application/octet-stream" },
    { "apj", "application/octet-stream" },
    { "zip", "application/zip" },
    { "gz", "application/gzip" },
    { "pdf", "application/pdf" },
    { "wav", "audio/wav" },
    { "mp3", "audio/mpeg" },
    { "mp4", "video/mp4" },
    { nullptr, nullptr }
};

AP_WebServer::AP_WebServer()
{
    singleton = this;
    AP_Param::setup_object_defaults(this, var_info);
}

void AP_WebServer::init()
{
    if (!enabled) {
        return;
    }

    // Create listening socket
    listen_sock = new SocketAPM(false);
    if (listen_sock == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "WebServer: failed to create socket");
        return;
    }

    listen_sock->reuseaddress();

    // Start server thread
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_WebServer::server_thread, void),
                                       "web_server",
                                       AP_WEBSERVER_STACK_SIZE,
                                       AP_HAL::Scheduler::PRIORITY_IO,
                                       0)) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "WebServer: failed to create thread");
        delete listen_sock;
        listen_sock = nullptr;
        return;
    }

    thread_started = true;
}

void AP_WebServer::server_thread()
{
    // Wait for networking to be ready
    AP::network().startup_wait();

    // Bind and listen
    if (!listen_sock->bind("0.0.0.0", port.get()) || !listen_sock->listen(AP_WEBSERVER_MAX_CLIENTS)) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "WebServer: failed to bind to port %d", (int)port.get());
        return;
    }

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "WebServer: listening on port %d", (int)port.get());

    uint8_t skip_sleep_count = 0;
    while (true) {
        accept_new_clients();
        if (!process_clients()) {
            skip_sleep_count = 0;
            hal.scheduler->delay_microseconds(1000);
        } else if (++skip_sleep_count >= 3) {
            skip_sleep_count = 0;
            hal.scheduler->delay_microseconds(100);
        }
    }
}

void AP_WebServer::accept_new_clients()
{
    if (!listen_sock->pollin(0)) {
        return;
    }

    SocketAPM *new_sock = listen_sock->accept(0);
    if (new_sock == nullptr) {
        return;
    }

    new_sock->set_blocking(false);

    // Find free client slot
    WITH_SEMAPHORE(sem);
    for (auto &client : clients) {
        if (!client.is_active()) {
            client.reset();
            client.sock = new_sock;
            client.last_activity_ms = AP_HAL::millis();
            return;
        }
    }

    // No free slots, reject connection
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "WebServer: no free client slots");
    delete new_sock;
}

// returns true if any client data or tasks were processed
bool AP_WebServer::process_clients()
{
    bool busy = false;
    const uint32_t now_ms = AP_HAL::millis();

    // Process each client without holding semaphore during I/O
    for (auto &client : clients) {
        if (!client.is_active()) {
            continue;
        }

        // Check for timeout
        if (now_ms - client.last_activity_ms > AP_WEBSERVER_CLIENT_TIMEOUT_MS) {
            WITH_SEMAPHORE(sem);
            close_client(client);
            continue;
        }

        process_client(client);

        // Check if client is done
        if (client.state == Client::State::DONE) {
            WITH_SEMAPHORE(sem);
            close_client(client);
        } else {
            busy = true;
        }
    }
    return busy;
}

void AP_WebServer::process_client(Client &client)
{
    // Safety check - socket could be null if closed elsewhere
    if (client.sock == nullptr) {
        client.state = Client::State::DONE;
        return;
    }

    switch (client.state) {
    case Client::State::RECEIVING:
        // Try to receive more data (only if socket has data ready)
        if (client.recv_len < sizeof(client.recv_buf) - 1 && client.sock->pollin(0)) {
            ssize_t n = client.sock->recv(&client.recv_buf[client.recv_len],
                                           sizeof(client.recv_buf) - 1 - client.recv_len, 0);
            if (n > 0) {
                client.recv_len += n;
                client.recv_buf[client.recv_len] = '\0';
                client.last_activity_ms = AP_HAL::millis();
            } else if (n == 0) {
                // Connection closed by peer before sending response
                if (client.recv_len == 0) {
                    // Connection closed without any data - likely a prefetch/preconnect
                    // Don't log, this is common with modern browsers
                } else {
                    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "WebServer: peer closed, recv_len=%u", client.recv_len);
                }
                client.state = Client::State::DONE;
                return;
            }
            // n < 0 with pollin true is an error, but we'll let timeout handle it
        }

        // Try to parse the request
        if (!client.header_complete) {
            if (!parse_request(client)) {
                return;  // Need more data
            }
            client.header_complete = true;
            client.state = Client::State::SENDING_HEADER;
        }
        break;

    case Client::State::SENDING_HEADER:
        // Dispatch to appropriate handler which will send header and set up body transfer
        send_response(client);
        break;

    case Client::State::SENDING_BODY:
        // Continue incremental body transfer
        continue_body_transfer(client);
        break;

    case Client::State::DONE:
        // Will be cleaned up by process_clients
        break;
    }
}

bool AP_WebServer::parse_request(Client &client)
{
    // Look for end of headers (\r\n\r\n)
    char *end_of_headers = strstr(client.recv_buf, "\r\n\r\n");
    if (end_of_headers == nullptr) {
        return false;  // Need more data
    }

    // Parse request line
    char *line = client.recv_buf;
    char *space1 = strchr(line, ' ');
    if (space1 == nullptr) {
        return true;  // Malformed, but complete
    }

    *space1 = '\0';
    if (strcmp(line, "GET") == 0) {
        client.method = Client::Method::GET;
    } else if (strcmp(line, "HEAD") == 0) {
        client.method = Client::Method::HEAD;
    } else {
        client.method = Client::Method::UNKNOWN;
        return true;
    }

    char *path_start = space1 + 1;
    char *space2 = strchr(path_start, ' ');
    if (space2 == nullptr) {
        return true;  // Malformed
    }

    *space2 = '\0';
    
    // Copy and decode path
    strncpy(client.path, path_start, sizeof(client.path) - 1);
    client.path[sizeof(client.path) - 1] = '\0';
    url_decode(client.path);

    return true;
}

void AP_WebServer::send_response(Client &client)
{
    if (client.method == Client::Method::UNKNOWN) {
        send_error(client, 405, "Method Not Allowed");
        return;
    }

    // Check for safe path
    if (!is_safe_path(client.path)) {
        send_error(client, 403, "Forbidden");
        return;
    }

    // Special path: download test (20MB of zeros for throughput testing)
    if (strcmp(client.path, "/speedtest") == 0) {
        send_speedtest(client);
        return;
    }

    // Special path: thread monitor UI (auto-updating with progress bars)
    if (strcmp(client.path, "/threads") == 0) {
        send_threads_ui(client);
        return;
    }

    // Special path: @SYS virtual filesystem (threads.txt, tasks.txt, memory.txt, etc.)
    if (strcmp(client.path, "/sys") == 0 || strcmp(client.path, "/sys/") == 0) {
        send_sys_index(client);
        return;
    }
    if (strncmp(client.path, "/sys/", 5) == 0) {
        send_sys_file(client, client.path + 5);  // Skip "/sys/"
        return;
    }

    // Build full filesystem path
    char filepath[300];
    if (client.path[0] == '/' && client.path[1] == '\0') {
        // Root path - serve directory listing
        snprintf(filepath, sizeof(filepath), "%s", AP_WEBSERVER_SD_ROOT);
    } else {
        snprintf(filepath, sizeof(filepath), "%s%s", AP_WEBSERVER_SD_ROOT, client.path);
    }

    // Check if path exists
    struct stat st;
    if (AP::FS().stat(filepath, &st) != 0) {
        send_error(client, 404, "Not Found");
        return;
    }

    if (S_ISDIR(st.st_mode)) {
        // Redirect to path with trailing slash if missing (required for relative links to work)
        size_t path_len = strlen(client.path);
        if (path_len == 0 || client.path[path_len - 1] != '/') {
            send_redirect(client, client.path);
            return;
        }
        // Directory listing
        send_directory_listing(client, filepath, client.path);
    } else {
        // Send file
        send_file(client, filepath);
    }
}

void AP_WebServer::send_file(Client &client, const char *filepath)
{
    // Get file info
    struct stat st;
    if (AP::FS().stat(filepath, &st) != 0) {
        send_error(client, 404, "Not Found");
        return;
    }

    // Open file
    int fd = AP::FS().open(filepath, O_RDONLY);
    if (fd < 0) {
        send_error(client, 500, "Internal Server Error");
        return;
    }

    const char *mime_type = get_mime_type(filepath);
    
    // Send headers
    char header[160];
    int header_len = snprintf(header, sizeof(header),
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: %s\r\n"
        "Content-Length: %lu\r\n"
        "Connection: close\r\n"
        "\r\n",
        mime_type,
        (unsigned long)st.st_size);

    client.sock->send(header, header_len);

    // Send body if GET (not HEAD)
    if (client.method == Client::Method::GET) {
        client.file_fd = fd;
        client.bytes_sent = 0;
        client.bytes_total = st.st_size;
        client.transfer_start_ms = AP_HAL::millis();
        client.state = Client::State::SENDING_BODY;
    } else {
        // HEAD request - no body to send
        AP::FS().close(fd);
        client.state = Client::State::DONE;
    }
}

void AP_WebServer::send_directory_listing(Client &client, const char *dirpath, const char *urlpath)
{
    auto *dir = AP::FS().opendir(dirpath);
    if (dir == nullptr) {
        send_error(client, 500, "Internal Server Error");
        return;
    }

    // Send HTTP headers
    const char *header = 
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: text/html; charset=utf-8\r\n"
        "Transfer-Encoding: chunked\r\n"
        "Connection: close\r\n"
        "\r\n";
    client.sock->send(header, strlen(header));

    // Helper to send a chunk
    auto send_chunk = [&client](const char *data, size_t len) {
        char chunk_hdr[16];
        snprintf(chunk_hdr, sizeof(chunk_hdr), "%x\r\n", (unsigned)len);
        client.sock->send(chunk_hdr, strlen(chunk_hdr));
        client.sock->send(data, len);
        client.sock->send("\r\n", 2);
    };

    // Send HTML header (use static strings to avoid stack usage)
    const char *html_start = 
        "<!DOCTYPE html>\n<html>\n<head><title>Directory</title>\n"
        "<style>body{font-family:monospace;margin:20px}a{color:#06c}</style>\n"
        "</head>\n<body>\n<h1>Directory Listing</h1>\n<table>\n";
    send_chunk(html_start, strlen(html_start));

    // Parent directory link
    if (strcmp(urlpath, "/") != 0) {
        const char *parent = "<tr><td><a href=\"..\">..</a></td><td>-</td></tr>\n";
        send_chunk(parent, strlen(parent));
    }

    // Use a smaller buffer for entries
    char buf[256];
    char fullpath[256];
    
    // List directory entries
    struct dirent *entry;
    while ((entry = AP::FS().readdir(dir)) != nullptr) {
        if (entry->d_name[0] == '.') {
            continue;
        }

        // Build path for stat
        snprintf(fullpath, sizeof(fullpath), "%.120s/%.120s", dirpath, entry->d_name);
        
        struct stat st;
        bool is_dir = false;
        unsigned long size = 0;
        
        if (AP::FS().stat(fullpath, &st) == 0) {
            is_dir = S_ISDIR(st.st_mode);
            size = st.st_size;
        }

        int len;
        if (is_dir) {
            len = snprintf(buf, sizeof(buf),
                "<tr><td><b><a href=\"%s/\">%s/</a></b></td><td>-</td></tr>\n",
                entry->d_name, entry->d_name);
        } else {
            const char *unit = "B";
            float display_size = size;
            if (size >= 1024*1024) {
                display_size = size / (1024.0f * 1024.0f);
                unit = "MB";
            } else if (size >= 1024) {
                display_size = size / 1024.0f;
                unit = "KB";
            }
            len = snprintf(buf, sizeof(buf),
                "<tr><td><a href=\"%s\">%s</a></td><td>%.1f %s</td></tr>\n",
                entry->d_name, entry->d_name, display_size, unit);
        }
        if (len > 0 && len < (int)sizeof(buf)) {
            send_chunk(buf, len);
        }
    }

    AP::FS().closedir(dir);

    // Send closing HTML
    const char *html_end = "</table>\n<hr>\n<p>Kraus Hamdani Aerospace</p>\n</body>\n</html>\n";
    send_chunk(html_end, strlen(html_end));

    // Final chunk
    client.sock->send("0\r\n\r\n", 5);
    client.state = Client::State::DONE;
}

void AP_WebServer::send_speedtest(Client &client)
{
    // Send 20MB of zeros for throughput testing
    static constexpr uint32_t TEST_SIZE = 20 * 1e6f;  // 20 MB
    
    // Send headers
    char header[256];
    int header_len = snprintf(header, sizeof(header),
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: application/octet-stream\r\n"
        "Content-Length: %lu\r\n"
        "Content-Disposition: attachment; filename=\"speedtest_20mb.bin\"\r\n"
        "Connection: close\r\n"
        "\r\n",
        (unsigned long)TEST_SIZE);

    if (client.sock->send(header, header_len) != header_len) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "WebServer: download test header send failed");
        client.state = Client::State::DONE;
        return;
    }

    // Set up for incremental body transfer if GET (not HEAD)
    if (client.method == Client::Method::GET) {
        client.file_fd = -1;  // -1 indicates speedtest mode (not a file)
        client.bytes_sent = 0;
        client.bytes_total = TEST_SIZE;
        client.transfer_start_ms = AP_HAL::millis();
        client.state = Client::State::SENDING_BODY;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "WebServer: download test starting");
    } else {
        client.state = Client::State::DONE;
    }
}

void AP_WebServer::send_sys_file(Client &client, const char *filename)
{
    if (client.sock == nullptr) {
        client.state = Client::State::DONE;
        return;
    }

    // Build @SYS path
    char syspath[64];
    snprintf(syspath, sizeof(syspath), "@SYS/%s", filename);

    // Read virtual file via filesystem API
    int fd = AP::FS().open(syspath, O_RDONLY);
    if (fd < 0) {
        // Try appending .txt extension if not found
        char syspath_txt[68];
        snprintf(syspath_txt, sizeof(syspath_txt), "@SYS/%s.txt", filename);
        fd = AP::FS().open(syspath_txt, O_RDONLY);
        if (fd < 0) {
            send_error(client, 404, "Not Found");
            return;
        }
    }

    // Allocate buffer on heap (virtual files are typically <8KB)
    const size_t buf_size = 8192;
    char *body = new char[buf_size];
    if (body == nullptr) {
        AP::FS().close(fd);
        send_error(client, 500, "Out of memory");
        return;
    }

    ssize_t body_len = AP::FS().read(fd, body, buf_size - 1);
    AP::FS().close(fd);

    if (body_len < 0) {
        delete[] body;
        send_error(client, 500, "Read error");
        return;
    }
    body[body_len] = '\0';

    // Send headers with Content-Length
    char header[160];
    int header_len = snprintf(header, sizeof(header),
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: text/plain; charset=utf-8\r\n"
        "Content-Length: %u\r\n"
        "Connection: close\r\n"
        "Cache-Control: no-cache\r\n"
        "\r\n",
        (unsigned)body_len);

    if (client.sock->send(header, header_len) != header_len) {
        delete[] body;
        client.state = Client::State::DONE;
        return;
    }

    // Send body if GET (not HEAD)
    if (client.method == Client::Method::GET && body_len > 0) {
        client.sock->send(body, body_len);
    }

    delete[] body;
    client.state = Client::State::DONE;
}

void AP_WebServer::send_sys_index(Client &client)
{
    if (client.sock == nullptr) {
        client.state = Client::State::DONE;
        return;
    }

    // List @SYS directory via filesystem
    auto *dir = AP::FS().opendir("@SYS");
    if (dir == nullptr) {
        send_error(client, 500, "Cannot open @SYS");
        return;
    }

    // Build HTML listing
    static const char html_start[] = 
        "<!DOCTYPE html>\n<html>\n<head><title>@SYS</title>\n"
        "<style>body{font-family:monospace;margin:20px;background:#1a1a2e;color:#eee}"
        "a{color:#0af;text-decoration:none}a:hover{text-decoration:underline}"
        "h1{color:#e94560}ul{list-style:none;padding:0}li{padding:4px 0}</style>\n"
        "</head>\n<body>\n<h1>@SYS Virtual Files</h1>\n<ul>\n";
    static const char html_end[] = "</ul>\n<p><a href=\"/\">Back to root</a></p>\n</body>\n</html>\n";

    // Send headers (chunked encoding)
    const char *header = 
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: text/html; charset=utf-8\r\n"
        "Transfer-Encoding: chunked\r\n"
        "Connection: close\r\n"
        "\r\n";
    client.sock->send(header, strlen(header));

    auto send_chunk = [&client](const char *data, size_t len) {
        char chunk_hdr[16];
        snprintf(chunk_hdr, sizeof(chunk_hdr), "%x\r\n", (unsigned)len);
        client.sock->send(chunk_hdr, strlen(chunk_hdr));
        client.sock->send(data, len);
        client.sock->send("\r\n", 2);
    };

    send_chunk(html_start, sizeof(html_start) - 1);

    char buf[128];
    struct dirent *entry;
    while ((entry = AP::FS().readdir(dir)) != nullptr) {
        int len = snprintf(buf, sizeof(buf), "<li><a href=\"/sys/%s\">%s</a></li>\n", 
                           entry->d_name, entry->d_name);
        if (len > 0 && len < (int)sizeof(buf)) {
            send_chunk(buf, len);
        }
    }
    AP::FS().closedir(dir);

    send_chunk(html_end, sizeof(html_end) - 1);
    client.sock->send("0\r\n\r\n", 5);  // Final chunk
    client.state = Client::State::DONE;
}

void AP_WebServer::send_threads_ui(Client &client)
{
    if (client.sock == nullptr) {
        client.state = Client::State::DONE;
        return;
    }

    // HTML page with embedded CSS and JavaScript for auto-updating thread display
    static const char html_page[] = R"(<!DOCTYPE html>
<html>
<head>
<title>Thread Monitor</title>
<meta charset="utf-8">
<style>
*{box-sizing:border-box}
body{font:12px/1.4 -apple-system,system-ui,sans-serif;margin:0;padding:12px;background:#0d1117;color:#c9d1d9}
h1{font-size:14px;font-weight:600;color:#58a6ff;margin:0 0 12px 0}
.thread{display:grid;grid-template-columns:110px 32px 1fr 54px;gap:8px;padding:4px 8px;align-items:center;border-bottom:1px solid #21262d}
.thread:hover{background:#161b22}
.name{font-family:ui-monospace,monospace;font-weight:500;overflow:hidden;text-overflow:ellipsis;white-space:nowrap}
.pri{color:#8b949e;font-size:11px;text-align:center}
.bar-container{background:#21262d;border-radius:3px;height:14px;overflow:hidden;position:relative}
.bar{height:100%;border-radius:3px;transition:width 0.2s}
.bar-stack{background:#238636}
.bar-warn{background:#9e6a03}
.bar-crit{background:#da3633}
.bar-text{position:absolute;right:4px;top:0;font-size:10px;line-height:14px;color:#fff;text-shadow:0 0 2px #000}
.load{text-align:right;font-family:ui-monospace,monospace;font-size:11px}
.header{font-weight:600;color:#8b949e;font-size:10px;text-transform:uppercase;letter-spacing:0.5px;border-bottom:1px solid #30363d;padding-bottom:6px;margin-bottom:2px}
.header:hover{background:transparent}
.updated{color:#484f58;font-size:10px;margin-top:12px;padding-top:8px;border-top:1px solid #21262d}
.isr{background:#1c1226}
</style>
</head>
<body>
<h1>Thread Monitor</h1>
<div class="thread header"><span>Name</span><span>Pri</span><span>Stack</span><span>CPU</span></div>
<div id="threads"></div>
<div class="updated">Updated: <span id="time">-</span></div>
<script>
function update() {
  fetch('/sys/threads.txt').then(r => r.text()).then(data => {
    const lines = data.trim().split('\n');
    let html = '';
    for (let i = 1; i < lines.length; i++) {
      const line = lines[i];
      // Parse: NAME PRI=xxx sp=xxx STACK=free/total [LOAD=xx.x%][*]
      const m = line.match(/^(.+?)\s+PRI=\s*(\d+)\s+sp=\S+\s+STACK=\s*(\d+)\s*\/\s*(\d+)/);
      if (!m) continue;
      const name = m[1].trim();
      const pri = m[2];
      const free = parseInt(m[3]);
      const total = parseInt(m[4]);
      // Try to extract load percentage
      const loadMatch = line.match(/LOAD=\s*([\d.]+)%/);
      const load = loadMatch ? parseFloat(loadMatch[1]) : 0;
      const used = total - free;
      const stackPct = (used / total * 100).toFixed(1);
      const stackClass = stackPct > 90 ? 'bar-crit' : stackPct > 70 ? 'bar-warn' : 'bar-stack';
      const isIsr = name === 'ISR';
      html += '<div class="thread' + (isIsr ? ' isr' : '') + '">' +
        '<span class="name" title="' + name + '">' + name + '</span>' +
        '<span class="pri">' + pri + '</span>' +
        '<div class="bar-container"><div class="bar ' + stackClass + '" style="width:' + stackPct + '%"></div>' +
        '<span class="bar-text">' + stackPct + '%</span></div>' +
        '<span class="load">' + load.toFixed(1) + '%</span></div>';
    }
    document.getElementById('threads').innerHTML = html;
    document.getElementById('time').textContent = new Date().toLocaleTimeString();
  }).catch(e => console.error(e));
}
update();
setInterval(update, 1000);
</script>
</body>
</html>)";

    const size_t body_len = sizeof(html_page) - 1;  // Exclude null terminator

    // Send headers
    char header[160];
    int header_len = snprintf(header, sizeof(header),
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: text/html; charset=utf-8\r\n"
        "Content-Length: %u\r\n"
        "Connection: close\r\n"
        "\r\n",
        (unsigned)body_len);

    if (client.sock->send(header, header_len) != header_len) {
        client.state = Client::State::DONE;
        return;
    }

    // Send body if GET (not HEAD)
    if (client.method == Client::Method::GET) {
        client.sock->send(html_page, body_len);
    }

    client.state = Client::State::DONE;
}

void AP_WebServer::continue_body_transfer(Client &client)
{
    if (client.sock == nullptr) {
        client.state = Client::State::DONE;
        return;
    }

    // Send as much data as the socket will accept in a tight loop.
    // Only return when the socket blocks, transfer completes, or an error occurs.
    static constexpr uint32_t CHUNK_SIZE = 4096;
    static uint8_t buf[CHUNK_SIZE];

    while (true) {
        uint32_t remaining = client.bytes_total - client.bytes_sent;
        if (remaining == 0) {
            // Transfer complete
            if (client.bytes_total > 0) {
                const uint32_t elapsed_ms = AP_HAL::millis() - client.transfer_start_ms;
                if (elapsed_ms > 0) {
                    const float mbps = ((float)client.bytes_sent * 8.0f) / (elapsed_ms * 1000.0f);
                    uint32_t bytes_sent = client.bytes_sent;
                    const uint32_t elapsed_min = elapsed_ms / (60*1000);
                    const uint32_t elapsed_sec_mod_minute = (elapsed_ms % (60*1000)) * 0.001f;

                    const char* units[] = {"KB", "MB", "GB"};
                    int unit_index = 0;
                    while (unit_index < 2 && bytes_sent >= 1e6f) {
                        bytes_sent *= 1e-3f;
                        unit_index++;
                    }

                    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "WebServer: transfer %.2f%s in %um%us %.2f Mbps",
                                    (double)(bytes_sent*1e-3f), units[unit_index], (unsigned)elapsed_min, (unsigned)elapsed_sec_mod_minute, mbps);
                }
            }
            if (client.file_fd >= 0) {
                AP::FS().close(client.file_fd);
                client.file_fd = -1;
            }
            client.state = Client::State::DONE;
            return;
        }

        // Check if socket is ready for writing
        if (!client.sock->pollout(0)) {
            return;  // Socket would block, try again next iteration
        }

        uint32_t to_send = (remaining > CHUNK_SIZE) ? CHUNK_SIZE : remaining;
        ssize_t n_read;

        if (client.file_fd >= 0) {
            n_read = AP::FS().read(client.file_fd, buf, to_send);
            if (n_read <= 0) {
                GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "WebServer: read error at %lu bytes",
                              (unsigned long)client.bytes_sent);
                client.state = Client::State::DONE;
                return;
            }
        } else {
            // Speedtest - send zeros
            memset(buf, 0, to_send);
            n_read = to_send;
        }

        ssize_t sent = client.sock->send(buf, n_read);
        if (sent <= 0) {
            // Send buffer full or error - return and retry next iteration
            if (client.file_fd >= 0 && n_read > 0) {
                AP::FS().lseek(client.file_fd, 0, SEEK_CUR);  // no seek needed, we didn't advance
            }
            return;
        }

        client.bytes_sent += sent;
        client.last_activity_ms = AP_HAL::millis();

        // If we didn't send all we read from a file, seek back
        if (client.file_fd >= 0 && sent < n_read) {
            AP::FS().lseek(client.file_fd, -(n_read - sent), SEEK_CUR);
        }
    }
}

void AP_WebServer::send_error(Client &client, uint16_t code, const char *message)
{
    if (client.sock == nullptr) {
        client.state = Client::State::DONE;
        return;
    }

    char buf[256];
    int body_len = snprintf(buf, sizeof(buf),
        "<!DOCTYPE html>\n<html>\n<head><title>%u %s</title></head>\n"
        "<body><h1>%u %s</h1></body>\n</html>\n",
        code, message, code, message);

    char header[128];
    int header_len = snprintf(header, sizeof(header),
        "HTTP/1.1 %u %s\r\n"
        "Content-Type: text/html\r\n"
        "Content-Length: %d\r\n"
        "Connection: close\r\n\r\n",
        code, message, body_len);

    client.sock->send(header, header_len);
    if (client.method != Client::Method::HEAD) {
        client.sock->send(buf, body_len);
    }
    client.state = Client::State::DONE;
}

void AP_WebServer::send_redirect(Client &client, const char *path)
{
    if (client.sock == nullptr) {
        client.state = Client::State::DONE;
        return;
    }

    // Redirect to path with trailing slash
    char header[256];
    int header_len = snprintf(header, sizeof(header),
        "HTTP/1.1 301 Moved Permanently\r\n"
        "Location: %s/\r\n"
        "Content-Length: 0\r\n"
        "Connection: close\r\n\r\n",
        path);

    client.sock->send(header, header_len);
    client.state = Client::State::DONE;
}

const char *AP_WebServer::get_mime_type(const char *filename)
{
    const char *ext = strrchr(filename, '.');
    if (ext == nullptr) {
        return "application/octet-stream";
    }
    ext++;  // Skip the dot

    for (const MimeType *mt = mime_types; mt->extension != nullptr; mt++) {
        if (strcasecmp(ext, mt->extension) == 0) {
            return mt->mime_type;
        }
    }

    return "application/octet-stream";
}

void AP_WebServer::url_decode(char *str)
{
    char *dst = str;
    char *src = str;

    while (*src) {
        if (*src == '%' && isxdigit(src[1]) && isxdigit(src[2])) {
            // Decode %XX
            char hex[3] = { src[1], src[2], '\0' };
            *dst = (char)strtol(hex, nullptr, 16);
            src += 3;
        } else if (*src == '+') {
            *dst = ' ';
            src++;
        } else {
            *dst = *src;
            src++;
        }
        dst++;
    }
    *dst = '\0';
}

bool AP_WebServer::is_safe_path(const char *path)
{
    // Check for directory traversal attempts
    if (strstr(path, "..") != nullptr) {
        return false;
    }

    // Path must start with /
    if (path[0] != '/') {
        return false;
    }

    return true;
}

void AP_WebServer::close_client(Client &client)
{
    if (client.file_fd >= 0) {
        AP::FS().close(client.file_fd);
        client.file_fd = -1;
    }
    if (client.sock != nullptr) {
        client.sock->close();
        delete client.sock;
        client.sock = nullptr;
    }
}

void AP_WebServer::Client::reset()
{
    sock = nullptr;
    last_activity_ms = 0;
    recv_len = 0;
    header_complete = false;
    method = Method::UNKNOWN;
    path[0] = '\0';
    state = State::RECEIVING;
    file_fd = -1;
    bytes_sent = 0;
    bytes_total = 0;
    transfer_start_ms = 0;
}

namespace AP {
AP_WebServer *webserver()
{
    return AP_WebServer::get_singleton();
}
}

#endif // AP_WEBSERVER_ENABLED
