#ifndef WEBSOCKET_SERVER_H
#define WEBSOCKET_SERVER_H

// Minimal header-only broadcast-only WebSocket server (RFC 6455).
// - POSIX sockets (macOS / Linux)
// - Inline SHA1 + base64 (no external crypto deps)
// - Sends unmasked text frames only
// - Silently drops clients whose sends would block

#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include <cstdint>
#include <cstring>
#include <cstdio>
#include <sstream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

#ifndef MSG_NOSIGNAL
#define MSG_NOSIGNAL 0
#endif

class WebSocketServer {
public:
    WebSocketServer() = default;
    ~WebSocketServer() { stop(); }

    bool start(int port) {
        listen_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
        if (listen_fd_ < 0) return false;

        int yes = 1;
        ::setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
#ifdef SO_NOSIGPIPE
        ::setsockopt(listen_fd_, SOL_SOCKET, SO_NOSIGPIPE, &yes, sizeof(yes));
#endif

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_ANY);
        addr.sin_port = htons(port);
        if (::bind(listen_fd_, (sockaddr*)&addr, sizeof(addr)) < 0) {
            ::close(listen_fd_);
            listen_fd_ = -1;
            return false;
        }
        if (::listen(listen_fd_, 8) < 0) {
            ::close(listen_fd_);
            listen_fd_ = -1;
            return false;
        }

        running_ = true;
        accept_thread_ = std::thread([this] { accept_loop(); });
        return true;
    }

    void stop() {
        if (!running_.exchange(false)) return;
        if (listen_fd_ >= 0) {
            ::shutdown(listen_fd_, SHUT_RDWR);
            ::close(listen_fd_);
            listen_fd_ = -1;
        }
        if (accept_thread_.joinable()) accept_thread_.join();
        std::lock_guard<std::mutex> lock(clients_mutex_);
        for (int fd : clients_) ::close(fd);
        clients_.clear();
    }

    void broadcast(const std::string& msg) {
        std::vector<uint8_t> frame = build_text_frame(msg);
        std::lock_guard<std::mutex> lock(clients_mutex_);
        auto it = clients_.begin();
        while (it != clients_.end()) {
            if (!send_all(*it, frame.data(), frame.size())) {
                ::close(*it);
                it = clients_.erase(it);
            } else {
                ++it;
            }
        }
    }

private:
    int listen_fd_ = -1;
    std::atomic<bool> running_{false};
    std::thread accept_thread_;
    std::mutex clients_mutex_;
    std::vector<int> clients_;

    void accept_loop() {
        while (running_) {
            sockaddr_in caddr{};
            socklen_t clen = sizeof(caddr);
            int cfd = ::accept(listen_fd_, (sockaddr*)&caddr, &clen);
            if (cfd < 0) {
                if (!running_) break;
                continue;
            }
#ifdef SO_NOSIGPIPE
            int yes = 1;
            ::setsockopt(cfd, SOL_SOCKET, SO_NOSIGPIPE, &yes, sizeof(yes));
#endif
            int nodelay = 1;
            ::setsockopt(cfd, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));
            if (!handshake(cfd)) {
                ::close(cfd);
                continue;
            }
            std::lock_guard<std::mutex> lock(clients_mutex_);
            clients_.push_back(cfd);
        }
    }

    bool handshake(int fd) {
        // Read HTTP upgrade request (up to 4KB, until \r\n\r\n)
        std::string req;
        char buf[1024];
        while (req.find("\r\n\r\n") == std::string::npos && req.size() < 8192) {
            ssize_t n = ::recv(fd, buf, sizeof(buf), 0);
            if (n <= 0) return false;
            req.append(buf, buf + n);
        }

        // Find Sec-WebSocket-Key header (case-insensitive)
        std::string key = find_header(req, "sec-websocket-key");
        if (key.empty()) return false;

        std::string magic = key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
        unsigned char hash[20];
        sha1((const unsigned char*)magic.data(), magic.size(), hash);
        std::string accept = base64_encode(hash, 20);

        std::ostringstream resp;
        resp << "HTTP/1.1 101 Switching Protocols\r\n"
             << "Upgrade: websocket\r\n"
             << "Connection: Upgrade\r\n"
             << "Sec-WebSocket-Accept: " << accept << "\r\n\r\n";
        std::string r = resp.str();
        return send_all(fd, (const uint8_t*)r.data(), r.size());
    }

    static std::string find_header(const std::string& req, const std::string& name) {
        // Lowercase copy for case-insensitive search
        std::string lower;
        lower.reserve(req.size());
        for (char c : req) lower.push_back((c >= 'A' && c <= 'Z') ? (c + 32) : c);
        size_t pos = lower.find(name + ":");
        if (pos == std::string::npos) return "";
        pos = req.find(':', pos) + 1;
        size_t end = req.find("\r\n", pos);
        if (end == std::string::npos) return "";
        std::string val = req.substr(pos, end - pos);
        // Trim
        size_t a = val.find_first_not_of(" \t");
        size_t b = val.find_last_not_of(" \t");
        if (a == std::string::npos) return "";
        return val.substr(a, b - a + 1);
    }

    static std::vector<uint8_t> build_text_frame(const std::string& payload) {
        std::vector<uint8_t> f;
        f.push_back(0x81); // FIN=1, opcode=1 (text)
        size_t len = payload.size();
        if (len < 126) {
            f.push_back((uint8_t)len);
        } else if (len <= 0xFFFF) {
            f.push_back(126);
            f.push_back((uint8_t)((len >> 8) & 0xFF));
            f.push_back((uint8_t)(len & 0xFF));
        } else {
            f.push_back(127);
            for (int i = 7; i >= 0; i--) {
                f.push_back((uint8_t)((len >> (i * 8)) & 0xFF));
            }
        }
        f.insert(f.end(), payload.begin(), payload.end());
        return f;
    }

    static bool send_all(int fd, const uint8_t* data, size_t len) {
        size_t sent = 0;
        while (sent < len) {
            ssize_t n = ::send(fd, data + sent, len - sent, MSG_NOSIGNAL);
            if (n <= 0) return false;
            sent += (size_t)n;
        }
        return true;
    }

    // --- Base64 ---
    static std::string base64_encode(const unsigned char* data, size_t len) {
        static const char tbl[] =
            "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
        std::string out;
        int val = 0, valb = -6;
        for (size_t i = 0; i < len; i++) {
            val = (val << 8) | data[i];
            valb += 8;
            while (valb >= 0) {
                out.push_back(tbl[(val >> valb) & 0x3F]);
                valb -= 6;
            }
        }
        if (valb > -6) out.push_back(tbl[((val << 8) >> (valb + 8)) & 0x3F]);
        while (out.size() % 4) out.push_back('=');
        return out;
    }

    // --- SHA1 (RFC 3174) ---
    static void sha1(const unsigned char* data, size_t len, unsigned char out[20]) {
        uint32_t h0 = 0x67452301, h1 = 0xEFCDAB89, h2 = 0x98BADCFE,
                 h3 = 0x10325476, h4 = 0xC3D2E1F0;

        uint64_t bit_len = (uint64_t)len * 8;
        size_t padded_len = len + 1;
        while (padded_len % 64 != 56) padded_len++;
        padded_len += 8;

        std::vector<unsigned char> p(padded_len, 0);
        std::memcpy(p.data(), data, len);
        p[len] = 0x80;
        for (int i = 0; i < 8; i++) {
            p[padded_len - 1 - i] = (unsigned char)((bit_len >> (i * 8)) & 0xFF);
        }

        for (size_t c = 0; c < padded_len; c += 64) {
            uint32_t w[80];
            for (int i = 0; i < 16; i++) {
                w[i] = ((uint32_t)p[c + i*4] << 24) |
                       ((uint32_t)p[c + i*4 + 1] << 16) |
                       ((uint32_t)p[c + i*4 + 2] << 8)  |
                       ((uint32_t)p[c + i*4 + 3]);
            }
            for (int i = 16; i < 80; i++) {
                uint32_t v = w[i-3] ^ w[i-8] ^ w[i-14] ^ w[i-16];
                w[i] = (v << 1) | (v >> 31);
            }
            uint32_t a = h0, b = h1, cc = h2, d = h3, e = h4;
            for (int i = 0; i < 80; i++) {
                uint32_t f, k;
                if (i < 20)      { f = (b & cc) | ((~b) & d); k = 0x5A827999; }
                else if (i < 40) { f = b ^ cc ^ d;             k = 0x6ED9EBA1; }
                else if (i < 60) { f = (b & cc) | (b & d) | (cc & d); k = 0x8F1BBCDC; }
                else             { f = b ^ cc ^ d;             k = 0xCA62C1D6; }
                uint32_t tmp = ((a << 5) | (a >> 27)) + f + e + k + w[i];
                e = d; d = cc; cc = (b << 30) | (b >> 2); b = a; a = tmp;
            }
            h0 += a; h1 += b; h2 += cc; h3 += d; h4 += e;
        }

        uint32_t hs[5] = {h0, h1, h2, h3, h4};
        for (int i = 0; i < 5; i++) {
            out[i*4]     = (unsigned char)((hs[i] >> 24) & 0xFF);
            out[i*4 + 1] = (unsigned char)((hs[i] >> 16) & 0xFF);
            out[i*4 + 2] = (unsigned char)((hs[i] >> 8) & 0xFF);
            out[i*4 + 3] = (unsigned char)(hs[i] & 0xFF);
        }
    }
};

#endif
