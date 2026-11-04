#ifndef MYAGV_WEB_SERIAL_READ_H
#define MYAGV_WEB_SERIAL_READ_H
#include <cerrno>
#include <chrono>
#include <fcntl.h>
#include <poll.h>
#include <unistd.h>

// One deadline covers the entire frame, including header search and partial reads.
inline size_t myagvReadExact(int fd, unsigned char* data, size_t size,
                            std::chrono::steady_clock::time_point deadline) {
    int flags = fcntl(fd, F_GETFL);
    if (flags < 0 || fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0) return 0;
    struct RestoreFlags {
        int fd, flags;
        ~RestoreFlags() { fcntl(fd, F_SETFL, flags); }
    } restore{fd, flags};
    size_t received = 0;
    while (received < size) {
        auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
            deadline - std::chrono::steady_clock::now()).count();
        if (remaining <= 0) return 0;
        pollfd item{fd, POLLIN, 0};
        int ready = poll(&item, 1, static_cast<int>(remaining));
        if (ready < 0 && errno == EINTR) continue;
        if (ready <= 0 || (item.revents & (POLLERR | POLLHUP | POLLNVAL))) return 0;
        ssize_t n = read(fd, data + received, size - received);
        if (n < 0 && (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)) continue;
        if (n <= 0) return 0;
        received += static_cast<size_t>(n);
    }
    return received;
}
#endif
