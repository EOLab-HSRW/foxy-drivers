/*
   battery.h - tiny header-only C client for the foxy-battery-monitor service

   This is a small embeddable C/C++ client for the service API exposed by
   battery_monitor.py:

   Source code of the whole package that this file belong to:
   https://github.com/EOLab-HSRW/foxy-drivers

      status file:    /run/foxy-battery-monitor/status
      control socket: /run/foxy-battery-monitor/control.sock

   Usage:

      #define FOXY_BATTERY_IMPLEMENTATION
      #include "battery.h"

      int main(void) {
          foxy_battery_status s;
          if (foxy_battery_get(&s) == FOXY_BATTERY_OK && s.online) {
              printf("battery: %d%%\n", s.percent);
          }
          return 0;
      }

   Only one translation unit should define FOXY_BATTERY_IMPLEMENTATION, unless
   FOXY_BATTERY_STATIC is defined before including this header.

   Define FOXY_BATTERY_STATIC before including this header to make all API functions
   static and private to the translation unit.

   This file depends only on C/POSIX. It targets Linux because foxy-battery-monitor
   (the service) exposes a Unix-domain socket.

*/

#ifndef FOXY_BATTERY_H_INCLUDED
#define FOXY_BATTERY_H_INCLUDED

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>

#ifndef FOXY_BATTERY_STATUS_PATH
#define FOXY_BATTERY_STATUS_PATH "/run/foxy-battery-monitor/status"
#endif

#ifndef FOXY_BATTERY_CONTROL_SOCKET_PATH
#define FOXY_BATTERY_CONTROL_SOCKET_PATH "/run/foxy-battery-monitor/control.sock"
#endif

#ifndef FOXY_BATTERY_STATUS_LINE_MAX
#define FOXY_BATTERY_STATUS_LINE_MAX 512
#endif

#ifndef FOXY_BATTERY_RESPONSE_MAX
#define FOXY_BATTERY_RESPONSE_MAX 512
#endif

#ifndef FOXY_BATTERY_DEFAULT_TIMEOUT_MS
#define FOXY_BATTERY_DEFAULT_TIMEOUT_MS 1000
#endif

#ifdef FOXY_BATTERY_STATIC
#define FOXY_BATTERY_API static
#else
#define FOXY_BATTERY_API extern
#endif

typedef enum foxy_battery_result {
    FOXY_BATTERY_OK        =  0,
    FOXY_BATTERY_EINVAL    = -1,
    FOXY_BATTERY_EIO       = -2,
    FOXY_BATTERY_EPARSE    = -3,
    FOXY_BATTERY_ESOCKET   = -4,
    FOXY_BATTERY_ETIMEOUT  = -5,
    FOXY_BATTERY_EPROTO    = -6,
    FOXY_BATTERY_EOVERFLOW = -7
} foxy_battery_result;

typedef struct foxy_battery_status {
    int online;
    int stale;
    int percent;              /* -1 when unavailable */
    int voltage_mv;
    int current_ma;
    int temperature_mc;       /* milli-degrees Celsius */
    int time_to_empty_min;    /* -1 when unavailable */
    int cycle_count;          /* -1 when unavailable */
    int usb_out_1_mv;
    int usb_out_2_mv;
    int charger_voltage_mv;
    int age_ms;               /* -1 when no packet was seen */
    int packet_count;
    char error[64];           /* optional daemon error token; empty when absent */
} foxy_battery_status;

FOXY_BATTERY_API const char *foxy_battery_strerror(int code);
FOXY_BATTERY_API void foxy_battery_status_init(foxy_battery_status *out_status);
FOXY_BATTERY_API int foxy_battery_parse_status(const char *line, foxy_battery_status *out_status);
FOXY_BATTERY_API int foxy_battery_read_status_file(const char *path, foxy_battery_status *out_status);

/* Fast path: read FOXY_BATTERY_STATUS_PATH. This avoids opening the control socket. */
FOXY_BATTERY_API int foxy_battery_get(foxy_battery_status *out_status);

/* Low-level control socket command. response is always NUL-terminated on success. */
FOXY_BATTERY_API int foxy_battery_control_command_timeout(const char *socket_path,
                                                  const char *command,
                                                  char *response,
                                                  size_t response_capacity,
                                                  int timeout_ms);

FOXY_BATTERY_API int foxy_battery_control_command(const char *socket_path,
                                          const char *command,
                                          char *response,
                                          size_t response_capacity);

/* Higher-level control socket helpers. Pass NULL for socket_path to use default. */
FOXY_BATTERY_API int foxy_battery_ping(const char *socket_path);
FOXY_BATTERY_API int foxy_battery_control_get(const char *socket_path, foxy_battery_status *out_status);
FOXY_BATTERY_API int foxy_battery_shutdown(const char *socket_path);
FOXY_BATTERY_API int foxy_battery_version(const char *socket_path,
                                  char *daemon_version,
                                  size_t daemon_version_capacity,
                                  int *protocol_version);

#ifdef __cplusplus
}
#endif
#endif /* FOXY_BATTERY_H_INCLUDED */

#ifdef FOXY_BATTERY_IMPLEMENTATION

#include <errno.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/un.h>
#include <unistd.h>

#if defined(__GNUC__) || defined(__clang__)
#define FOXY_BATTERY_UNUSED(x) (void)(x)
#else
#define FOXY_BATTERY_UNUSED(x) (void)(x)
#endif

static int foxy_battery__is_space(char c)
{
    return c == ' ' || c == '\t' || c == '\r' || c == '\n' || c == '\v' || c == '\f';
}

static int foxy_battery__key_equals(const char *key, size_t key_len, const char *literal)
{
    return strlen(literal) == key_len && memcmp(key, literal, key_len) == 0;
}

static int foxy_battery__parse_int_span(const char *value, size_t value_len, int *out_value)
{
    char tmp[32];
    char *end = 0;
    long v;

    if (!value || !out_value || value_len == 0 || value_len >= sizeof(tmp)) return FOXY_BATTERY_EPARSE;

    memcpy(tmp, value, value_len);
    tmp[value_len] = '\0';

    errno = 0;
    v = strtol(tmp, &end, 10);
    if (errno != 0 || end == tmp || *end != '\0') return FOXY_BATTERY_EPARSE;

#if defined(INT_MAX) && defined(INT_MIN)
    if (v > INT_MAX || v < INT_MIN) return FOXY_BATTERY_EOVERFLOW;
#endif

    *out_value = (int)v;
    return FOXY_BATTERY_OK;
}

static void foxy_battery__copy_span(char *dst, size_t dst_cap, const char *src, size_t src_len)
{
    size_t n;
    if (!dst || dst_cap == 0) return;
    if (!src) {
        dst[0] = '\0';
        return;
    }
    n = src_len;
    if (n >= dst_cap) n = dst_cap - 1;
    memcpy(dst, src, n);
    dst[n] = '\0';
}

static int foxy_battery__parse_token(const char *key, size_t key_len,
                                 const char *value, size_t value_len,
                                 foxy_battery_status *s)
{
    int v = 0;

#define FOXY_BATTERY_PARSE_INT_FIELD(name_literal, field_name)                         \
    do {                                                                          \
        if (foxy_battery__key_equals(key, key_len, name_literal)) {                   \
            int rc__ = foxy_battery__parse_int_span(value, value_len, &v);            \
            if (rc__ != FOXY_BATTERY_OK) return rc__;                                \
            s->field_name = v;                                                    \
            return FOXY_BATTERY_OK;                                                   \
        }                                                                         \
    } while (0)

    FOXY_BATTERY_PARSE_INT_FIELD("online", online);
    FOXY_BATTERY_PARSE_INT_FIELD("stale", stale);
    FOXY_BATTERY_PARSE_INT_FIELD("percent", percent);
    FOXY_BATTERY_PARSE_INT_FIELD("voltage_mv", voltage_mv);
    FOXY_BATTERY_PARSE_INT_FIELD("current_ma", current_ma);
    FOXY_BATTERY_PARSE_INT_FIELD("temperature_mc", temperature_mc);
    FOXY_BATTERY_PARSE_INT_FIELD("time_to_empty_min", time_to_empty_min);
    FOXY_BATTERY_PARSE_INT_FIELD("cycle_count", cycle_count);
    FOXY_BATTERY_PARSE_INT_FIELD("usb_out_1_mv", usb_out_1_mv);
    FOXY_BATTERY_PARSE_INT_FIELD("usb_out_2_mv", usb_out_2_mv);
    FOXY_BATTERY_PARSE_INT_FIELD("charger_voltage_mv", charger_voltage_mv);
    FOXY_BATTERY_PARSE_INT_FIELD("age_ms", age_ms);
    FOXY_BATTERY_PARSE_INT_FIELD("packet_count", packet_count);

#undef FOXY_BATTERY_PARSE_INT_FIELD

    if (foxy_battery__key_equals(key, key_len, "error")) {
        foxy_battery__copy_span(s->error, sizeof(s->error), value, value_len);
        return FOXY_BATTERY_OK;
    }

    /* Forward-compatible: ignore unknown key=value fields. */
    return FOXY_BATTERY_OK;
}

FOXY_BATTERY_API const char *foxy_battery_strerror(int code)
{
    switch (code) {
        case FOXY_BATTERY_OK:        return "ok";
        case FOXY_BATTERY_EINVAL:    return "invalid argument";
        case FOXY_BATTERY_EIO:       return "I/O error";
        case FOXY_BATTERY_EPARSE:    return "parse error";
        case FOXY_BATTERY_ESOCKET:   return "socket error";
        case FOXY_BATTERY_ETIMEOUT:  return "timeout";
        case FOXY_BATTERY_EPROTO:    return "daemon protocol error";
        case FOXY_BATTERY_EOVERFLOW: return "buffer or numeric overflow";
        default:                 return "unknown foxy_battery error";
    }
}

FOXY_BATTERY_API void foxy_battery_status_init(foxy_battery_status *s)
{
    if (!s) return;
    s->online = 0;
    s->stale = 1;
    s->percent = -1;
    s->voltage_mv = 0;
    s->current_ma = 0;
    s->temperature_mc = 0;
    s->time_to_empty_min = -1;
    s->cycle_count = -1;
    s->usb_out_1_mv = 0;
    s->usb_out_2_mv = 0;
    s->charger_voltage_mv = 0;
    s->age_ms = -1;
    s->packet_count = 0;
    s->error[0] = '\0';
}

FOXY_BATTERY_API int foxy_battery_parse_status(const char *line, foxy_battery_status *out_status)
{
    const char *p;
    int saw_field = 0;

    if (!line || !out_status) return FOXY_BATTERY_EINVAL;
    foxy_battery_status_init(out_status);

    p = line;

    while (foxy_battery__is_space(*p)) ++p;

    /* Accept either "online=..." or a control-socket GET response: "OK online=...". */
    if (p[0] == 'O' && p[1] == 'K' && foxy_battery__is_space(p[2])) {
        p += 2;
    }

    while (*p) {
        const char *key;
        const char *eq;
        const char *value;
        const char *end;
        int rc;

        while (foxy_battery__is_space(*p)) ++p;
        if (!*p) break;

        key = p;
        eq = key;
        while (*eq && *eq != '=' && !foxy_battery__is_space(*eq)) ++eq;

        if (*eq != '=') return FOXY_BATTERY_EPARSE;

        value = eq + 1;
        end = value;
        while (*end && !foxy_battery__is_space(*end)) ++end;

        if (eq == key || end == value) return FOXY_BATTERY_EPARSE;

        rc = foxy_battery__parse_token(key, (size_t)(eq - key), value, (size_t)(end - value), out_status);
        if (rc != FOXY_BATTERY_OK) return rc;

        saw_field = 1;
        p = end;
    }

    return saw_field ? FOXY_BATTERY_OK : FOXY_BATTERY_EPARSE;
}

FOXY_BATTERY_API int foxy_battery_read_status_file(const char *path, foxy_battery_status *out_status)
{
    char line[FOXY_BATTERY_STATUS_LINE_MAX];
    FILE *f;
    int rc;

    if (!out_status) return FOXY_BATTERY_EINVAL;
    if (!path) path = FOXY_BATTERY_STATUS_PATH;

    f = fopen(path, "rb");
    if (!f) return FOXY_BATTERY_EIO;

    if (!fgets(line, sizeof(line), f)) {
        fclose(f);
        return FOXY_BATTERY_EIO;
    }

    if (!strchr(line, '\n') && !feof(f)) {
        fclose(f);
        return FOXY_BATTERY_EOVERFLOW;
    }

    fclose(f);
    rc = foxy_battery_parse_status(line, out_status);
    return rc;
}

FOXY_BATTERY_API int foxy_battery_get(foxy_battery_status *out_status)
{
    return foxy_battery_read_status_file(FOXY_BATTERY_STATUS_PATH, out_status);
}

static int foxy_battery__set_timeouts(int fd, int timeout_ms)
{
    struct timeval tv;

    if (timeout_ms < 0) return FOXY_BATTERY_OK;

    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;

    if (setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, (const char *)&tv, sizeof(tv)) != 0) return FOXY_BATTERY_ESOCKET;
    if (setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, (const char *)&tv, sizeof(tv)) != 0) return FOXY_BATTERY_ESOCKET;

    return FOXY_BATTERY_OK;
}

FOXY_BATTERY_API int foxy_battery_control_command_timeout(const char *socket_path,
                                                  const char *command,
                                                  char *response,
                                                  size_t response_capacity,
                                                  int timeout_ms)
{
    int fd;
    struct sockaddr_un addr;
    size_t command_len;
    size_t used = 0;

    if (!command || !response || response_capacity == 0) return FOXY_BATTERY_EINVAL;
    if (!socket_path) socket_path = FOXY_BATTERY_CONTROL_SOCKET_PATH;

    response[0] = '\0';
    command_len = strlen(command);
    if (command_len == 0 || command_len > 128) return FOXY_BATTERY_EINVAL;
    if (strlen(socket_path) >= sizeof(addr.sun_path)) return FOXY_BATTERY_EINVAL;

    fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) return FOXY_BATTERY_ESOCKET;

    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, socket_path, sizeof(addr.sun_path) - 1);

    if (foxy_battery__set_timeouts(fd, timeout_ms) != FOXY_BATTERY_OK) {
        close(fd);
        return FOXY_BATTERY_ESOCKET;
    }

    if (connect(fd, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
        int saved_errno = errno;
        close(fd);
        FOXY_BATTERY_UNUSED(saved_errno);
        return (saved_errno == EAGAIN || saved_errno == EWOULDBLOCK || saved_errno == ETIMEDOUT) ? FOXY_BATTERY_ETIMEOUT : FOXY_BATTERY_ESOCKET;
    }

    while (command_len > 0) {
        ssize_t n = send(fd, command, command_len, 0);
        if (n < 0) {
            int saved_errno = errno;
            close(fd);
            return (saved_errno == EAGAIN || saved_errno == EWOULDBLOCK || saved_errno == ETIMEDOUT) ? FOXY_BATTERY_ETIMEOUT : FOXY_BATTERY_ESOCKET;
        }
        if (n == 0) {
            close(fd);
            return FOXY_BATTERY_EIO;
        }
        command += (size_t)n;
        command_len -= (size_t)n;
    }

    while (used + 1 < response_capacity) {
        ssize_t n = recv(fd, response + used, response_capacity - 1 - used, 0);
        if (n < 0) {
            int saved_errno = errno;
            close(fd);
            response[used] = '\0';
            return (saved_errno == EAGAIN || saved_errno == EWOULDBLOCK || saved_errno == ETIMEDOUT) ? FOXY_BATTERY_ETIMEOUT : FOXY_BATTERY_ESOCKET;
        }
        if (n == 0) break;
        used += (size_t)n;
        response[used] = '\0';
        if (memchr(response, '\n', used)) break;
    }

    close(fd);

    if (used == 0) return FOXY_BATTERY_EIO;
    if (used + 1 >= response_capacity && !memchr(response, '\n', used)) return FOXY_BATTERY_EOVERFLOW;

    response[used] = '\0';
    return FOXY_BATTERY_OK;
}

FOXY_BATTERY_API int foxy_battery_control_command(const char *socket_path,
                                          const char *command,
                                          char *response,
                                          size_t response_capacity)
{
    return foxy_battery_control_command_timeout(socket_path, command, response, response_capacity, FOXY_BATTERY_DEFAULT_TIMEOUT_MS);
}

static int foxy_battery__reply_is_ok(const char *response)
{
    return response && response[0] == 'O' && response[1] == 'K' &&
           (response[2] == '\0' || foxy_battery__is_space(response[2]));
}

FOXY_BATTERY_API int foxy_battery_ping(const char *socket_path)
{
    char response[FOXY_BATTERY_RESPONSE_MAX];
    int rc = foxy_battery_control_command(socket_path, "PING", response, sizeof(response));
    if (rc != FOXY_BATTERY_OK) return rc;
    return foxy_battery__reply_is_ok(response) ? FOXY_BATTERY_OK : FOXY_BATTERY_EPROTO;
}

FOXY_BATTERY_API int foxy_battery_control_get(const char *socket_path, foxy_battery_status *out_status)
{
    char response[FOXY_BATTERY_RESPONSE_MAX];
    int rc;

    if (!out_status) return FOXY_BATTERY_EINVAL;

    rc = foxy_battery_control_command(socket_path, "GET", response, sizeof(response));
    if (rc != FOXY_BATTERY_OK) return rc;
    if (!foxy_battery__reply_is_ok(response)) return FOXY_BATTERY_EPROTO;

    return foxy_battery_parse_status(response, out_status);
}

FOXY_BATTERY_API int foxy_battery_shutdown(const char *socket_path)
{
    char response[FOXY_BATTERY_RESPONSE_MAX];
    int rc = foxy_battery_control_command(socket_path, "SHUTDOWN", response, sizeof(response));
    if (rc != FOXY_BATTERY_OK) return rc;
    return foxy_battery__reply_is_ok(response) ? FOXY_BATTERY_OK : FOXY_BATTERY_EPROTO;
}

FOXY_BATTERY_API int foxy_battery_version(const char *socket_path,
                                  char *daemon_version,
                                  size_t daemon_version_capacity,
                                  int *protocol_version)
{
    char response[FOXY_BATTERY_RESPONSE_MAX];
    const char *p;
    int rc;
    int saw_ok = 0;

    if ((daemon_version_capacity > 0 && !daemon_version) || !protocol_version) return FOXY_BATTERY_EINVAL;
    if (daemon_version && daemon_version_capacity > 0) daemon_version[0] = '\0';
    *protocol_version = 0;

    rc = foxy_battery_control_command(socket_path, "VERSION", response, sizeof(response));
    if (rc != FOXY_BATTERY_OK) return rc;
    if (!foxy_battery__reply_is_ok(response)) return FOXY_BATTERY_EPROTO;

    p = response;
    while (foxy_battery__is_space(*p)) ++p;
    if (p[0] == 'O' && p[1] == 'K') {
        p += 2;
        saw_ok = 1;
    }
    if (!saw_ok) return FOXY_BATTERY_EPROTO;

    while (*p) {
        const char *key;
        const char *eq;
        const char *value;
        const char *end;

        while (foxy_battery__is_space(*p)) ++p;
        if (!*p) break;

        key = p;
        eq = key;
        while (*eq && *eq != '=' && !foxy_battery__is_space(*eq)) ++eq;
        if (*eq != '=') return FOXY_BATTERY_EPROTO;

        value = eq + 1;
        end = value;
        while (*end && !foxy_battery__is_space(*end)) ++end;

        if (foxy_battery__key_equals(key, (size_t)(eq - key), "foxy-battery-monitor")) {
            foxy_battery__copy_span(daemon_version, daemon_version_capacity, value, (size_t)(end - value));
        } else if (foxy_battery__key_equals(key, (size_t)(eq - key), "protocol")) {
            rc = foxy_battery__parse_int_span(value, (size_t)(end - value), protocol_version);
            if (rc != FOXY_BATTERY_OK) return rc;
        }

        p = end;
    }

    return FOXY_BATTERY_OK;
}

#endif /* FOXY_BATTERY_IMPLEMENTATION */
