/*
   batteryd.h - tiny header-only C client for the batteryd daemon

   This is a small embeddable C/C++ client for the daemon API exposed by
   batteryd.py:

      status file:    /run/batteryd/status
      control socket: /run/batteryd/control.sock

   Usage:

      #define BATTERYD_IMPLEMENTATION
      #include "batteryd.h"

      int main(void) {
          batteryd_status s;
          if (batteryd_get(&s) == BATTERYD_OK && s.online) {
              printf("battery: %d%%\n", s.percent);
          }
          return 0;
      }

   Only one translation unit should define BATTERYD_IMPLEMENTATION, unless
   BATTERYD_STATIC is defined before including this header.

   Define BATTERYD_STATIC before including this header to make all API functions
   static and private to the translation unit.

   This file depends only on C/POSIX. It targets Linux because batteryd exposes
   a Unix-domain socket.

*/

#ifndef BATTERYD_H_INCLUDED
#define BATTERYD_H_INCLUDED

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>

#ifndef BATTERYD_STATUS_PATH
#define BATTERYD_STATUS_PATH "/run/batteryd/status"
#endif

#ifndef BATTERYD_CONTROL_SOCKET_PATH
#define BATTERYD_CONTROL_SOCKET_PATH "/run/batteryd/control.sock"
#endif

#ifndef BATTERYD_STATUS_LINE_MAX
#define BATTERYD_STATUS_LINE_MAX 512
#endif

#ifndef BATTERYD_RESPONSE_MAX
#define BATTERYD_RESPONSE_MAX 512
#endif

#ifndef BATTERYD_DEFAULT_TIMEOUT_MS
#define BATTERYD_DEFAULT_TIMEOUT_MS 1000
#endif

#ifdef BATTERYD_STATIC
#define BATTERYD_API static
#else
#define BATTERYD_API extern
#endif

typedef enum batteryd_result {
    BATTERYD_OK        =  0,
    BATTERYD_EINVAL    = -1,
    BATTERYD_EIO       = -2,
    BATTERYD_EPARSE    = -3,
    BATTERYD_ESOCKET   = -4,
    BATTERYD_ETIMEOUT  = -5,
    BATTERYD_EPROTO    = -6,
    BATTERYD_EOVERFLOW = -7
} batteryd_result;

typedef struct batteryd_status {
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
} batteryd_status;

BATTERYD_API const char *batteryd_strerror(int code);
BATTERYD_API void batteryd_status_init(batteryd_status *out_status);
BATTERYD_API int batteryd_parse_status(const char *line, batteryd_status *out_status);
BATTERYD_API int batteryd_read_status_file(const char *path, batteryd_status *out_status);

/* Fast path: read BATTERYD_STATUS_PATH. This avoids opening the control socket. */
BATTERYD_API int batteryd_get(batteryd_status *out_status);

/* Low-level control socket command. response is always NUL-terminated on success. */
BATTERYD_API int batteryd_control_command_timeout(const char *socket_path,
                                                  const char *command,
                                                  char *response,
                                                  size_t response_capacity,
                                                  int timeout_ms);

BATTERYD_API int batteryd_control_command(const char *socket_path,
                                          const char *command,
                                          char *response,
                                          size_t response_capacity);

/* Higher-level control socket helpers. Pass NULL for socket_path to use default. */
BATTERYD_API int batteryd_ping(const char *socket_path);
BATTERYD_API int batteryd_control_get(const char *socket_path, batteryd_status *out_status);
BATTERYD_API int batteryd_shutdown(const char *socket_path);
BATTERYD_API int batteryd_version(const char *socket_path,
                                  char *daemon_version,
                                  size_t daemon_version_capacity,
                                  int *protocol_version);

#ifdef __cplusplus
}
#endif
#endif /* BATTERYD_H_INCLUDED */

#ifdef BATTERYD_IMPLEMENTATION

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
#define BATTERYD_UNUSED(x) (void)(x)
#else
#define BATTERYD_UNUSED(x) (void)(x)
#endif

static int batteryd__is_space(char c)
{
    return c == ' ' || c == '\t' || c == '\r' || c == '\n' || c == '\v' || c == '\f';
}

static int batteryd__key_equals(const char *key, size_t key_len, const char *literal)
{
    return strlen(literal) == key_len && memcmp(key, literal, key_len) == 0;
}

static int batteryd__parse_int_span(const char *value, size_t value_len, int *out_value)
{
    char tmp[32];
    char *end = 0;
    long v;

    if (!value || !out_value || value_len == 0 || value_len >= sizeof(tmp)) return BATTERYD_EPARSE;

    memcpy(tmp, value, value_len);
    tmp[value_len] = '\0';

    errno = 0;
    v = strtol(tmp, &end, 10);
    if (errno != 0 || end == tmp || *end != '\0') return BATTERYD_EPARSE;

#if defined(INT_MAX) && defined(INT_MIN)
    if (v > INT_MAX || v < INT_MIN) return BATTERYD_EOVERFLOW;
#endif

    *out_value = (int)v;
    return BATTERYD_OK;
}

static void batteryd__copy_span(char *dst, size_t dst_cap, const char *src, size_t src_len)
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

static int batteryd__parse_token(const char *key, size_t key_len,
                                 const char *value, size_t value_len,
                                 batteryd_status *s)
{
    int v = 0;

#define BATTERYD_PARSE_INT_FIELD(name_literal, field_name)                         \
    do {                                                                          \
        if (batteryd__key_equals(key, key_len, name_literal)) {                   \
            int rc__ = batteryd__parse_int_span(value, value_len, &v);            \
            if (rc__ != BATTERYD_OK) return rc__;                                \
            s->field_name = v;                                                    \
            return BATTERYD_OK;                                                   \
        }                                                                         \
    } while (0)

    BATTERYD_PARSE_INT_FIELD("online", online);
    BATTERYD_PARSE_INT_FIELD("stale", stale);
    BATTERYD_PARSE_INT_FIELD("percent", percent);
    BATTERYD_PARSE_INT_FIELD("voltage_mv", voltage_mv);
    BATTERYD_PARSE_INT_FIELD("current_ma", current_ma);
    BATTERYD_PARSE_INT_FIELD("temperature_mc", temperature_mc);
    BATTERYD_PARSE_INT_FIELD("time_to_empty_min", time_to_empty_min);
    BATTERYD_PARSE_INT_FIELD("cycle_count", cycle_count);
    BATTERYD_PARSE_INT_FIELD("usb_out_1_mv", usb_out_1_mv);
    BATTERYD_PARSE_INT_FIELD("usb_out_2_mv", usb_out_2_mv);
    BATTERYD_PARSE_INT_FIELD("charger_voltage_mv", charger_voltage_mv);
    BATTERYD_PARSE_INT_FIELD("age_ms", age_ms);
    BATTERYD_PARSE_INT_FIELD("packet_count", packet_count);

#undef BATTERYD_PARSE_INT_FIELD

    if (batteryd__key_equals(key, key_len, "error")) {
        batteryd__copy_span(s->error, sizeof(s->error), value, value_len);
        return BATTERYD_OK;
    }

    /* Forward-compatible: ignore unknown key=value fields. */
    return BATTERYD_OK;
}

BATTERYD_API const char *batteryd_strerror(int code)
{
    switch (code) {
        case BATTERYD_OK:        return "ok";
        case BATTERYD_EINVAL:    return "invalid argument";
        case BATTERYD_EIO:       return "I/O error";
        case BATTERYD_EPARSE:    return "parse error";
        case BATTERYD_ESOCKET:   return "socket error";
        case BATTERYD_ETIMEOUT:  return "timeout";
        case BATTERYD_EPROTO:    return "daemon protocol error";
        case BATTERYD_EOVERFLOW: return "buffer or numeric overflow";
        default:                 return "unknown batteryd error";
    }
}

BATTERYD_API void batteryd_status_init(batteryd_status *s)
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

BATTERYD_API int batteryd_parse_status(const char *line, batteryd_status *out_status)
{
    const char *p;
    int saw_field = 0;

    if (!line || !out_status) return BATTERYD_EINVAL;
    batteryd_status_init(out_status);

    p = line;

    while (batteryd__is_space(*p)) ++p;

    /* Accept either "online=..." or a control-socket GET response: "OK online=...". */
    if (p[0] == 'O' && p[1] == 'K' && batteryd__is_space(p[2])) {
        p += 2;
    }

    while (*p) {
        const char *key;
        const char *eq;
        const char *value;
        const char *end;
        int rc;

        while (batteryd__is_space(*p)) ++p;
        if (!*p) break;

        key = p;
        eq = key;
        while (*eq && *eq != '=' && !batteryd__is_space(*eq)) ++eq;

        if (*eq != '=') return BATTERYD_EPARSE;

        value = eq + 1;
        end = value;
        while (*end && !batteryd__is_space(*end)) ++end;

        if (eq == key || end == value) return BATTERYD_EPARSE;

        rc = batteryd__parse_token(key, (size_t)(eq - key), value, (size_t)(end - value), out_status);
        if (rc != BATTERYD_OK) return rc;

        saw_field = 1;
        p = end;
    }

    return saw_field ? BATTERYD_OK : BATTERYD_EPARSE;
}

BATTERYD_API int batteryd_read_status_file(const char *path, batteryd_status *out_status)
{
    char line[BATTERYD_STATUS_LINE_MAX];
    FILE *f;
    int rc;

    if (!out_status) return BATTERYD_EINVAL;
    if (!path) path = BATTERYD_STATUS_PATH;

    f = fopen(path, "rb");
    if (!f) return BATTERYD_EIO;

    if (!fgets(line, sizeof(line), f)) {
        fclose(f);
        return BATTERYD_EIO;
    }

    if (!strchr(line, '\n') && !feof(f)) {
        fclose(f);
        return BATTERYD_EOVERFLOW;
    }

    fclose(f);
    rc = batteryd_parse_status(line, out_status);
    return rc;
}

BATTERYD_API int batteryd_get(batteryd_status *out_status)
{
    return batteryd_read_status_file(BATTERYD_STATUS_PATH, out_status);
}

static int batteryd__set_timeouts(int fd, int timeout_ms)
{
    struct timeval tv;

    if (timeout_ms < 0) return BATTERYD_OK;

    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;

    if (setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, (const char *)&tv, sizeof(tv)) != 0) return BATTERYD_ESOCKET;
    if (setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, (const char *)&tv, sizeof(tv)) != 0) return BATTERYD_ESOCKET;

    return BATTERYD_OK;
}

BATTERYD_API int batteryd_control_command_timeout(const char *socket_path,
                                                  const char *command,
                                                  char *response,
                                                  size_t response_capacity,
                                                  int timeout_ms)
{
    int fd;
    struct sockaddr_un addr;
    size_t command_len;
    size_t used = 0;

    if (!command || !response || response_capacity == 0) return BATTERYD_EINVAL;
    if (!socket_path) socket_path = BATTERYD_CONTROL_SOCKET_PATH;

    response[0] = '\0';
    command_len = strlen(command);
    if (command_len == 0 || command_len > 128) return BATTERYD_EINVAL;
    if (strlen(socket_path) >= sizeof(addr.sun_path)) return BATTERYD_EINVAL;

    fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) return BATTERYD_ESOCKET;

    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, socket_path, sizeof(addr.sun_path) - 1);

    if (batteryd__set_timeouts(fd, timeout_ms) != BATTERYD_OK) {
        close(fd);
        return BATTERYD_ESOCKET;
    }

    if (connect(fd, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
        int saved_errno = errno;
        close(fd);
        BATTERYD_UNUSED(saved_errno);
        return (saved_errno == EAGAIN || saved_errno == EWOULDBLOCK || saved_errno == ETIMEDOUT) ? BATTERYD_ETIMEOUT : BATTERYD_ESOCKET;
    }

    while (command_len > 0) {
        ssize_t n = send(fd, command, command_len, 0);
        if (n < 0) {
            int saved_errno = errno;
            close(fd);
            return (saved_errno == EAGAIN || saved_errno == EWOULDBLOCK || saved_errno == ETIMEDOUT) ? BATTERYD_ETIMEOUT : BATTERYD_ESOCKET;
        }
        if (n == 0) {
            close(fd);
            return BATTERYD_EIO;
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
            return (saved_errno == EAGAIN || saved_errno == EWOULDBLOCK || saved_errno == ETIMEDOUT) ? BATTERYD_ETIMEOUT : BATTERYD_ESOCKET;
        }
        if (n == 0) break;
        used += (size_t)n;
        response[used] = '\0';
        if (memchr(response, '\n', used)) break;
    }

    close(fd);

    if (used == 0) return BATTERYD_EIO;
    if (used + 1 >= response_capacity && !memchr(response, '\n', used)) return BATTERYD_EOVERFLOW;

    response[used] = '\0';
    return BATTERYD_OK;
}

BATTERYD_API int batteryd_control_command(const char *socket_path,
                                          const char *command,
                                          char *response,
                                          size_t response_capacity)
{
    return batteryd_control_command_timeout(socket_path, command, response, response_capacity, BATTERYD_DEFAULT_TIMEOUT_MS);
}

static int batteryd__reply_is_ok(const char *response)
{
    return response && response[0] == 'O' && response[1] == 'K' &&
           (response[2] == '\0' || batteryd__is_space(response[2]));
}

BATTERYD_API int batteryd_ping(const char *socket_path)
{
    char response[BATTERYD_RESPONSE_MAX];
    int rc = batteryd_control_command(socket_path, "PING", response, sizeof(response));
    if (rc != BATTERYD_OK) return rc;
    return batteryd__reply_is_ok(response) ? BATTERYD_OK : BATTERYD_EPROTO;
}

BATTERYD_API int batteryd_control_get(const char *socket_path, batteryd_status *out_status)
{
    char response[BATTERYD_RESPONSE_MAX];
    int rc;

    if (!out_status) return BATTERYD_EINVAL;

    rc = batteryd_control_command(socket_path, "GET", response, sizeof(response));
    if (rc != BATTERYD_OK) return rc;
    if (!batteryd__reply_is_ok(response)) return BATTERYD_EPROTO;

    return batteryd_parse_status(response, out_status);
}

BATTERYD_API int batteryd_shutdown(const char *socket_path)
{
    char response[BATTERYD_RESPONSE_MAX];
    int rc = batteryd_control_command(socket_path, "SHUTDOWN", response, sizeof(response));
    if (rc != BATTERYD_OK) return rc;
    return batteryd__reply_is_ok(response) ? BATTERYD_OK : BATTERYD_EPROTO;
}

BATTERYD_API int batteryd_version(const char *socket_path,
                                  char *daemon_version,
                                  size_t daemon_version_capacity,
                                  int *protocol_version)
{
    char response[BATTERYD_RESPONSE_MAX];
    const char *p;
    int rc;
    int saw_ok = 0;

    if ((daemon_version_capacity > 0 && !daemon_version) || !protocol_version) return BATTERYD_EINVAL;
    if (daemon_version && daemon_version_capacity > 0) daemon_version[0] = '\0';
    *protocol_version = 0;

    rc = batteryd_control_command(socket_path, "VERSION", response, sizeof(response));
    if (rc != BATTERYD_OK) return rc;
    if (!batteryd__reply_is_ok(response)) return BATTERYD_EPROTO;

    p = response;
    while (batteryd__is_space(*p)) ++p;
    if (p[0] == 'O' && p[1] == 'K') {
        p += 2;
        saw_ok = 1;
    }
    if (!saw_ok) return BATTERYD_EPROTO;

    while (*p) {
        const char *key;
        const char *eq;
        const char *value;
        const char *end;

        while (batteryd__is_space(*p)) ++p;
        if (!*p) break;

        key = p;
        eq = key;
        while (*eq && *eq != '=' && !batteryd__is_space(*eq)) ++eq;
        if (*eq != '=') return BATTERYD_EPROTO;

        value = eq + 1;
        end = value;
        while (*end && !batteryd__is_space(*end)) ++end;

        if (batteryd__key_equals(key, (size_t)(eq - key), "batteryd")) {
            batteryd__copy_span(daemon_version, daemon_version_capacity, value, (size_t)(end - value));
        } else if (batteryd__key_equals(key, (size_t)(eq - key), "protocol")) {
            rc = batteryd__parse_int_span(value, (size_t)(end - value), protocol_version);
            if (rc != BATTERYD_OK) return rc;
        }

        p = end;
    }

    return BATTERYD_OK;
}

#endif /* BATTERYD_IMPLEMENTATION */

