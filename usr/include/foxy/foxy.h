#ifndef FOXY_H_INCLUDED
#define FOXY_H_INCLUDED
/*
 * foxy.h - single-header C library (stb-style)
 *
 * Usage:
 *   In exactly one C/C++ translation unit:
 *     #define FOXY_IMPLEMENTATION
 *     #include "foxy.h"
 *
 *   In all other translation units:
 *     #include "foxy.h"
 *
 * Build example (Linux):
 *   gcc -std=c11 -Wall -Wextra -O2 main.c -o app -lm
 *
 *
 * Organization:
 *  - Public API forward declaration
 *  - Small helper for platform introspection
 *  - Platform, model and Hat functions (platform_t)
 *  - peripherals denitions, errors code and helpers (peripheral_desc_t)
 *  - Robot definition (robot_def_t)
 *  - Linux helper for internal use on this drivers
 *    - i2c
 *    - gpio
 *  - Hardware specific function
 *    - MPU6050
 *    - ...
 *  - Robot (robot_t)
 *  - drivers
 *  - Resource handles - Capabilities, Init, Close and binding to (driver) implementations.
 *  - Drivers registry (global table with available driver)
 */

#ifndef FOXY_API
#define FOXY_API extern
#endif

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/**
 * High-level platform family.
 */
typedef enum {
    PLATFORM_UNKNOWN   = 0,
    PLATFORM_JETSON    = 1,
    PLATFORM_RASPBERRY = 2,
} platform_family_t;

/**
 * Robot HAT (Hardware Attached on Top) revision.
 * @HAT_UNKNOWN: Unknown or not detected.
 * @HAT_V3_15: HAT revision v3.15.
 *
 * The HAT defines the "physical hardware contract" for peripheral
 * connections (IMU, buttons, LEDs, etc.). This enum tracks hardware
 * revisions to ensure correct pinmuxing and peripheral routing.
 */
typedef enum {
    HAT_UNKNOWN = 0,
    HAT_V3_15   = 1,
} hat_t;


// forward declaration
struct peripheral_desc;
typedef struct peripheral_desc peripheral_desc_t;
typedef struct peripheral_driver peripheral_driver_t;

/**
 * Full robot description
 */
typedef struct {
    uint32_t key;
    const char *id;
    const char *display_name;

    platform_family_t platform_family;
    uint16_t platform_model;
    hat_t hat;

    const peripheral_desc_t *peripherals;
    size_t num_peripherals;
} robot_def_t;

typedef struct {
    const peripheral_driver_t *driver;
    void *ctx;
    uint16_t refs;
} robot_slot_t;


/**
 * Harley: any assumption in software development is usually wrong,
 * but let's allow ourselves to be wrong this time, since this small
 * "foxy robot" wouldn't have more peripheral than this.
 *
 * If that's not the case, and the robot has a large a number of peripherals attached,
 * the fundamental principle of simplicity and minimalism that drive the philosophy of this
 * driver are broken at their core. In that situation, a different approach should be consider
 * instead of extending this single header to the point where it becomes fragile and unmaintainable.
 *
 * Therefore, consider this number a complexity cealing that ideally should not be exceeded,
 * and if it is exceeded, that's is a good sign that it's time to rethink the approach.
 * In the best-case scenario, this number should increase only if it isn't accompanied
 * by additional code.
 */
#define ROBOT_MAX_PERIPHERALS 32

typedef struct {
    int error;
    const robot_def_t *def;
    robot_slot_t slots[ROBOT_MAX_PERIPHERALS];
    size_t nslots;
} robot_t;

/* Unified public resource handle */
typedef struct {
    const void *ops;
    void *ctx;
    uint16_t _idx;
} resource_t;

typedef resource_t led_t;
typedef resource_t imu_t;
typedef resource_t gpio_t;
typedef resource_t motor_t;

typedef struct {
    float accel_ms2[3];
    float gyro_dps[3];
    float mag_uT[3];
    float temp_c;
} imu_sample_t;

/* ====== PUBLIC API PROTOTYPES ====== */

FOXY_API void sleep_ms(unsigned ms);

FOXY_API robot_t robot_init(void);
FOXY_API void    robot_deinit(robot_t *r);

static inline int robot_ok(const robot_t *r) { return r && r->error == 0; }

FOXY_API void robot_def_dump(const robot_def_t *def, void *out);

FOXY_API led_t   led_init_name(robot_t *r, const char *name);
FOXY_API void    led_deinit(robot_t *r, led_t *h);
FOXY_API int     led_set_rgb(led_t led, uint8_t idx, uint8_t r, uint8_t g, uint8_t b);

FOXY_API imu_t        imu_init_name(robot_t *r, const char *name);
FOXY_API void         imu_deinit(robot_t *r, imu_t *h);
FOXY_API imu_sample_t imu_read(imu_t imu);

FOXY_API gpio_t gpio_init_name(robot_t *r, const char *name);
FOXY_API void   gpio_deinit(robot_t *r, gpio_t *h);
FOXY_API int    gpio_set_as_input(gpio_t g);
FOXY_API int    gpio_set_as_output(gpio_t g);
FOXY_API int    gpio_set_active_low(gpio_t g);
FOXY_API int    gpio_set_active_high(gpio_t g);
FOXY_API int    gpio_read(gpio_t g);
FOXY_API int    gpio_write(gpio_t g, int value);

FOXY_API motor_t motor_init_name(robot_t *r, const char *name);
FOXY_API void    motor_deinit(robot_t *r, motor_t *m);
FOXY_API int     motor_set(motor_t m, float power);
FOXY_API int     motor_stop(motor_t m);
FOXY_API int     motor_brake(motor_t m);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* FOXY_H_INCLUDED */

/* ============================== IMPLEMENTATION ============================== */
#ifdef FOXY_IMPLEMENTATION
#ifndef FOXY_IMPLEMENTATION_ONCE
#define FOXY_IMPLEMENTATION_ONCE

#ifdef __cplusplus
extern "C" {
#endif

/* Standard C Headers */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

 /* POSIX / System Headers */
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>

#ifndef __linux__
#error "foxy.h supports Linux only (implementation uses Linux i2c/gpio headers)."
#endif

#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/gpio.h>

/**
 * Remove trailing whitespace (newline, carriage return, space, tab).
 * @param s: The string to modify in-place.
 */
static void rstrip(char *s) {
    size_t n;
    if (!s) return;
    n = strlen(s);
    while (n > 0) {
        char c = s[n - 1];
        if (c == '\n' || c == '\r' || c == ' ' || c == '\t')
            s[--n] = '\0';
        else
            break;
    }
}

/**
 * Convert a character to lowercase (ASCII only).
 * @param c: The character to convert.
 *
 * @return: The lowercase equivalent if it is an uppercase letter, otherwise the character itself.
 */
static int ascii_tolower(int c) {
    if (c >= 'A' && c <= 'Z') return c + ('a' - 'A');
    return c;
}

/**
 * Check if a case-insensitive substring exists.
 * @param haystack: The string to search in.
 * @param needle: The substring to search for.
 *
 * @return: 1 if found, 0 otherwise.
 */
static int contains_ci(const char *haystack, const char *needle) {
    size_t hlen, nlen;
    size_t i, j;

    if (!haystack || !needle) return 0;
    hlen = strlen(haystack);
    nlen = strlen(needle);
    if (nlen == 0) return 1;
    if (nlen > hlen) return 0;

    for (i = 0; i + nlen <= hlen; i++) {
        for (j = 0; j < nlen; j++) {
            if (ascii_tolower((unsigned char)haystack[i + j]) !=
                ascii_tolower((unsigned char)needle[j]))
                break;
        }
        if (j == nlen) return 1;
    }
    return 0;
}

static size_t foxy_strnlen(const char *s, size_t maxlen) {
    size_t n = 0;
    if (!s) return 0;
    while (n < maxlen && s[n] != '\0') n++;
    return n;
}

/**
 * Read a file into a buffer.
 * @param path: Path to the file to read.
 * @param buf: Output buffer to store file contents.
 * @param bufsz: Size of the output buffer.
 *
 * @return: Number of bytes read (>=0), or -1 on error.
 */
static ssize_t read_file_bytes(const char *path, uint8_t *buf, size_t bufsz) {
    int fd;
    ssize_t n;

    if (!path || !buf || bufsz == 0) {
        errno = EINVAL;
        return -1;
    }

    fd = open(path, O_RDONLY);
    if (fd < 0) return -1;

    n = read(fd, buf, bufsz);
    close(fd);
    return n;
}

/**
 * Read a Device Tree string property (NUL-terminated in file).
 *
 * @param path: Path to the device tree file (e.g. "/proc/device-tree/model").
 * @param out: Output buffer for the string.
 * @param outsz: Size of the output buffer.
 *
 * @return: 0 on success, -1 on error.
 */
static int read_dt_string(const char *path, char *out, size_t outsz) {
    uint8_t tmp[256];
    ssize_t n;
    size_t len;

    if (!out || outsz == 0) {
        errno = EINVAL;
        return -1;
    }
    out[0] = '\0';

    n = read_file_bytes(path, tmp, sizeof(tmp));
    if (n <= 0) return -1;

    len = 0;
    while (len < (size_t)n && tmp[len] != '\0') len++;

    if (len >= outsz) len = outsz - 1;
    memcpy(out, tmp, len);
    out[len] = '\0';
    rstrip(out);
    return 0;
}

/**
 * Check if "compatible" has a matching string.
 *
 * @param needle: Substring to find (case-insensitive) in the compatible string list.
 *
 * @return: 1 if found, 0 if not found or file not readable.
 */
static int dt_compatible_contains(const char *needle) {
    uint8_t buf[1024];
    ssize_t n = read_file_bytes("/proc/device-tree/compatible", buf, sizeof(buf));
    size_t i = 0;

    if (n <= 0 || !needle) return 0;

    while (i < (size_t)n) {
        const char *s = (const char *)&buf[i];
        size_t max = (size_t)n - i;
        size_t len = foxy_strnlen(s, max);
        if (len == 0) break;

        if (contains_ci(s, needle)) return 1;
        i += len + 1;
    }
    return 0;
}

/* ========================================================================================
 * PLATFORM - Definitions and Detections
 * ======================================================================================== */

static const char *family_to_string(platform_family_t f) {
    switch (f) {
        case PLATFORM_JETSON:    return "jetson";
        case PLATFORM_RASPBERRY: return "raspberry";
        default:                 return "unknown";
    }
}

typedef enum {
    JETSON_MODEL_UNKNOWN   = 0,
    JETSON_MODEL_NANO      = 1,
    JETSON_MODEL_XAVIER_NX = 2,
    JETSON_MODEL_ORIN_NANO = 3,
    JETSON_MODEL_ORIN_NX   = 4,
    JETSON_MODEL_AGX_ORIN  = 5
} jetson_model_t;

static jetson_model_t parse_jetson_model(const char *model_str) {
    if (model_str && contains_ci(model_str, "AGX Orin"))    return JETSON_MODEL_AGX_ORIN;
    if (model_str && contains_ci(model_str, "Orin NX"))     return JETSON_MODEL_ORIN_NX;
    if (model_str && contains_ci(model_str, "Orin Nano"))   return JETSON_MODEL_ORIN_NANO;
    if (model_str && contains_ci(model_str, "Xavier NX"))   return JETSON_MODEL_XAVIER_NX;
    if (model_str && contains_ci(model_str, "Jetson Nano")) return JETSON_MODEL_NANO;

    if (dt_compatible_contains("jetson-agx-orin"))  return JETSON_MODEL_AGX_ORIN;
    if (dt_compatible_contains("jetson-orin-nx"))   return JETSON_MODEL_ORIN_NX;
    if (dt_compatible_contains("jetson-orin-nano")) return JETSON_MODEL_ORIN_NANO;
    if (dt_compatible_contains("jetson-xavier-nx")) return JETSON_MODEL_XAVIER_NX;
    if (dt_compatible_contains("jetson-nano"))      return JETSON_MODEL_NANO;

    return JETSON_MODEL_UNKNOWN;
}

typedef enum {
    RPI_MODEL_UNKNOWN = 0,
    RPI_MODEL_PI4     = 1,
    RPI_MODEL_PI5     = 2,
    RPI_MODEL_CM4     = 3
} rpi_model_t;

static rpi_model_t parse_rpi_model(const char *model_str) {
    if (model_str && contains_ci(model_str, "Raspberry Pi 5"))  return RPI_MODEL_PI5;
    if (model_str && contains_ci(model_str, "Raspberry Pi 4"))  return RPI_MODEL_PI4;
    if (model_str && contains_ci(model_str, "Compute Module 4")) return RPI_MODEL_CM4;
    return RPI_MODEL_UNKNOWN;
}

typedef struct {
    platform_family_t family;
    uint16_t model;
} platform_t;

/**
 *  Detect platform family + model on Linux.
 * @param out: output platform struct
 * @param human: optional output buffer for a human-readable model string (may be NULL)
 * @param humansz: size of @human
 *
 * Detection order:
 *   1) Device Tree (/proc/device-tree/...) for embedded boards.
 *
 * @return: 0 on success (even if model is UNKNOWN), -1 on hard failure.
 */
static int platform_detect(platform_t *out, char *human, size_t humansz) {
    char model[256] = {0};

    if (!out) {
        errno = EINVAL;
        return -1;
    }

    out->family = PLATFORM_UNKNOWN;
    out->model  = 0;
    if (human && humansz) human[0] = '\0';

    if (read_dt_string("/proc/device-tree/model", model, sizeof(model)) == 0) {
        if (contains_ci(model, "NVIDIA Jetson") || dt_compatible_contains("nvidia,jetson")) {
            out->family = PLATFORM_JETSON;
            out->model  = (uint16_t)parse_jetson_model(model);
            if (human && humansz) {
                strncpy(human, model, humansz - 1);
                human[humansz - 1] = '\0';
            }
            return 0;
        }

        if (contains_ci(model, "Raspberry Pi")) {
            out->family = PLATFORM_RASPBERRY;
            out->model  = (uint16_t)parse_rpi_model(model);
            if (human && humansz) {
                strncpy(human, model, humansz - 1);
                human[humansz - 1] = '\0';
            }
            return 0;
        }

        out->family = PLATFORM_UNKNOWN;
        out->model  = 0;
        if (human && humansz) {
            strncpy(human, model, humansz - 1);
            human[humansz - 1] = '\0';
        }
        return 0;
    }

    return -1;
}

/* ========================================================================================
 * PERIPHERALS - Definitions, error codes and helpers
 * ======================================================================================== */

typedef enum {
    PERIPH_NONE      = 0,
    PERIPH_BATTERY   = 1,
    PERIPH_MOTOR     = 2,
    PERIPH_ENCODER   = 3,
    PERIPH_TOF       = 4,
    PERIPH_DISPLAY   = 5,
    PERIPH_GPIO      = 6,
    PERIPH_IMU       = 7,
    PERIPH_LED       = 8,
} peripheral_type_t;

static const char *type_to_string(peripheral_type_t t) {
    switch (t) {
        case PERIPH_BATTERY: return "battery";
        case PERIPH_MOTOR:   return "motor";
        case PERIPH_ENCODER: return "encoder";
        case PERIPH_TOF:     return "tof";
        case PERIPH_DISPLAY: return "display";
        case PERIPH_GPIO:    return "gpio";
        case PERIPH_IMU:     return "imu";
        case PERIPH_LED:     return "led";
        default:             return "none";
    }
}

typedef enum {
    IFACE_NONE = 0,
    IFACE_I2C  = 1,
    IFACE_SPI  = 2,
    IFACE_UART = 3,
    IFACE_GPIO = 4,
    IFACE_PWM  = 5,
    IFACE_USB  = 6,
    IFACE_V4L2 = 7,
    IFACE_CSI  = 8,
} peripheral_iface_t;

static const char *iface_to_string(peripheral_iface_t i) {
    switch (i) {
        case IFACE_I2C:  return "i2c";
        case IFACE_SPI:  return "spi";
        case IFACE_UART: return "uart";
        case IFACE_GPIO: return "gpio";
        case IFACE_PWM:  return "pwm";
        case IFACE_USB:  return "usb";
        case IFACE_V4L2: return "v4l2";
        case IFACE_CSI:  return "csi";
        default:         return "none";
    }
}

typedef enum {
    PERIPH_FLAG_NONE     = 0,
    PERIPH_FLAG_OPTIONAL = 1u << 0,
    PERIPH_FLAG_HOTPLUG  = 1u << 1,
    PERIPH_FLAG_READONLY = 1u << 2,
} peripheral_flags_t;

typedef struct {
    const char *key;
    const char *value;
} peripheral_kv_t;

typedef enum {
    ENDPOINT_ROLE_NONE = 0,
    ENDPOINT_ROLE_IRQ,
    ENDPOINT_ROLE_RESET,
    ENDPOINT_ROLE_ENABLE,
    ENDPOINT_ROLE_AUX0,
    ENDPOINT_ROLE_AUX1,
} endpoint_role_t;

static const char *role_to_string(endpoint_role_t r) {
    switch (r) {
        case ENDPOINT_ROLE_IRQ:    return "irq";
        case ENDPOINT_ROLE_RESET:  return "reset";
        case ENDPOINT_ROLE_ENABLE: return "enable";
        case ENDPOINT_ROLE_AUX0:   return "aux0";
        case ENDPOINT_ROLE_AUX1:   return "aux1";
        default:                   return "none";
    }
}

typedef struct {
    const char *chip;
    uint32_t offset;
    bool active_low;
} gpio_desc_t;

typedef union {
    struct { const char *adapter; uint16_t addr;                            } i2c;  // e.g. adapter="/dev/i2c-1", addr=0x3C
    struct { const char *dev;     uint32_t hz;        uint8_t mode;         } spi;  // e.g. dev="/dev/spidev0.0"
    struct { const char *dev;     uint32_t baud;                            } uart; // e.g. dev="/dev/ttyAMA0"
    struct { gpio_desc_t line;                                              } gpio; // gpiochipN + offset (line)
    struct { const char *chip;    uint32_t channel;   uint32_t period_ns;   } pwm;
    struct { uint16_t vid, pid;   const char *serial; uint8_t interface_no; } usb;
    struct { const char *dev;                                               } v4l2;
    struct { uint8_t port;        uint8_t lanes;                            } csi;
} endpoint_u_t;

static void dump_endpoint_u(FILE *fp, peripheral_iface_t iface, const endpoint_u_t *u) {
    switch (iface) {
        case IFACE_I2C:
            fprintf(fp, "adapter=%s addr=0x%02x",
                    u->i2c.adapter ? u->i2c.adapter : "(null)",
                    (unsigned)(u->i2c.addr & 0x7Fu));
            break;
        case IFACE_SPI:
            fprintf(fp, "dev=%s hz=%u mode=%u",
                    u->spi.dev ? u->spi.dev : "(null)",
                    (unsigned)u->spi.hz,
                    (unsigned)u->spi.mode);
            break;
        case IFACE_UART:
            fprintf(fp, "dev=%s baud=%u",
                    u->uart.dev ? u->uart.dev : "(null)",
                    (unsigned)u->uart.baud);
            break;
        case IFACE_GPIO:
            fprintf(fp, "chip=%s offset=%u active_low=%d",
                    u->gpio.line.chip ? u->gpio.line.chip : "(null)",
                    (unsigned)u->gpio.line.offset,
                    u->gpio.line.active_low ? 1 : 0);
            break;
        case IFACE_PWM:
            fprintf(fp, "chip=%s channel=%u period_ns=%u",
                    u->pwm.chip ? u->pwm.chip : "(null)",
                    (unsigned)u->pwm.channel,
                    (unsigned)u->pwm.period_ns);
            break;
        case IFACE_V4L2:
            fprintf(fp, "dev=%s", u->v4l2.dev ? u->v4l2.dev : "(null)");
            break;
        case IFACE_CSI:
            fprintf(fp, "port=%u lanes=%u",
                    (unsigned)u->csi.port,
                    (unsigned)u->csi.lanes);
            break;
        case IFACE_USB:
            fprintf(fp, "vid=0x%04x pid=0x%04x serial=%s if=%u",
                    (unsigned)u->usb.vid,
                    (unsigned)u->usb.pid,
                    u->usb.serial ? u->usb.serial : "(null)",
                    (unsigned)u->usb.interface_no);
            break;
        default:
            fprintf(fp, "(none)");
            break;
    }
}

typedef struct {
    peripheral_iface_t iface;
    endpoint_u_t u;
} peripheral_primary_t;

#ifdef __cplusplus

#define PRI_I2C(_adapter, _addr) \
    peripheral_primary_t{ .iface=IFACE_I2C, .u={.i2c = {.adapter=(_adapter), .addr=(_addr) } } }

#define PRI_GPIO(_chip, _offset, _active_low) \
    peripheral_primary_t{ .iface=IFACE_GPIO, .u={.gpio = {.line={.chip = (_chip), .offset = (_offset), .active_low = (_active_low) } } } }

#define PRI_SPI(_dev, _hz, _mode) \
    peripheral_primary_t{ \
        .iface = IFACE_SPI, \
        .u = { .spi = { .dev = (_dev), .hz = (_hz), .mode = (_mode) } } \
    }

#define PRI_UART(_dev, _baud) \
    peripheral_primary_t{ \
        .iface = IFACE_UART, \
        .u = { .uart = { .dev = (_dev), .baud = (_baud) } } \
    }

#define PRI_V4L2(_dev) \
    peripheral_primary_t{ \
        .iface = IFACE_V4L2, \
        .u = { .v4l2 = { .dev = (_dev) } } \
    }

#define PRI_CSI(_port, _lanes) \
    peripheral_primary_t{ \
        .iface = IFACE_CSI, \
        .u = { .csi = { .port = (_port), .lanes = (_lanes) } } \
    }


#else

#define PRI_I2C(_adapter, _addr) \
    (peripheral_primary_t){ .iface=IFACE_I2C, .u.i2c={ .adapter=(_adapter), .addr=(_addr) } }

#define PRI_GPIO(_chip, _offset, _active_low) \
    (peripheral_primary_t){ .iface=IFACE_GPIO, .u.gpio={ .line={ .chip=(_chip), .offset=(_offset), .active_low=(_active_low) } } }

#define PRI_SPI(_dev, _hz, _mode) \
    (peripheral_primary_t){ .iface=IFACE_SPI, .u.spi={ .dev=(_dev), .hz=(_hz), .mode=(_mode) } }

#define PRI_UART(_dev, _baud) \
    (peripheral_primary_t){ .iface=IFACE_UART, .u.uart={ .dev=(_dev), .baud=(_baud) } }

#define PRI_V4L2(_dev) \
    (peripheral_primary_t){ .iface=IFACE_V4L2, .u.v4l2={ .dev=(_dev) } }

#define PRI_CSI(_port, _lanes) \
    (peripheral_primary_t){ .iface=IFACE_CSI, .u.csi={ .port=(_port), .lanes=(_lanes) } }

#endif


#define PERIPH_MAX_AUX 3

typedef struct {
    endpoint_role_t role;
    peripheral_iface_t iface;
    endpoint_u_t u;
} peripheral_aux_t;

struct peripheral_desc {
    peripheral_type_t type;
    const char *name;
    const char *driver;
    uint32_t flags;

    peripheral_primary_t primary;

    uint8_t num_aux;
    peripheral_aux_t aux[PERIPH_MAX_AUX];

    const peripheral_kv_t *props;
    uint16_t num_props;
};

static const peripheral_aux_t *peripheral_get_aux(const peripheral_desc_t *d, endpoint_role_t role) {
    uint8_t i;
    if (!d || role == ENDPOINT_ROLE_NONE) return NULL;
    for (i = 0; i < d->num_aux; i++) {
        if (d->aux[i].role == role) return &d->aux[i];
    }
    return NULL;
}

static const char *peripheral_prop_get(const peripheral_desc_t *d, const char *key) {
    uint16_t i;
    if (!d || !d->props || !key) return NULL;
    for (i = 0; i < d->num_props; i++) {
        if (!d->props[i].key) continue;
        if (strcmp(d->props[i].key, key) == 0) return d->props[i].value;
    }
    return NULL;
}

static int peripheral_prop_get_u32(const peripheral_desc_t *d, const char *key, uint32_t *out) {
    const char *v = peripheral_prop_get(d, key);
    char *end = NULL;
    unsigned long x;
    if (!v || !out) return -1;
    x = strtoul(v, &end, 0);
    if (end == v) return -1;
    *out = (uint32_t)x;
    return 0;
}

static int peripheral_prop_get_double(const peripheral_desc_t *d, const char *key, double *out) {
    const char *v = peripheral_prop_get(d, key);
    char *end = NULL;
    double x;
    if (!v || !out) return -1;
    x = strtod(v, &end);
    if (end == v) return -1;
    *out = x;
    return 0;
}

typedef enum {
    PERIPH_OK = 0,
    // there is not point of having a primary peripheral
    // without an iface to interact with it.
    PERIPH_ERROR_PRIMARY_NONE,

    // this is to prevent blowup the aux endpoint
    // of a peripheral with unnecessary aux(s).
    PERIPH_ERROR_AUX_COUNT,

    // aux endpoint on peripheral are not allowed to have role none, 
    // this is "semantically" **important** for the "consumer driver"
    // so driver can decide to use the aux endpoint in the peripheral
    // given the role attach to it.
    PERIPH_ERROR_AUX_ROLE_NONE, 

    // duplicated role not allowed
    // no need to have two aux endpoint for a peripheral
    // that serve the same role.
    PERIPH_ERROR_AUX_ROLE_DUP,  

    // in the same way as PERIPH_ERROR_PRIMARY_NONE
    // there is not point of having a aux endpoint for a peripheral
    // if it does not have iface to interact with it.
    PERIPH_ERROR_AUX_IFACE_NONE, 
} peripheral_error_t;

static const char *peripheral_error_to_string(peripheral_error_t e) {
    switch (e) {
        case PERIPH_OK:                   return "peripheral is ok";
        case PERIPH_ERROR_PRIMARY_NONE:   return "primary peripheral can't be of iface none";
        case PERIPH_ERROR_AUX_COUNT:      return "exceeding max number of aux peripheral endpoints";
        case PERIPH_ERROR_AUX_ROLE_NONE:  return "aux endpoints can't be of role none";
        case PERIPH_ERROR_AUX_ROLE_DUP:   return "aux endpoints can't have duplicate roles";
        case PERIPH_ERROR_AUX_IFACE_NONE: return "aux endpoint requires an iface";
        default:                          return "unknown peripheral error";
    }
}

static peripheral_error_t peripheral_validate_basic(const peripheral_desc_t *d) {
    uint32_t seen = 0;
    uint8_t i;

    if (!d) return PERIPH_ERROR_PRIMARY_NONE;
    if (d->primary.iface == IFACE_NONE) return PERIPH_ERROR_PRIMARY_NONE;
    if (d->num_aux > PERIPH_MAX_AUX) return PERIPH_ERROR_AUX_COUNT;

    for (i = 0; i < d->num_aux; i++) {
        const peripheral_aux_t *a = &d->aux[i];
        uint32_t bit;

        if (a->role == ENDPOINT_ROLE_NONE) return PERIPH_ERROR_AUX_ROLE_NONE;
        if (a->iface == IFACE_NONE) return PERIPH_ERROR_AUX_IFACE_NONE;

        bit = 1u << (uint32_t)a->role;
        if (seen & bit) return PERIPH_ERROR_AUX_ROLE_DUP;
        seen |= bit;
    }
    return PERIPH_OK;
}

/* ========================================================================================
 * ROBOT (HARDWARE) DEFINITION
 * ======================================================================================== */

/**
 * Pack platform family + model + hat into a single key.
 *
 * Layout: [ family:8 | model:16 | hat:8 ]
 *
 * return: packed 32-bit key.
 */
#define ROBOT_DEF_KEY(family, model, hat) \
    ((((uint32_t)((family) & 0xFFu)) << 24) | \
     (((uint32_t)((model)  & 0xFFFFu)) << 8) | \
     (((uint32_t)((hat)    & 0xFFu))))

static inline uint32_t robot_def_key(platform_family_t platform, uint16_t model, hat_t hat) {
    return ROBOT_DEF_KEY(platform, model, hat);
}

static const peripheral_kv_t leds_props[] = {
    { "pwm_hz", "1000" },
    { "gamma", "2.2" },
};

static const peripheral_kv_t imu_props[] = {
    { "accel_range", "1" },
    { "gyro_range", "1" },
    { "dlpf_cfg", "3" },
    { "sample_rate_div", "9" },
};

static const peripheral_kv_t motor1_props[] = {
    // the semantic in this case is:
    // <chip>_channel_<motor func>
    { "pca9685_ch_pwm", "8" },
    { "pca9685_ch_in1", "10" },
    { "pca9685_ch_in2", "9" },
    { "invert", "0" },
};

static const peripheral_kv_t motor2_props[] = {
    // the semantic in this case is:
    // <chip>_channel_<motor func>
    { "pca9685_ch_pwm", "13" },
    { "invert", "0" },
    // the IN1 and IN2 for this motors
    // are better define as aux endpoints
};

static const peripheral_kv_t motor_enc_props[] = {
    { "ticks_per_rev", "360" },
};

static const peripheral_desc_t jetson_nano_hat_v3_15[] = {
    {
        .type = PERIPH_MOTOR,
        .name = "motor1",
        .driver = "motor_hbridge",
        .flags = PERIPH_FLAG_NONE,
        .primary = PRI_I2C("/dev/i2c-1", 0x60),
        .num_aux = 0,
        .props = motor1_props,
        .num_props = (uint16_t)(sizeof(motor1_props)/sizeof(motor1_props[0])),
    },
    {
        .type = PERIPH_GPIO,
        .name = "motor1_enc_a",
        .driver = "no_driver",
        .flags = PERIPH_FLAG_NONE,
        .primary = PRI_GPIO("/dev/gpiochip0", 79, false), // physical pin 12
        .num_aux = 0,
        .props = motor_enc_props,
        .num_props = (uint16_t)(sizeof(motor_enc_props)/sizeof(motor_enc_props[0])),
    },
    {
        .type = PERIPH_GPIO,
        .name = "motor1_enc_b",
        .driver = "no_driver",
        .flags = PERIPH_FLAG_NONE,
        .primary = PRI_GPIO("/dev/gpiochip0", 232, false), // physical pin 16
        .num_aux = 0,
        .props = motor_enc_props,
        .num_props = (uint16_t)(sizeof(motor_enc_props)/sizeof(motor_enc_props[0])),
    },
    {
        .type = PERIPH_MOTOR,
        .name = "motor2",
        .driver = "motor_hbridge",
        .flags = PERIPH_FLAG_NONE,
        .primary = PRI_I2C("/dev/i2c-1", 0x60),
        .num_aux = 2,
        .aux = {
            { .role=ENDPOINT_ROLE_AUX0, .iface=IFACE_GPIO,
              .u={.gpio = { .line = { .chip="/dev/gpiochip0", .offset=38, .active_low=false } } } }, // physical pin 33
            { .role=ENDPOINT_ROLE_AUX1, .iface=IFACE_GPIO,
              .u = { .gpio = { .line = { .chip="/dev/gpiochip0", .offset=200, .active_low=false } } } }, // physical pin 31
        },
        .props = motor2_props,
        .num_props = (uint16_t)(sizeof(motor2_props)/sizeof(motor2_props[0])),
    },
    {
        .type = PERIPH_GPIO,
        .name = "motor2_enc_a",
        .driver = "no_driver",
        .flags = PERIPH_FLAG_NONE,
        .primary = PRI_GPIO("/dev/gpiochip0", 76, false), // physical pin 35
        .num_aux = 0,
        .props = motor_enc_props,
        .num_props = (uint16_t)(sizeof(motor_enc_props)/sizeof(motor_enc_props[0])),
    },
    {
        .type = PERIPH_GPIO,
        .name = "motor2_enc_b",
        .driver = "no_driver",
        .flags = PERIPH_FLAG_NONE,
        .primary = PRI_GPIO("/dev/gpiochip0", 51, false), // physical pin 36
        .num_aux = 0,
        .props = motor_enc_props,
        .num_props = (uint16_t)(sizeof(motor_enc_props)/sizeof(motor_enc_props[0])),
    },
    {
        .type = PERIPH_DISPLAY,
        .name = "display0",
        .driver = "ssd1306",
        .flags = PERIPH_FLAG_NONE,
        .primary = PRI_I2C("/dev/i2c-1", 0x3C),
        .num_aux = 0,
    },
    {
        .type = PERIPH_LED,
        .name = "leds_front_and_rear",
        .driver = "pca9685",
        .flags = PERIPH_FLAG_NONE,
        .primary = PRI_I2C("/dev/i2c-1", 0x40),
        .num_aux = 0,
        .props = leds_props,
        .num_props = (uint16_t)(sizeof(leds_props)/sizeof(leds_props[0])),
    },
    {
        .type = PERIPH_IMU,
        .name = "imu0",
        .driver = "mpu6050",
        .flags = PERIPH_FLAG_NONE,
        .primary = PRI_I2C("/dev/i2c-1", 0x68),
        .num_aux = 0,
        .props = imu_props,
        .num_props = (uint16_t)(sizeof(imu_props)/sizeof(imu_props[0])),
    },
    {
        .type = PERIPH_GPIO,
        .name = "top_button",
        .driver = "gpio",
        .flags = PERIPH_FLAG_NONE,
        .primary = PRI_GPIO("/dev/gpiochip0", 78, false),
        .num_aux = 0,
    },
    {
        .type = PERIPH_GPIO,
        .name = "top_button_led",
        .driver = "gpio",
        .flags = PERIPH_FLAG_NONE,
        .primary = PRI_GPIO("/dev/gpiochip0", 12, false),
        .num_aux = 0,
    },
    {
        .type = PERIPH_GPIO,
        .name = "hat_builtin_led",
        .driver = "gpio",
        .flags = PERIPH_FLAG_NONE,
        .primary = PRI_GPIO("/dev/gpiochip0", 77, false),
        .num_aux = 0,
    },
};

#define ROBOT_ENTRY(FAMILY, MODEL, HAT, ID, NAME, PERIPHS) \
{ \
    .key = ROBOT_DEF_KEY((FAMILY), (uint16_t)(MODEL), (HAT)), \
    .id = (ID), \
    .display_name = (NAME), \
    .platform_family = (FAMILY), \
    .platform_model = (uint16_t)(MODEL), \
    .hat = (HAT), \
    .peripherals = (PERIPHS), \
    .num_peripherals = sizeof(PERIPHS)/sizeof((PERIPHS)[0]), \
}

static const robot_def_t robot_table[] = {
    ROBOT_ENTRY(
        PLATFORM_JETSON,
        JETSON_MODEL_NANO,
        HAT_V3_15,
        "jetson-nano_hat_v3_15",
        "Jetson Nano (HAT v3.15)",
        jetson_nano_hat_v3_15
    )
};

static int robot_def_get(platform_t platform, hat_t hat, const robot_def_t **out) {
    uint32_t key;
    size_t i;

    if (!out) return EINVAL;
    *out = NULL;

    if (platform.family == PLATFORM_UNKNOWN || platform.model == 0 || hat == HAT_UNKNOWN)
        return EINVAL;

    key = robot_def_key(platform.family, platform.model, hat);

    for (i = 0; i < sizeof(robot_table)/sizeof(robot_table[0]); i++) {
        if (robot_table[i].key == key) {
            *out = &robot_table[i];
            return 0;
        }
    }

    return EINVAL;
}

FOXY_API void robot_def_dump(const robot_def_t *def, void *out) {
    FILE *fp = (FILE *)out;
    size_t i;

    if (!fp) fp = stdout;
    if (!def) {
        fprintf(fp, "Robot: (null)\n");
        return;
    }

    fprintf(fp, "Robot:\n");
    fprintf(fp, "  id: %s\n", def->id ? def->id : "(null)");
    fprintf(fp, "  name: %s\n", def->display_name ? def->display_name : "(null)");
    fprintf(fp, "  key: 0x%08x\n", (unsigned)def->key);
    fprintf(fp, "  platform: %s\n", family_to_string(def->platform_family));
    fprintf(fp, "  peripherals: %zu\n", def->num_peripherals);

    for (i = 0; i < def->num_peripherals; i++) {
        const peripheral_desc_t *p = &def->peripherals[i];
        peripheral_error_t val_error = peripheral_validate_basic(p);
        uint8_t a;
        uint16_t k;

        fprintf(fp,
                "    - [%zu] type=%s name=%s driver=%s flags=0x%x",
                i,
                type_to_string(p->type),
                p->name ? p->name : "(null)",
                p->driver ? p->driver : "(null)",
                (unsigned)p->flags);

        if (val_error != PERIPH_OK) {
            fprintf(fp, " (error: %d) %s\n", (int)val_error, peripheral_error_to_string(val_error));
            continue;
        }
        fprintf(fp, "\n");

        fprintf(fp, "\t primary: iface=%s ", iface_to_string(p->primary.iface));
        dump_endpoint_u(fp, p->primary.iface, &p->primary.u);
        fprintf(fp, "\n");

        if (p->num_aux) {
            fprintf(fp, "\t aux:\n");
            for (a = 0; a < p->num_aux; a++) {
                const peripheral_aux_t *aux = &p->aux[a];
                fprintf(fp, "\t - role=%s iface=%s ",
                        role_to_string(aux->role),
                        iface_to_string(aux->iface));
                dump_endpoint_u(fp, aux->iface, &aux->u);
                fprintf(fp, "\n");
            }
        }

        if (p->props && p->num_props) {
            fprintf(fp, "\t props:\n");
            for (k = 0; k < p->num_props; k++) {
                fprintf(fp, "\t - %s=%s\n",
                        p->props[k].key ? p->props[k].key : "(null)",
                        p->props[k].value ? p->props[k].value : "(null)");
            }
        }
    }
}

/* ========================================================================================
 * HELPERS IOCTL functions
 * ======================================================================================== */

static int i2c_rdwr(int fd, struct i2c_msg *msgs, int nmsgs) {
    struct i2c_rdwr_ioctl_data data;
    data.msgs  = msgs;
    data.nmsgs = (uint32_t)nmsgs;

    if (ioctl(fd, I2C_RDWR, &data) < 0) return -1;
    return 0;
}

static int i2c_write_reg_bytes(int fd, uint8_t addr7, uint8_t reg, const uint8_t *buf, size_t len) {
    uint8_t tmp[1 + 32];
    struct i2c_msg msg;

    if (len > 32) {
        errno = EINVAL;
        return -1;
    }

    tmp[0] = reg;
    if (len) memcpy(&tmp[1], buf, len);

    msg.addr  = addr7;
    msg.flags = 0;
    msg.len   = (uint16_t)(1 + len);
    msg.buf   = tmp;

    return i2c_rdwr(fd, &msg, 1);
}

static int i2c_write_reg_u8(int fd, uint8_t addr7, uint8_t reg, uint8_t value) {
    return i2c_write_reg_bytes(fd, addr7, reg, &value, 1);
}

static int i2c_read_reg_bytes(int fd, uint8_t addr7, uint8_t reg, uint8_t *out, size_t len) {
    struct i2c_msg msgs[2];
    uint8_t r = reg;

    msgs[0].addr  = addr7;
    msgs[0].flags = 0;
    msgs[0].len   = 1;
    msgs[0].buf   = &r;

    msgs[1].addr  = addr7;
    msgs[1].flags = I2C_M_RD;
    msgs[1].len   = (uint16_t)len;
    msgs[1].buf   = out;

    return i2c_rdwr(fd, msgs, 2);
}

static int i2c_read_reg_u8(int fd, uint8_t addr7, uint8_t reg, uint8_t *out) {
    struct i2c_msg msgs[2];
    uint8_t r = reg;

    msgs[0].addr  = addr7;
    msgs[0].flags = 0;
    msgs[0].len   = 1;
    msgs[0].buf   = &r;

    msgs[1].addr  = addr7;
    msgs[1].flags = I2C_M_RD;
    msgs[1].len   = 1;
    msgs[1].buf   = out;

    return i2c_rdwr(fd, msgs, 2);
}

static int open_i2c_fd(const char *file_descriptor) {
    int fd;
    unsigned long funcs = 0;

    if (!file_descriptor) return -EINVAL;

    fd = open(file_descriptor, O_RDWR | O_CLOEXEC);
    if (fd < 0) return -errno;

    if (ioctl(fd, I2C_FUNCS, &funcs) < 0) {
        int e = -errno;
        close(fd);
        return e;
    }

    if (!(funcs & I2C_FUNC_I2C)) {
        close(fd);
        return -ENOTSUP;
    }

    return fd;
}

/* ========================================================================================
 * TIMING UTILITY FUNCTIONS
 * ======================================================================================== */

static uint64_t mono_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ((uint64_t)ts.tv_sec * 1000ull) + ((uint64_t)ts.tv_nsec / 1000000ull);
}

FOXY_API void sleep_ms(unsigned ms) {
    struct timespec ts;
    ts.tv_sec = (time_t)(ms / 1000);
    ts.tv_nsec = (long)((ms % 1000) * 1000000UL);
    nanosleep(&ts, NULL);
}

/* ========================================================================================
 * PERIPHERALS: Low-level functions
 * ======================================================================================== */

/* ++++++++++++++++++++++++++ PCA9685 ++++++++++++++++++++++++++ */

#define PCA9685_MODE1       0x00
#define PCA9685_MODE2       0x01
#define PCA9685_PRESCALE    0xFE

#define PCA9685_LED0_ON_L     0x06
#define PCA9685_ALL_LED_ON_L  0xFA

#define MODE1_RESTART (1u << 7)
#define MODE1_AI      (1u << 5)
#define MODE1_SLEEP   (1u << 4)

#define MODE2_OUTDRV  (1u << 2)
#define LED_FULL_ON_OFF_BIT (1u << 4)
#define PCA9685_OSC_HZ 25000000.0
#define PCA9685_TICKS_COUNT 4096
#define PCA9685_TICKS_MAX (PCA9685_TICKS_COUNT - 1)

static uint16_t gamma_lut[256];

static void gamma_init(double gamma) {
    int i;
    uint16_t res_12bit = PCA9685_TICKS_MAX;
    for (i = 0; i < 256; i++) {
        double x = (double)i / 255.0;
        double y = pow(x, gamma);
        long v12 = lround(y * (double)res_12bit);
        if (v12 < 0) v12 = 0;
        if (v12 > res_12bit) v12 = res_12bit;
        gamma_lut[i] = (uint16_t)v12;
    }
}

static uint8_t prescale_for_hz(double pwm_hz) {
    double prescale_f = (PCA9685_OSC_HZ / ((double)PCA9685_TICKS_COUNT * pwm_hz)) - 1.0;
    long prescale = lround(prescale_f);

    if (prescale < 3) prescale = 3;
    if (prescale > 255) prescale = 255;
    return (uint8_t)prescale;
}

static int pca9685_init(int fd, uint8_t addr7, double pwm_hz) {
    uint8_t mode1 = 0;
    uint8_t mode1_sleep;
    uint8_t prescale;
    uint8_t mode1_wake;
    uint8_t all_off[4] = {0x00, 0x00, 0x00, LED_FULL_ON_OFF_BIT };

    if (i2c_write_reg_u8(fd, addr7, PCA9685_MODE2, MODE2_OUTDRV) < 0) return -1;
    if (i2c_read_reg_u8(fd, addr7, PCA9685_MODE1, &mode1) < 0) return -1;

    mode1_sleep = (uint8_t)((mode1 & ~MODE1_RESTART) | MODE1_SLEEP | MODE1_AI);
    if (i2c_write_reg_u8(fd, addr7, PCA9685_MODE1, mode1_sleep) < 0) return -1;

    prescale = prescale_for_hz(pwm_hz);
    if (i2c_write_reg_u8(fd, addr7, PCA9685_PRESCALE, prescale) < 0) return -1;

    mode1_wake = (uint8_t)((mode1 & ~MODE1_SLEEP) | MODE1_AI);
    if (i2c_write_reg_u8(fd, addr7, PCA9685_MODE1, mode1_wake) < 0) return -1;

    sleep_ms(2);

    if (i2c_write_reg_u8(fd, addr7, PCA9685_MODE1, (uint8_t)(mode1_wake | MODE1_RESTART)) < 0) return -1;
    if (i2c_write_reg_bytes(fd, addr7, PCA9685_ALL_LED_ON_L, all_off, sizeof(all_off)) < 0) return -1;

    return 0;
}

static int pca9685_set_pwm(int fd, uint8_t addr7, uint8_t channel, uint16_t on, uint16_t off) {
    uint8_t reg;
    uint8_t buf[4];

    if (channel > 15 || on > PCA9685_TICKS_MAX || off > PCA9685_TICKS_MAX) {
        errno = EINVAL;
        return -1;
    }

    reg = (uint8_t)(PCA9685_LED0_ON_L + 4 * channel);

    if (off == 0) {
        buf[0] = 0x00; buf[1] = 0x00; buf[2] = 0x00; buf[3] = LED_FULL_ON_OFF_BIT;
    } else if (off >= PCA9685_TICKS_MAX) {
        buf[0] = 0x00; buf[1] = LED_FULL_ON_OFF_BIT; buf[2] = 0x00; buf[3] = 0x00;
    } else {
        buf[0] = (uint8_t)(on & 0xFF);
        buf[1] = (uint8_t)((on >> 8) & 0x0F);
        buf[2] = (uint8_t)(off & 0xFF);
        buf[3] = (uint8_t)((off >> 8) & 0x0F);
    }

    return i2c_write_reg_bytes(fd, addr7, reg, buf, sizeof(buf));
}

static uint16_t rgb8_to_12(uint8_t v8) {
    return gamma_lut[v8];
}

static int pca9685_set_rgb(int fd, uint8_t addr7, uint8_t led_index, uint8_t r8, uint8_t g8, uint8_t b8) {
    uint8_t base = (uint8_t)(led_index * 3);
    uint16_t r12, g12, b12;

    if (base + 2 > 15) {
        errno = EINVAL;
        return -1;
    }

    r12 = rgb8_to_12(r8);
    g12 = rgb8_to_12(g8);
    b12 = rgb8_to_12(b8);

    if (pca9685_set_pwm(fd, addr7, (uint8_t)(base + 0), 0, r12) < 0) return -1;
    if (pca9685_set_pwm(fd, addr7, (uint8_t)(base + 1), 0, g12) < 0) return -1;
    if (pca9685_set_pwm(fd, addr7, (uint8_t)(base + 2), 0, b12) < 0) return -1;

    return 0;
}

/* ++++++++++++++++++++++++++ MPU6050 ++++++++++++++++++++++++++ */

#define MPU6050_WHO_AM_I         0x75
#define MPU6050_PWR_MGMT_1       0x6B
#define MPU6050_SAMPLE_RATE_DIV  0x19
#define MPU6050_CONFIG           0x1A
#define MPU6050_GYRO_CONFIG      0x1B
#define MPU6050_ACCEL_CONFIG     0x1C
#define MPU6050_ACCEL_XOUT_H     0x3B

#define PWR1_SLEEP            (1u << 6)
#define PWR1_CLKSEL_PLL_XGYRO 0x01

typedef enum {
    MPU6050_ACCEL_2G  = 0,
    MPU6050_ACCEL_4G  = 1,
    MPU6050_ACCEL_8G  = 2,
    MPU6050_ACCEL_16G = 3
} mpu6050_accel_range_t;

typedef enum {
    MPU6050_GYRO_250DPS  = 0,
    MPU6050_GYRO_500DPS  = 1,
    MPU6050_GYRO_1000DPS = 2,
    MPU6050_GYRO_2000DPS = 3
} mpu6050_gyro_range_t;

typedef struct {
    uint8_t addr7;
    mpu6050_accel_range_t accel_range;
    mpu6050_gyro_range_t gyro_range;
    uint8_t dlpf_cfg;
    uint8_t sample_rate_div;

    float accel_lsb_per_g;
    float gyro_lsb_per_dps;
} mpu6050_t;

typedef struct {
    int16_t ax, ay, az;
    int16_t temp;
    int16_t gx, gy, gz;
} mpu6050_raw_t;

typedef struct {
    float ax_g, ay_g, az_g;
    float ax_ms2, ay_ms2, az_ms2;
    float gx_dps, gy_dps, gz_dps;
    float temp_c;
} mpu6050_si_t;

static float accel_lsb_per_g(mpu6050_accel_range_t r) {
    switch (r) {
        case MPU6050_ACCEL_2G:  return 16384.0f;
        case MPU6050_ACCEL_4G:  return 8192.0f;
        case MPU6050_ACCEL_8G:  return 4096.0f;
        case MPU6050_ACCEL_16G: return 2048.0f;
        default:                return 8192.0f;
    }
}

static float gyro_lsb_per_dps(mpu6050_gyro_range_t r) {
    switch (r) {
        case MPU6050_GYRO_250DPS:  return 131.0f;
        case MPU6050_GYRO_500DPS:  return 65.5f;
        case MPU6050_GYRO_1000DPS: return 32.8f;
        case MPU6050_GYRO_2000DPS: return 16.4f;
        default:                   return 131.0f;
    }
}

static int mpu6050_init(int fd, mpu6050_t *dev) {
    uint8_t who = 0;
    uint8_t gyro_cfg;
    uint8_t accel_cfg;

    if (!dev) { errno = EINVAL; return -1; }

    if (i2c_read_reg_u8(fd, dev->addr7, MPU6050_WHO_AM_I, &who) < 0) return -1;
    if ((who & 0x7E) != 0x68) return -1;

    if (i2c_write_reg_u8(fd, dev->addr7, MPU6050_PWR_MGMT_1, PWR1_CLKSEL_PLL_XGYRO) < 0) return -1;
    sleep_ms(10);

    if (dev->dlpf_cfg > 6) dev->dlpf_cfg = 3;
    if (i2c_write_reg_u8(fd, dev->addr7, MPU6050_CONFIG, (uint8_t)(dev->dlpf_cfg & 0x07)) < 0) return -1;
    if (i2c_write_reg_u8(fd, dev->addr7, MPU6050_SAMPLE_RATE_DIV, dev->sample_rate_div) < 0) return -1;

    gyro_cfg = (uint8_t)((dev->gyro_range & 0x03) << 3);
    if (i2c_write_reg_u8(fd, dev->addr7, MPU6050_GYRO_CONFIG, gyro_cfg) < 0) return -1;

    accel_cfg = (uint8_t)((dev->accel_range & 0x03) << 3);
    if (i2c_write_reg_u8(fd, dev->addr7, MPU6050_ACCEL_CONFIG, accel_cfg) < 0) return -1;

    dev->accel_lsb_per_g   = accel_lsb_per_g(dev->accel_range);
    dev->gyro_lsb_per_dps  = gyro_lsb_per_dps(dev->gyro_range);

    return 0;
}

static int mpu6050_read_raw(int fd, const mpu6050_t *dev, mpu6050_raw_t *out) {
    uint8_t buf[14];

    if (!dev || !out) { errno = EINVAL; return -1; }

    if (i2c_read_reg_bytes(fd, dev->addr7, MPU6050_ACCEL_XOUT_H, buf, sizeof(buf)) < 0) return -1;

    out->ax   = (int16_t)((buf[0]  << 8) | buf[1]);
    out->ay   = (int16_t)((buf[2]  << 8) | buf[3]);
    out->az   = (int16_t)((buf[4]  << 8) | buf[5]);
    out->temp = (int16_t)((buf[6]  << 8) | buf[7]);
    out->gx   = (int16_t)((buf[8]  << 8) | buf[9]);
    out->gy   = (int16_t)((buf[10] << 8) | buf[11]);
    out->gz   = (int16_t)((buf[12] << 8) | buf[13]);

    return 0;
}

static void mpu6050_convert_si(const mpu6050_t *dev, const mpu6050_raw_t *raw, mpu6050_si_t *si) {
    const float g = 9.80665f;

    si->ax_g = (float)raw->ax / dev->accel_lsb_per_g;
    si->ay_g = (float)raw->ay / dev->accel_lsb_per_g;
    si->az_g = (float)raw->az / dev->accel_lsb_per_g;

    si->ax_ms2 = si->ax_g * g;
    si->ay_ms2 = si->ay_g * g;
    si->az_ms2 = si->az_g * g;

    si->gx_dps = (float)raw->gx / dev->gyro_lsb_per_dps;
    si->gy_dps = (float)raw->gy / dev->gyro_lsb_per_dps;
    si->gz_dps = (float)raw->gz / dev->gyro_lsb_per_dps;

    si->temp_c = ((float)raw->temp / 340.0f) + 36.53f;
}

/* ++++++++++++++++++++++++++ VL53L0X (partial helpers) ++++++++++++++++++++++++++ */

typedef struct {
    int fd;
    uint8_t addr7;
    int io_timeout_ms;
    int did_timeout;
    uint8_t stop_variable;
    uint32_t measurement_timing_budget_us;
} vl53l0x_t;

static int poll_timeout(vl53l0x_t *dev, uint64_t start_ms) {
    if (dev->io_timeout_ms <= 0) return 0;
    if ((int)(mono_ms() - start_ms) >= dev->io_timeout_ms) {
        dev->did_timeout = 1;
        errno = ETIMEDOUT;
        return -1;
    }
    return 0;
}

static uint8_t decode_vcsel_period_pclks(uint8_t reg_val) {
    return (uint8_t)(((reg_val + 1u) & 0xFFu) << 1);
}

static uint16_t decode_timeout(uint16_t reg_val) {
    return (uint16_t)(((reg_val & 0x00FFu) << ((reg_val & 0xFF00u) >> 8)) + 1u);
}

static uint16_t encode_timeout(uint16_t timeout_mclks) {
    uint32_t ls;
    uint16_t ms = 0;
    if (timeout_mclks == 0) return 0;
    ls = (uint32_t)timeout_mclks - 1u;
    while (ls > 255u) { ls >>= 1; ms++; }
    return (uint16_t)((ms << 8) | (ls & 0xFFu));
}

static uint32_t calc_macro_period_ns(uint8_t vcsel_period_pclks) {
    return (uint32_t)((((uint32_t)2304u * (uint32_t)vcsel_period_pclks * 1655u) + 500u) / 1000u);
}

static uint32_t timeout_mclks_to_us(uint16_t timeout_mclks, uint8_t vcsel_period_pclks) {
    uint32_t macro_ns = calc_macro_period_ns(vcsel_period_pclks);
    return ((uint32_t)timeout_mclks * macro_ns + (macro_ns / 2u)) / 1000u;
}

static uint32_t timeout_us_to_mclks(uint32_t timeout_us, uint8_t vcsel_period_pclks) {
    uint32_t macro_ns = calc_macro_period_ns(vcsel_period_pclks);
    return ((timeout_us * 1000u) + (macro_ns / 2u)) / macro_ns;
}

static int vl53l0x_set_signal_rate_limit_mcps(int fd, float mcps) {
    (void)fd;
    if (mcps < 0.0f) mcps = 0.0f;
    if (mcps > 511.99f) mcps = 511.99f;
    return -ENOSYS;
}

/* ========================================================================================
 * RUNTIME: Robot API
 * ======================================================================================== */

/* IMPORTANT: this is a contract matching peripheral driver + type to implementation */

/**
 * IMPORTANT: this is like a contract
 * that match peripheral driver+type with
 * the actual driver implementation
 */
struct peripheral_driver {
    const char *name; // this name MUST match exactly the name defined in desc->driver.
                      // QUESTION: should this name be unique? of should we allow multiples
                      // peripherals to share the same driver implementation?
                      // it can be use useful for motor, leds, buttons
    peripheral_type_t type; // NOTE: not sure if really we need this for sanity check. leaving for now
    int (*bind)(const peripheral_desc_t *desc, void **out_ctx);
    void (*unbind)(void *ctx);
    const void *ops; // this point to the available <driver>_ops_t.
};

/**
 * C++-safe forward declarations for registry.
 * Use extern here; static incomplete const declarations are treated as definitions in C++.
 * These symbols exist only in the TU that defines FOXY_IMPLEMENTATION.
 */
extern const peripheral_driver_t global_drivers[];
extern const size_t global_num_drivers;

/**
 * driver lookup: exact-match only, deterministic
 *
 * TODO: how should we tread duplicates ?
 */
static const peripheral_driver_t *driver_find_exact(const char *name,
                                                    const peripheral_driver_t *tbl,
                                                    size_t n) {
    const peripheral_driver_t *m = NULL;
    size_t i;

    if (!name || !name[0] || !tbl) return NULL;

    for (i = 0; i < n; i++) {
        if (!tbl[i].name) continue;
        if (strcmp(tbl[i].name, name) != 0) continue;
        if (m) return NULL; // duplicate name, returning NULL for now util we decide for a final solution
        m = &tbl[i];
    }
    return m;
}

/**
 * Count the number of peripherals attach to the robot
 * for a given type
 */
static size_t robot_count_type(const robot_t *r, peripheral_type_t type) {
    size_t n = 0;
    size_t i;
    if (!r || !r->def) return 0;
    for (i = 0; i < r->def->num_peripherals; i++) {
        if (r->def->peripherals[i].type == type) n++;
    }
    return n;
}

static void robot_warn_ambiguous_type(const robot_t *r,
                                      peripheral_type_t type,
                                      const char *api_name,
                                      const char *matching_name) {
    size_t n = robot_count_type(r, type);
    if (n <= 1) return;
    fprintf(stderr,
            "warning: %s() found %zu peripherals of type '%s'; returning first match '%s'. "
            "Use %s_name(r, ...) to select unambiguously.\n",
            api_name, n, type_to_string(type), matching_name, api_name);
}

static int robot_find_index(const robot_t *r,
                            peripheral_type_t type,
                            const char *name,
                            const char *api_name,
                            uint16_t *out_idx) {
    size_t i;
    uint16_t found = UINT16_MAX;

    if (!r || !r->def || !out_idx) return -EINVAL;

    if (!name || !name[0]) {
        for (i = 0; i < r->def->num_peripherals; i++) {
            if (r->def->peripherals[i].type == type) {
                *out_idx = (uint16_t)i;
                robot_warn_ambiguous_type(r, type, api_name, r->def->peripherals[i].name);
                return 0;
            }
        }
        return -ENOENT;
    }

    for (i = 0; i < r->def->num_peripherals; i++) {
        const peripheral_desc_t *p = &r->def->peripherals[i];
        if (p->type != type) continue;
        if (!p->name) continue;
        if (strcmp(p->name, name) == 0) {
            found = (uint16_t)i;
            break;
        }
    }

    // returns -ENOENT when named peripheral is not found.
    if (found == UINT16_MAX) return -ENOENT;
    *out_idx = found;
    return 0;
}

static int robot_slot_acquire(robot_t *r, uint16_t idx) {
    robot_slot_t *s;
    const peripheral_desc_t *p;
    const peripheral_driver_t *driver;
    void *ctx = NULL;
    int brc;

    if (!r || !r->def) return -EINVAL;
    if (idx >= r->nslots) return -EINVAL;

    s = &r->slots[idx];
    p = &r->def->peripherals[idx];

    if (s->ctx) {
        s->refs++;
        return 0;
    }

    if (peripheral_validate_basic(p) != PERIPH_OK) return -EINVAL;
    if (!p->driver || !p->driver[0]) return -EINVAL;

    driver = driver_find_exact(p->driver, global_drivers, global_num_drivers);
    if (!driver || !driver->bind) return -ENOENT;
    if (driver->type != p->type) return -EINVAL;

    brc = driver->bind(p, &ctx);
    if (brc != 0) {
        if (p->flags & PERIPH_FLAG_OPTIONAL) {
            s->driver = driver;
            s->ctx = NULL;
            s->refs = 0;
        }
        return brc;
    }

    s->driver = driver;
    s->ctx = ctx;
    s->refs = 1;
    return 0;
}

static void robot_slot_release(robot_t *r, uint16_t idx) {
    robot_slot_t *s;

    if (!r || idx >= r->nslots) return;

    s = &r->slots[idx];
    if (!s->ctx) return;

    if (s->refs > 1) {
        s->refs--;
        return;
    }

    if (s->driver && s->driver->unbind) s->driver->unbind(s->ctx);
    s->ctx = NULL;
    s->driver = NULL;
    s->refs = 0;
}

/**
 *
 * Run validity checks on robot definition
 *
 * Steps:
 *  1) Validate each peripheral individualy
 *  2) All peripheral MUST HAVE a NAME
 *  3) The Name MUST BE UNIQUE
 */
static int robot_def_validate(const robot_def_t *def) {
    size_t i, j;

    if (!def || !def->peripherals) return -EINVAL;

    for (i = 0; i < def->num_peripherals; i++) {
        const peripheral_desc_t *p = &def->peripherals[i];
        peripheral_error_t pe = peripheral_validate_basic(p);

        // 1) verify each peripheral
        if (pe != PERIPH_OK) {
            fprintf(stderr,
                    "error: peripheral idx=%zu type=%s name=%s is invalid (error %d) %s\n",
                    i, type_to_string(p->type), p->name ? p->name : "(null)",
                    (int)pe, peripheral_error_to_string(pe));
            return -EINVAL;
        }

        // 2) Check name
        if (!p->name || !p->name[0]) {
            fprintf(stderr,
                    "error: peripheral idx=%zu type=%s has empty name. All peripherals must have a name\n",
                    i, type_to_string(p->type));
            return -EINVAL;
        }

        // 3) Check name unique
        for (j = i + 1; j < def->num_peripherals; j++) {
            const peripheral_desc_t *b = &def->peripherals[j];
            if (p->type != b->type) continue;
            if (strcmp(p->name, b->name) == 0) {
                fprintf(stderr,
                        "error: duplicate peripheral name (type=%s name=%s) at idx=%zu and idx=%zu\n",
                        type_to_string(p->type), p->name, i, j);
                return -EINVAL;
            }
        }
    }
    return 0;
}

/* ========================================================================================
 * PUBLIC ROBOT FUNCTIONS
 * ======================================================================================== */

FOXY_API robot_t robot_init(void) {
    robot_t r;
    platform_t platform;
    char human[256];
    hat_t hat;
    const robot_def_t *def = NULL;
    size_t i;

    memset(&r, 0, sizeof(r));

    if (platform_detect(&platform, human, sizeof(human)) != 0) {
        r.error = -ENODEV;
        return r;
    }

    (void)human; /* optional informational string */

    /* TODO: detect HAT at runtime */
    // Harley 15 feb: IDK why the built-in EEPROM
    // does not show up in the I2C bus.
    hat = HAT_V3_15;

    if (robot_def_get(platform, hat, &def) != 0 || !def) {
        r.error = -ENODEV;
        return r;
    }

    r.def = def;

    if (robot_def_validate(def) != 0) {
        r.error = -EINVAL;
        r.def = NULL;
        r.nslots = 0;
        return r;
    }

    r.nslots = def->num_peripherals;
    if (r.nslots > ROBOT_MAX_PERIPHERALS) {
        r.error = -ENOMEM;
        r.def = NULL;
        r.nslots = 0;
        return r;
    }

    for (i = 0; i < r.nslots; i++) {
        r.slots[i].driver = NULL;
        r.slots[i].ctx = NULL;
        r.slots[i].refs = 0;
    }

    r.error = 0;
    return r;
}

FOXY_API void robot_deinit(robot_t *r) {
    size_t i;
    if (!r || !r->def) return;

    for (i = 0; i < r->nslots; i++) {
        while (r->slots[i].ctx && r->slots[i].refs) {
            robot_slot_release(r, (uint16_t)i);
        }
    }
    memset(r, 0, sizeof(*r));
}

/* ========================================================================================
 * DRIVER IMPLEMENTATIONS
 * ======================================================================================== */

/* ---------------------- LED (PCA9685) ---------------------- */

typedef struct {
    int fd;
    uint8_t addr7;
} pca9685_led_ctx_t;

static int led_pca9685_set_rgb(void *p, uint8_t idx, uint8_t r, uint8_t g, uint8_t b) {
    pca9685_led_ctx_t *ctx = (pca9685_led_ctx_t *)p;
    if (!ctx) return -EINVAL;
    if (pca9685_set_rgb(ctx->fd, ctx->addr7, idx, r, g, b) < 0) return -errno;
    return 0;
}

static int led_pca9685_bind(const peripheral_desc_t *desc, void **out_ctx) {
    const char *adapter;
    uint8_t addr7;
    double pwm_hz = 1000.0;
    double gamma = 2.2;
    int fd;
    pca9685_led_ctx_t *ctx;
    static bool gamma_inited = false;

    if (!desc || !out_ctx) return -EINVAL;
    if (desc->type != PERIPH_LED) return -EINVAL;
    if (desc->primary.iface != IFACE_I2C) return -EINVAL;

    adapter = desc->primary.u.i2c.adapter;
    addr7   = (uint8_t)(desc->primary.u.i2c.addr & 0x7F);

    (void)peripheral_prop_get_double(desc, "pwm_hz", &pwm_hz);
    (void)peripheral_prop_get_double(desc, "gamma", &gamma);

    fd = open_i2c_fd(adapter);
    if (fd < 0) return fd;

    if (!gamma_inited) {
        gamma_init(gamma);
        gamma_inited = true;
    }

    if (pca9685_init(fd, addr7, pwm_hz) < 0) {
        int e = errno ? -errno : -EIO;
        close(fd);
        return e;
    }

    ctx = (pca9685_led_ctx_t *)calloc(1, sizeof(*ctx));
    if (!ctx) { close(fd); return -ENOMEM; }

    ctx->fd = fd;
    ctx->addr7 = addr7;
    *out_ctx = ctx;
    return 0;
}

static void led_pca9685_unbind(void *p) {
    pca9685_led_ctx_t *ctx = (pca9685_led_ctx_t *)p;
    if (!ctx) return;
    if (ctx->fd >= 0) close(ctx->fd);
    free(ctx);
}

/* ---------------------- IMU (MPU6050) ---------------------- */

typedef struct {
    int fd;
    mpu6050_t dev;
} mpu6050_imu_ctx_t;

static int imu_mpu6050_read(void *p, imu_sample_t *out) {
    mpu6050_imu_ctx_t *ctx;
    mpu6050_raw_t raw;
    mpu6050_si_t si;

    if (!p || !out) return -EINVAL;

    ctx = (mpu6050_imu_ctx_t *)p;
    if (mpu6050_read_raw(ctx->fd, &ctx->dev, &raw) < 0) return -errno;

    mpu6050_convert_si(&ctx->dev, &raw, &si);

    out->accel_ms2[0] = si.ax_ms2;
    out->accel_ms2[1] = si.ay_ms2;
    out->accel_ms2[2] = si.az_ms2;

    out->gyro_dps[0] = si.gx_dps;
    out->gyro_dps[1] = si.gy_dps;
    out->gyro_dps[2] = si.gz_dps;

    out->mag_uT[0] = 0.0f;
    out->mag_uT[1] = 0.0f;
    out->mag_uT[2] = 0.0f;

    out->temp_c = si.temp_c;
    return 0;
}

static int imu_mpu6050_bind(const peripheral_desc_t *desc, void **out_ctx) {
    const char *adapter;
    uint8_t addr7;
    mpu6050_t dev;
    uint32_t v = 0;
    int fd;
    mpu6050_imu_ctx_t *ctx;

    if (!desc || !out_ctx) return -EINVAL;
    if (desc->type != PERIPH_IMU) return -EINVAL;
    if (desc->primary.iface != IFACE_I2C) return -EINVAL;

    adapter = desc->primary.u.i2c.adapter;
    addr7   = (uint8_t)(desc->primary.u.i2c.addr & 0x7F);

    dev.addr7 = addr7;
    dev.accel_range = MPU6050_ACCEL_4G;
    dev.gyro_range = MPU6050_GYRO_500DPS;
    dev.dlpf_cfg = 3;
    dev.sample_rate_div = 9;
    dev.accel_lsb_per_g = 0.0f;
    dev.gyro_lsb_per_dps = 0.0f;

    if (peripheral_prop_get_u32(desc, "accel_range", &v) == 0) {
        if (v > 3) v = 3;
        dev.accel_range = (mpu6050_accel_range_t)v;
    }
    if (peripheral_prop_get_u32(desc, "gyro_range", &v) == 0) {
        if (v > 3) v = 3;
        dev.gyro_range = (mpu6050_gyro_range_t)v;
    }
    if (peripheral_prop_get_u32(desc, "dlpf_cfg", &v) == 0) {
        if (v > 6) v = 6;
        dev.dlpf_cfg = (uint8_t)v;
    }
    if (peripheral_prop_get_u32(desc, "sample_rate_div", &v) == 0) {
        if (v > 255) v = 255;
        dev.sample_rate_div = (uint8_t)v;
    }

    fd = open_i2c_fd(adapter);
    if (fd < 0) return fd;

    if (mpu6050_init(fd, &dev) < 0) {
        int e = errno ? -errno : -EIO;
        close(fd);
        return e;
    }

    ctx = (mpu6050_imu_ctx_t *)calloc(1, sizeof(*ctx));
    if (!ctx) { close(fd); return -ENOMEM; }

    ctx->fd = fd;
    ctx->dev = dev;
    *out_ctx = ctx;
    return 0;
}

static void imu_mpu6050_unbind(void *p) {
    mpu6050_imu_ctx_t *ctx = (mpu6050_imu_ctx_t *)p;
    if (!ctx) return;
    if (ctx->fd >= 0) close(ctx->fd);
    free(ctx);
}

/* ---------------------- GPIO (linux gpio chardev v1) ---------------------- */

typedef struct {
    int chip_fd;
    int line_fd;
    char chip_path[128];
    uint32_t offset;

    bool active_low;
    bool is_output;
    int last_output;
} gpiochip_line_ctx_t;

static void gpiochip_line_close_line(gpiochip_line_ctx_t *ctx) {
    if (!ctx) return;
    if (ctx->line_fd >= 0) close(ctx->line_fd);
    ctx->line_fd = -1;
}

static int gpiochip_line_request(gpiochip_line_ctx_t *ctx) {
    struct gpiohandle_request req;

    if (!ctx || ctx->chip_fd < 0) return -EINVAL;

    gpiochip_line_close_line(ctx);
    memset(&req, 0, sizeof(req));

    req.lineoffsets[0] = ctx->offset;
    req.lines = 1;
    req.flags = ctx->is_output ? GPIOHANDLE_REQUEST_OUTPUT : GPIOHANDLE_REQUEST_INPUT;
    if (ctx->active_low) req.flags |= GPIOHANDLE_REQUEST_ACTIVE_LOW;
    if (ctx->is_output) req.default_values[0] = (ctx->last_output ? 1 : 0);

    snprintf(req.consumer_label, sizeof(req.consumer_label), "robot-gpio");

    if (ioctl(ctx->chip_fd, GPIO_GET_LINEHANDLE_IOCTL, &req) < 0) return -errno;

    ctx->line_fd = req.fd;
    return 0;
}

static int gpiochip_line_set_dir(void *p, bool output) {
    gpiochip_line_ctx_t *ctx = (gpiochip_line_ctx_t *)p;
    if (!ctx) return -EINVAL;
    if (ctx->is_output == output && ctx->line_fd >= 0) return 0;
    ctx->is_output = output;
    return gpiochip_line_request(ctx);
}

static int gpiochip_line_set_active_low(void *p, bool active_low) {
    gpiochip_line_ctx_t *ctx = (gpiochip_line_ctx_t *)p;
    if (!ctx) return -EINVAL;
    if (ctx->active_low == active_low && ctx->line_fd >= 0) return 0;
    ctx->active_low = active_low;
    return gpiochip_line_request(ctx);
}

static int gpiochip_line_read(void *p, int *out_value) {
    gpiochip_line_ctx_t *ctx = (gpiochip_line_ctx_t *)p;
    struct gpiohandle_data data;

    if (!ctx || !out_value) return -EINVAL;
    if (ctx->line_fd < 0) return -ENODEV;

    memset(&data, 0, sizeof(data));
    if (ioctl(ctx->line_fd, GPIOHANDLE_GET_LINE_VALUES_IOCTL, &data) < 0) return -errno;

    *out_value = data.values[0] ? 1 : 0;
    return 0;
}

static int gpiochip_line_write(void *p, int value) {
    gpiochip_line_ctx_t *ctx = (gpiochip_line_ctx_t *)p;
    struct gpiohandle_data data;

    if (!ctx) return -EINVAL;
    if (ctx->line_fd < 0) return -ENODEV;
    if (!ctx->is_output) return -EPERM;

    memset(&data, 0, sizeof(data));
    data.values[0] = value ? 1 : 0;

    if (ioctl(ctx->line_fd, GPIOHANDLE_SET_LINE_VALUES_IOCTL, &data) < 0) return -errno;

    ctx->last_output = value ? 1 : 0;
    return 0;
}

static int gpiochip_line_bind(const peripheral_desc_t *desc, void **out_ctx) {
    const char *chip;
    uint32_t offset;
    bool active_low;
    int chip_fd;
    gpiochip_line_ctx_t *ctx;
    int rc;

    if (!desc || !out_ctx) return -EINVAL;
    if (desc->type != PERIPH_GPIO) return -EINVAL;
    if (desc->primary.iface != IFACE_GPIO) return -EINVAL;

    chip = desc->primary.u.gpio.line.chip;
    offset = desc->primary.u.gpio.line.offset;
    active_low = desc->primary.u.gpio.line.active_low;

    if (!chip || !chip[0]) return -EINVAL;

    chip_fd = open(chip, O_RDONLY | O_CLOEXEC);
    if (chip_fd < 0) return -errno;

    ctx = (gpiochip_line_ctx_t *)calloc(1, sizeof(*ctx));
    if (!ctx) { close(chip_fd); return -ENOMEM; }

    ctx->chip_fd = chip_fd;
    ctx->line_fd = -1;
    ctx->offset = offset;
    ctx->active_low = active_low;
    ctx->is_output = false;
    ctx->last_output = 0;
    snprintf(ctx->chip_path, sizeof(ctx->chip_path), "%s", chip);

    rc = gpiochip_line_request(ctx);
    if (rc < 0) {
        gpiochip_line_close_line(ctx);
        close(ctx->chip_fd);
        free(ctx);
        return rc;
    }

    *out_ctx = ctx;
    return 0;
}

static void gpiochip_line_unbind(void *p) {
    gpiochip_line_ctx_t *ctx = (gpiochip_line_ctx_t *)p;
    if (!ctx) return;
    gpiochip_line_close_line(ctx);
    if (ctx->chip_fd >= 0) close(ctx->chip_fd);
    free(ctx);
}

/* ---------------------- MOTOR (H-bridge) ---------------------- */

static float clampf(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static uint16_t duty_from_power(float power, float max_duty) {
    long v;
    power = clampf(power, 0.0f, 1.0f);
    v = lroundf(power * max_duty);
    if (v < 0) v = 0;
    if (v > (long)max_duty) v = (long)max_duty;
    return (uint16_t)v;
}

static int pca_set_digital(int fd, uint8_t addr7, uint8_t ch, int level) {
    return pca9685_set_pwm(fd, addr7, ch, 0, level ? PCA9685_TICKS_MAX : 0);
}

typedef enum {
    MOTOR_DIR_PCA  = 0,
    MOTOR_DIR_GPIO = 1,
} motor_dir_backend_t;

typedef struct {
    int fd;
    uint8_t addr7;
    uint8_t ch_pwm;
    bool invert;
    motor_dir_backend_t dir_kind;
    union {
        struct { uint8_t ch_in1; uint8_t ch_in2; } pca;
        struct { gpiochip_line_ctx_t in1, in2; } gpio;
    } dir;
} motor_hbridge_ctx_t;

static int motor_set_dir(motor_hbridge_ctx_t *m, int in1, int in2) {
    if (!m) return -EINVAL;

    if (m->dir_kind == MOTOR_DIR_PCA) {
        if (pca_set_digital(m->fd, m->addr7, m->dir.pca.ch_in1, in1) < 0) return -errno;
        if (pca_set_digital(m->fd, m->addr7, m->dir.pca.ch_in2, in2) < 0) return -errno;
        return 0;
    }

    if (m->dir_kind == MOTOR_DIR_GPIO) {
        int rc;
        rc = gpiochip_line_write(&m->dir.gpio.in1, in1);
        if (rc < 0) return rc;
        rc = gpiochip_line_write(&m->dir.gpio.in2, in2);
        if (rc < 0) return rc;
        return 0;
    }

    return -EINVAL;
}

static int motor_set_pwm(motor_hbridge_ctx_t *m, uint16_t duty) {
    if (!m) return -EINVAL;
    if (pca9685_set_pwm(m->fd, m->addr7, m->ch_pwm, 0, duty) < 0) return -errno;
    return 0;
}

static int gpiochip_line_ctx_open_output(gpiochip_line_ctx_t *ctx,
                                         const gpio_desc_t *line,
                                         int initial_value) {
    int chip_fd;
    int rc;

    if (!ctx || !line || !line->chip) return -EINVAL;
    memset(ctx, 0, sizeof(*ctx));

    chip_fd = open(line->chip, O_RDONLY | O_CLOEXEC);
    if (chip_fd < 0) return -errno;

    ctx->chip_fd = chip_fd;
    ctx->line_fd = -1;
    ctx->offset = line->offset;
    ctx->active_low = line->active_low;
    ctx->is_output = true;
    ctx->last_output = initial_value ? 1 : 0;
    snprintf(ctx->chip_path, sizeof(ctx->chip_path), "%s", line->chip);

    rc = gpiochip_line_request(ctx);
    if (rc < 0) {
        if (ctx->chip_fd >= 0) close(ctx->chip_fd);
        ctx->chip_fd = -1;
        return rc;
    }
    return 0;
}

static void gpiochip_line_ctx_close_all(gpiochip_line_ctx_t *ctx) {
    if (!ctx) return;
    gpiochip_line_close_line(ctx);
    if (ctx->chip_fd >= 0) close(ctx->chip_fd);
    ctx->chip_fd = -1;
}

static int motor_hbridge_stop(void *p) {
    motor_hbridge_ctx_t *m = (motor_hbridge_ctx_t *)p;
    int rc;
    if (!m) return -EINVAL;
    rc = motor_set_pwm(m, 0);
    if (rc < 0) return rc;
    return motor_set_dir(m, 0, 0);
}

static int motor_hbridge_brake(void *p) {
    motor_hbridge_ctx_t *m = (motor_hbridge_ctx_t *)p;
    int rc;
    if (!m) return -EINVAL;
    rc = motor_set_pwm(m, 0);
    if (rc < 0) return rc;
    return motor_set_dir(m, 1, 1);
}

static int motor_hbridge_set(void *p, float power) {
    motor_hbridge_ctx_t *m = (motor_hbridge_ctx_t *)p;
    float a;
    uint16_t duty;
    int in1, in2;
    int rc;

    if (!m) return -EINVAL;

    power = clampf(power, -1.0f, 1.0f);
    if (m->invert) power = -power;

    a = fabsf(power);
    if (a <= 0.0001f) return motor_hbridge_stop(p);

    duty = duty_from_power(a, (float)PCA9685_TICKS_MAX);
    in1 = (power >= 0.0f) ? 1 : 0;
    in2 = (power >= 0.0f) ? 0 : 1;

    rc = motor_set_dir(m, in1, in2);
    if (rc < 0) return rc;
    return motor_set_pwm(m, duty);
}

static int motor_hbridge_bind(const peripheral_desc_t *desc, void **out_ctx) {
    const char *adapter;
    uint8_t addr7;
    uint32_t ch_pwm_u = 0;
    uint32_t invert_u = 0;
    bool invert;
    int fd;
    motor_hbridge_ctx_t *m;
    const peripheral_aux_t *in1, *in2;
    int rc;

    if (!desc || !out_ctx) return -EINVAL;
    if (desc->type != PERIPH_MOTOR) return -EINVAL;
    if (desc->primary.iface != IFACE_I2C) return -EINVAL;

    adapter = desc->primary.u.i2c.adapter;
    addr7 = (uint8_t)(desc->primary.u.i2c.addr & 0x7F);

    if (peripheral_prop_get_u32(desc, "pca9685_ch_pwm", &ch_pwm_u) != 0 || ch_pwm_u > 15) return -EINVAL;

    (void)peripheral_prop_get_u32(desc, "invert", &invert_u);
    invert = (invert_u != 0);

    fd = open_i2c_fd(adapter);
    if (fd < 0) return fd;

    if (pca9685_init(fd, addr7, 1000.0) < 0) {
        int e = errno ? -errno : -EIO;
        close(fd);
        return e;
    }

    m = (motor_hbridge_ctx_t *)calloc(1, sizeof(*m));
    if (!m) { close(fd); return -ENOMEM; }

    m->fd = fd;
    m->addr7 = addr7;
    m->ch_pwm = (uint8_t)ch_pwm_u;
    m->invert = invert;

    in1 = peripheral_get_aux(desc, ENDPOINT_ROLE_AUX0);
    in2 = peripheral_get_aux(desc, ENDPOINT_ROLE_AUX1);

    if (in1 && in2) {
        if (in1->iface != IFACE_GPIO || in2->iface != IFACE_GPIO) {
            free(m);
            close(fd);
            return -EINVAL;
        }

        m->dir_kind = MOTOR_DIR_GPIO;

        rc = gpiochip_line_ctx_open_output(&m->dir.gpio.in1, &in1->u.gpio.line, 0);
        if (rc < 0) {
            free(m);
            close(fd);
            return rc;
        }

        rc = gpiochip_line_ctx_open_output(&m->dir.gpio.in2, &in2->u.gpio.line, 0);
        if (rc < 0) {
            gpiochip_line_ctx_close_all(&m->dir.gpio.in1);
            free(m);
            close(fd);
            return rc;
        }
    } else {
        uint32_t ch_in1_u = 0, ch_in2_u = 0;

        if (peripheral_prop_get_u32(desc, "pca9685_ch_in1", &ch_in1_u) != 0) {
            free(m); close(fd); return -EINVAL;
        }
        if (peripheral_prop_get_u32(desc, "pca9685_ch_in2", &ch_in2_u) != 0) {
            free(m); close(fd); return -EINVAL;
        }
        if (ch_in1_u > 15 || ch_in2_u > 15) {
            free(m); close(fd); return -EINVAL;
        }

        m->dir_kind = MOTOR_DIR_PCA;
        m->dir.pca.ch_in1 = (uint8_t)ch_in1_u;
        m->dir.pca.ch_in2 = (uint8_t)ch_in2_u;
    }

    (void)motor_hbridge_stop(m);

    *out_ctx = m;
    return 0;
}

static void motor_hbridge_unbind(void *p) {
    motor_hbridge_ctx_t *m = (motor_hbridge_ctx_t *)p;
    if (!m) return;

    (void)motor_hbridge_stop(m);

    if (m->dir_kind == MOTOR_DIR_GPIO) {
        gpiochip_line_ctx_close_all(&m->dir.gpio.in1);
        gpiochip_line_ctx_close_all(&m->dir.gpio.in2);
    }

    if (m->fd >= 0) close(m->fd);
    free(m);
}

/* ========================================================================================
 * Resource handles - Capabilities, Init, Close and binding to implementations.
 * ======================================================================================== */

/* ---------------------- LED resource ---------------------- */

typedef struct led_ops led_ops_t;
struct led_ops {
    int (*set_rgb)(void *ctx, uint8_t idx, uint8_t r, uint8_t g, uint8_t b);
};

static const led_ops_t led_pca9685_ops = {
    .set_rgb = led_pca9685_set_rgb,
};

FOXY_API int led_set_rgb(led_t led, uint8_t idx, uint8_t r, uint8_t g, uint8_t b) {
    const led_ops_t *ops = (const led_ops_t *)led.ops;
    if (!ops || !ops->set_rgb || !led.ctx) return -ENODEV;
    return ops->set_rgb(led.ctx, idx, r, g, b);
}

/* ---------------------- IMU resource ---------------------- */

typedef struct imu_ops imu_ops_t;
struct imu_ops {
    int (*read)(void *ctx, imu_sample_t *out);
};

static const imu_ops_t imu_mpu6050_ops = {
    .read = imu_mpu6050_read,
};

static imu_sample_t imu_sample_zero(void) {
    imu_sample_t s;
    memset(&s, 0, sizeof(s));
    return s;
}

FOXY_API imu_sample_t imu_read(imu_t imu) {
    imu_sample_t s = imu_sample_zero();
    const imu_ops_t *ops = (const imu_ops_t *)imu.ops;
    if (!ops || !ops->read || !imu.ctx) return s;
    (void)ops->read(imu.ctx, &s);
    return s;
}

/* ---------------------- GPIO resource ---------------------- */

typedef struct gpio_ops gpio_ops_t;
struct gpio_ops {
    int (*set_dir)(void *ctx, bool output);
    int (*set_active_low)(void *ctx, bool active_low);
    int (*read)(void *ctx, int *out_value);
    int (*write)(void *ctx, int value);
};

static const gpio_ops_t gpiochip_line_ops = {
    .set_dir = gpiochip_line_set_dir,
    .set_active_low = gpiochip_line_set_active_low,
    .read = gpiochip_line_read,
    .write = gpiochip_line_write,
};

FOXY_API int gpio_set_as_input(gpio_t g) {
    const gpio_ops_t *ops = (const gpio_ops_t *)g.ops;
    if (!ops || !ops->set_dir || !g.ctx) return -ENODEV;
    return ops->set_dir(g.ctx, false);
}

FOXY_API int gpio_set_as_output(gpio_t g) {
    const gpio_ops_t *ops = (const gpio_ops_t *)g.ops;
    if (!ops || !ops->set_dir || !g.ctx) return -ENODEV;
    return ops->set_dir(g.ctx, true);
}

FOXY_API int gpio_set_active_low(gpio_t g) {
    const gpio_ops_t *ops = (const gpio_ops_t *)g.ops;
    if (!ops || !ops->set_active_low || !g.ctx) return -ENODEV;
    return ops->set_active_low(g.ctx, true);
}

FOXY_API int gpio_set_active_high(gpio_t g) {
    const gpio_ops_t *ops = (const gpio_ops_t *)g.ops;
    if (!ops || !ops->set_active_low || !g.ctx) return -ENODEV;
    return ops->set_active_low(g.ctx, false);
}

FOXY_API int gpio_read(gpio_t g) {
    const gpio_ops_t *ops = (const gpio_ops_t *)g.ops;
    int v = 0;
    int rc;
    if (!ops || !ops->read || !g.ctx) return -ENODEV;
    rc = ops->read(g.ctx, &v);
    if (rc < 0) return rc;
    return (v != 0);
}

FOXY_API int gpio_write(gpio_t g, int value) {
    const gpio_ops_t *ops = (const gpio_ops_t *)g.ops;
    if (!ops || !ops->write || !g.ctx) return -ENODEV;
    return ops->write(g.ctx, value ? 1 : 0);
}

/* ---------------------- Motor resource ---------------------- */

typedef struct motor_ops motor_ops_t;
struct motor_ops {
    int (*set)(void *ctx, float power);
    int (*stop)(void *ctx);
    int (*brake)(void *ctx);
};

static const motor_ops_t motor_hbridge_ops = {
    .set = motor_hbridge_set,
    .stop = motor_hbridge_stop,
    .brake = motor_hbridge_brake,
};

FOXY_API int motor_set(motor_t m, float power) {
    const motor_ops_t *ops = (const motor_ops_t *)m.ops;
    if (!ops || !ops->set || !m.ctx) return -ENODEV;
    return ops->set(m.ctx, power);
}

FOXY_API int motor_stop(motor_t m) {
    const motor_ops_t *ops = (const motor_ops_t *)m.ops;
    if (!ops || !ops->stop || !m.ctx) return -ENODEV;
    return ops->stop(m.ctx);
}

FOXY_API int motor_brake(motor_t m) {
    const motor_ops_t *ops = (const motor_ops_t *)m.ops;
    if (!ops || !m.ctx) return -ENODEV;
    if (ops->brake) return ops->brake(m.ctx);
    return motor_stop(m);
}

/* ---------------------- Resource acquisition ---------------------- */

static resource_t resource_open(robot_t *r, peripheral_type_t type, const char *name, const char *api_name) {
    resource_t out;
    uint16_t idx;
    robot_slot_t *s;

    out.ops = NULL;
    out.ctx = NULL;
    out._idx = 0;

    if (!r || !r->def) return out;
    if (robot_find_index(r, type, name, api_name, &idx) != 0) return out;
    if (robot_slot_acquire(r, idx) != 0) return out;

    s = &r->slots[idx];
    if (!s->ctx || !s->driver || !s->driver->ops) {
        robot_slot_release(r, idx);
        return out;
    }

    out.ops = s->driver->ops;
    out.ctx = s->ctx;
    out._idx = idx;
    return out;
}

static void resource_close(robot_t *r, resource_t *h) {
    if (!r || !h || !h->ctx) return;
    if (h->_idx < r->nslots) robot_slot_release(r, h->_idx);
    h->ops = NULL;
    h->ctx = NULL;
    h->_idx = 0;
}

FOXY_API led_t led_init_name(robot_t *r, const char *name) {
    return resource_open(r, PERIPH_LED, name, "led_init");
}

FOXY_API void led_deinit(robot_t *r, led_t *h) {
    resource_close(r, h);
}

FOXY_API imu_t imu_init_name(robot_t *r, const char *name) {
    return resource_open(r, PERIPH_IMU, name, "imu_init");
}

FOXY_API void imu_deinit(robot_t *r, imu_t *h) {
    resource_close(r, h);
}

FOXY_API gpio_t gpio_init_name(robot_t *r, const char *name) {
    return resource_open(r, PERIPH_GPIO, name, "gpio_init");
}

FOXY_API void gpio_deinit(robot_t *r, gpio_t *h) {
    resource_close(r, h);
}

FOXY_API motor_t motor_init_name(robot_t *r, const char *name) {
    return resource_open(r, PERIPH_MOTOR, name, "motor_init");
}

FOXY_API void motor_deinit(robot_t *r, motor_t *m) {
    resource_close(r, m);
}

/* ========================================================================================
 * DRIVERS REGISTRY
 * ======================================================================================== */

// static const peripheral_driver_t 
const peripheral_driver_t global_drivers[] = {
    { .name="pca9685",      .type=PERIPH_LED,   .bind=led_pca9685_bind,   .unbind=led_pca9685_unbind, .ops=&led_pca9685_ops },
    { .name="mpu6050",      .type=PERIPH_IMU,   .bind=imu_mpu6050_bind,   .unbind=imu_mpu6050_unbind, .ops=&imu_mpu6050_ops },
    { .name="gpio",         .type=PERIPH_GPIO,  .bind=gpiochip_line_bind, .unbind=gpiochip_line_unbind, .ops=&gpiochip_line_ops },
    { .name="motor_hbridge",.type=PERIPH_MOTOR, .bind=motor_hbridge_bind, .unbind=motor_hbridge_unbind, .ops=&motor_hbridge_ops },
};

const size_t global_num_drivers = sizeof(global_drivers) / sizeof(global_drivers[0]);

#ifdef __cplusplus
} /* extern "C" */
#endif
#endif /* FOXY_IMPLEMENTATION_ONCE */
#endif /* FOXY_IMPLEMENTATION */
