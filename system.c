/*
 * Build:
 *   gcc -std=c11 -Wall -Wextra -O2 system.c -o systm
 */

#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdbool.h>

#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#include <math.h>
#include <time.h>

/**
 * High-level platform family.
 * @PLATFORM_UNKNOWN: Unknown or not detected.
 * @PLATFORM_JETSON: NVIDIA Jetson family.
 * @PLATFORM_RASPBERRY: Raspberry Pi family.
 */
typedef enum {
	PLATFORM_UNKNOWN   = 0,
	PLATFORM_JETSON    = 1,
	PLATFORM_RASPBERRY = 2,
} platform_family_t;

/** Given the current platform family, 
 * This define the specific model of 
 * with in a platform family.
 */

typedef enum {
	JETSON_MODEL_UNKNOWN   = 0,
	JETSON_MODEL_NANO      = 1,
	JETSON_MODEL_XAVIER_NX = 2,
	JETSON_MODEL_ORIN_NANO = 3,
	JETSON_MODEL_ORIN_NX   = 4,
	JETSON_MODEL_AGX_ORIN  = 5
} jetson_model_t;

typedef enum {
	RPI_MODEL_UNKNOWN = 0,
	RPI_MODEL_PI4     = 1,
	RPI_MODEL_PI5     = 2,
	RPI_MODEL_CM4     = 3
} rpi_model_t;

/**
 * struct platform - Detected platform identity.
 * @family: platform family
 * @model: family-specific model code (cast based on family)
 */
typedef struct {
	platform_family_t family;
	uint16_t model;
} platform_t;

/** The HAT on the robot represents
 * the "physical hardware contract" that
 * define how to interact with peripherals
 * that are connected to the HAT.
 *
 * e.g. IMU, buttons, leds, etc.
 *
 * this "physical hardware contract" can
 * be broken if the routering of the peripherals
 * change on hardware revision of the HAT.
 *
 * This should not happen frequently,
 * but it is good to have a safety net,
 * and this enum defines which "hardware contract"
 * is currently in effect.
 *
 * In case there is a change that breaks the contract
 * add a new entry to this enum.
 */
typedef enum {
	HAT_UNKNOWN   = 0,
    HAT_V3_15     = 1, // HAT revision v3.15
} hat_t;

/* ---------- Small helpers ---------- */

static void rstrip(char *s)
{
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

static int ascii_tolower(int c)
{
	if (c >= 'A' && c <= 'Z') return c + ('a' - 'A');
	return c;
}

static int contains_ci(const char *haystack, const char *needle)
{
	/* simple case-insensitive substring search (ASCII) */
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

/**
 * read_file_bytes - Read a file into a buffer.
 * @path: path to read
 * @buf: output buffer
 * @bufsz: size of @buf
 *
 * Return: number of bytes read (>=0), or -1 on error.
 */
static ssize_t read_file_bytes(const char *path, uint8_t *buf, size_t bufsz)
{
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
 * read_dt_string - Read a Device Tree string property (NUL-terminated in file).
 * @path: e.g. "/proc/device-tree/model"
 * @out: output string
 * @outsz: size of @out
 *
 * Return: 0 on success, -1 on error.
 */
static int read_dt_string(const char *path, char *out, size_t outsz)
{
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

	/* DT strings are NUL-terminated; find first NUL or use n */
	len = 0;
	while (len < (size_t)n && tmp[len] != '\0') len++;

	if (len >= outsz) len = outsz - 1;
	memcpy(out, tmp, len);
	out[len] = '\0';
	rstrip(out);
	return 0;
}

/**
 * dt_compatible_contains - Check if any NUL-separated string in "compatible" matches.
 * @needle: substring to find (case-insensitive)
 *
 * Return: 1 if found, 0 if not found or file not readable.
 */
static int dt_compatible_contains(const char *needle)
{
	uint8_t buf[1024];
	ssize_t n = read_file_bytes("/proc/device-tree/compatible", buf, sizeof(buf));
	size_t i = 0;

	if (n <= 0 || !needle) return 0;

	while (i < (size_t)n) {
		const char *s = (const char *)&buf[i];
		size_t max = (size_t)n - i;
		size_t len = strnlen(s, max);
		if (len == 0) break;

		if (contains_ci(s, needle)) return 1;
		i += len + 1;
	}
	return 0;
}

/* ---------- Model parsing heuristics ---------- */

static jetson_model_t parse_jetson_model(const char *model_str)
{
	/* Check specific tokens first (avoid "Orin Nano" matching "Nano") */
	if (model_str && contains_ci(model_str, "AGX Orin"))   return JETSON_MODEL_AGX_ORIN;
	if (model_str && contains_ci(model_str, "Orin NX"))    return JETSON_MODEL_ORIN_NX;
	if (model_str && contains_ci(model_str, "Orin Nano"))  return JETSON_MODEL_ORIN_NANO;
	if (model_str && contains_ci(model_str, "Xavier NX"))  return JETSON_MODEL_XAVIER_NX;
	if (model_str && contains_ci(model_str, "Jetson Nano"))return JETSON_MODEL_NANO;

	/* Also try hints from compatible (common on Jetson). */
	if (dt_compatible_contains("jetson-agx-orin")) return JETSON_MODEL_AGX_ORIN;
	if (dt_compatible_contains("jetson-orin-nx"))  return JETSON_MODEL_ORIN_NX;
	if (dt_compatible_contains("jetson-orin-nano"))return JETSON_MODEL_ORIN_NANO;
	if (dt_compatible_contains("jetson-xavier-nx"))return JETSON_MODEL_XAVIER_NX;
	if (dt_compatible_contains("jetson-nano"))     return JETSON_MODEL_NANO;

	return JETSON_MODEL_UNKNOWN;
}

static rpi_model_t parse_rpi_model(const char *model_str)
{
	if (model_str && contains_ci(model_str, "Raspberry Pi 5")) return RPI_MODEL_PI5;
	if (model_str && contains_ci(model_str, "Raspberry Pi 4")) return RPI_MODEL_PI4;
	if (model_str && contains_ci(model_str, "Compute Module 4"))return RPI_MODEL_CM4;
	return RPI_MODEL_UNKNOWN;
}

/* ---------- DMI fallback for x86 ---------- */

static int read_sysfs_string_first(char *out, size_t outsz, const char *a, const char *b)
{
	/* Try path a, then b */
	if (read_dt_string(a, out, outsz) == 0) return 0;
	if (b && read_dt_string(b, out, outsz) == 0) return 0;
	return -1;
}

/* ---------- Public API ---------- */

/**
 * platform_detect - Detect platform family + model on Linux.
 * @out: output platform struct
 * @human: optional output buffer for a human-readable model string (may be NULL)
 * @humansz: size of @human
 *
 * Detection order:
 *   1) Device Tree (/proc/device-tree/...) for embedded boards.
 *
 * Return: 0 on success (even if model is UNKNOWN), -1 on hard failure.
 */
static int platform_detect(platform_t *out, char *human, size_t humansz)
{
	char model[256] = {0};

	if (!out) {
		errno = EINVAL;
		return -1;
	}

	out->family = PLATFORM_UNKNOWN;
	out->model  = 0;
	if (human && humansz) human[0] = '\0';

	/* 1) Device Tree: model string is usually enough for Jetson/RPi. :contentReference[oaicite:9]{index=9} */
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

		/* Some other DT-based board */
		out->family = PLATFORM_UNKNOWN;
		out->model  = 0;
		if (human && humansz) {
			strncpy(human, model, humansz - 1);
			human[humansz - 1] = '\0';
		}
		return 0;
	}

	/* Could not detect anything useful */
	return -1;
}

static const char *family_to_string(platform_family_t f)
{
	switch (f) {
	case PLATFORM_JETSON:    return "jetson";
	case PLATFORM_RASPBERRY: return "raspberry";
	default:                 return "unknown";
	}
}

/**
 * enum peripheral_type - Robot peripheral clases.
 */
typedef enum {
    PERIPH_NON        = 0,
    PERIPH_BATTERY    = 1,
    PERIPH_MOTOR_CTRL = 2,
    PERIPH_ENCODER    = 3,
    PERIPH_TOF        = 4,
    PERIPH_DISPLAY    = 5,
    PERIPH_BUTTON     = 6,
    PERIPH_IMU        = 7,
    PERIPH_LED        = 8,
} peripheral_type_t;

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

typedef enum {
    PERIPH_FLAG_NONE     = 0,
    PERIPH_FLAG_OPTIONAL = 1u << 0,
    PERIPH_FLAG_HOTPLUG  = 1u << 1,
    PERIPH_FLAG_READONLY = 1u << 2,
} peripheral_flags_t;

/* optional space hatch for off config bits */
typedef struct {
    const char *key;
    const char *value;
} peripheral_kv_t;

/* Aux endpoint roles: unique within a peripheral. */
typedef enum {
    ENDPOINT_ROLE_NONE = 0,
    ENDPOINT_ROLE_IRQ,
    ENDPOINT_ROLE_RESET,
    ENDPOINT_ROLE_ENABLE,
    ENDPOINT_ROLE_AUX0,
    ENDPOINT_ROLE_AUX1,
} endpoint_role_t;

typedef struct {
    const char *chip; // e.g. "/dev/gpiochip0"
    uint32_t offset;    // offset
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

typedef struct {
    peripheral_iface_t iface;
    endpoint_u_t u;
} peripheral_primary_t;

// for optional aux endpoints, unique roles, driver(s) may use or ignore.
typedef struct {
    endpoint_role_t role;
    peripheral_iface_t iface;
    endpoint_u_t u;
} peripheral_aux_t;

#define PERIPH_MAX_AUX 2

/**
 * Single peripheral description.
 *
 * The idea: this is *metadata* the rest the
 * stack can use to bind to the right driver
 * and OS resources.
 */
typedef struct {
    peripheral_type_t type;
    const char *name;   // this name is a human readable "label": "display0", "imu0", ...
    const char *driver; // bind hint: "ssd1306", ... (optional)
    uint32_t flags;     // peripheral_flags_t

    peripheral_primary_t primary;

    uint8_t num_aux;
    peripheral_aux_t aux[PERIPH_MAX_AUX];

    const peripheral_kv_t *props; // optional
    uint16_t num_props;
} peripheral_desc_t;

// Convenience helper to initlialize primary peripheral 
#define PRI_I2C(_adapter, _addr) \
    (peripheral_primary_t){.iface=IFACE_I2C, .u.i2c={ .adapter=(_adapter), .addr=(_addr) } }

#define PRI_SPI(_dev, _hz, _mode) \
    (peripheral_primary_t){.iface=IFACE_SPI, .u.spi={ .dev=(_dev), .hz=(_hz), .mode=(_mode) } }

#define PRI_UART(_dev, _baud) \
    (peripheral_primary_t){.iface=IFACE_UART, .u.uart={ .dev=(_dev), .baud=(_baud) } }

#define PRI_V4L2(_dev) \
    (peripheral_primary_t){.iface=IFACE_V4L2, .u.v4l2={ .dev=(_dev) } }

#define PRI_CSI(_port, _lanes) \
    (peripheral_primary_t){.iface=IFACE_CSI, .u.csi={.port=(_port), .lanes=(_lanes) } }

// peripheral validation helper enum
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

static peripheral_error_t peripheral_validate_basic(const peripheral_desc_t *d) {
    if (!d) return PERIPH_ERROR_PRIMARY_NONE;
    if (d->primary.iface == IFACE_NONE) return PERIPH_ERROR_PRIMARY_NONE;
    if (d->num_aux > PERIPH_MAX_AUX) return PERIPH_ERROR_AUX_COUNT;

    uint32_t seen = 0;
    for (uint8_t i = 0; i < d->num_aux; i++) {
        const peripheral_aux_t *a = &d->aux[i];
        if (a->role == ENDPOINT_ROLE_NONE) return PERIPH_ERROR_AUX_ROLE_NONE;
        if (a->iface == IFACE_NONE) return PERIPH_ERROR_AUX_IFACE_NONE;

        // using bit set to track roles and check for 
        // repited roles
        uint32_t bit = 1u << (uint32_t)a->role;
        if (seen & bit) return PERIPH_ERROR_AUX_ROLE_DUP;
        seen |= bit;
    }
    return PERIPH_OK;
}

static const peripheral_aux_t *peripheral_get_aux(const peripheral_desc_t *d, endpoint_role_t role) {
    if (!d || role == ENDPOINT_ROLE_NONE) return NULL;
    for (uint8_t i = 0; i < d->num_aux; i++) {
        if (d->aux[i].role == role) return &d->aux[i];
    }
    return NULL;
}

static const char *iface_to_string(peripheral_iface_t i) {
    switch(i) {
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

static const char *role_to_string(endpoint_role_t r) {
    switch(r) {
        case ENDPOINT_ROLE_IRQ:    return "irq";
        case ENDPOINT_ROLE_RESET:  return "reset";
        case ENDPOINT_ROLE_ENABLE: return "enable";
        case ENDPOINT_ROLE_AUX0:   return "ax0";
        case ENDPOINT_ROLE_AUX1:   return "ax1";
        default:                   return "none";
    }
}

static const char *type_to_string(peripheral_type_t t) {
    switch (t) {
        case PERIPH_BATTERY:    return "battery";
        case PERIPH_MOTOR_CTRL: return "motor_ctrl";
        case PERIPH_ENCODER:    return "encoder";
        case PERIPH_TOF:        return "tof";
        case PERIPH_DISPLAY:    return "display";
        case PERIPH_BUTTON:     return "button";
        case PERIPH_IMU:        return "imu";
        case PERIPH_LED:        return "led";
        default:                return "none";
    }
}

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
                    (unsigned)(u->spi.hz),
                    (unsigned)(u->spi.mode));
            break;
        case IFACE_UART:
            fprintf(fp, "dev=%s baud=%u",
                    u->uart.dev ? u->uart.dev : "(null)",
                    (unsigned)(u->uart.baud));
            break;
        case IFACE_GPIO:
            fprintf(fp, "chip=%s offset=%u active_low=%d",
                    u->gpio.line.chip ? u->gpio.line.chip : "(null)",
                    (unsigned)(u->gpio.line.offset),
                    u->gpio.line.active_low ? 1 : 0);
            break;
        case IFACE_PWM:
            fprintf(fp, "chip=%s channel=%u period_ns=%u",
                    u->pwm.chip ? u->pwm.chip : "(null)",
                    (unsigned)(u->pwm.channel),
                    (unsigned)(u->pwm.period_ns));
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

/**
 * Pack platform family + model + hat into a single key.
 *
 * Layout: [ family:8 | model:16 | hat:8 ]
 *
 * return: packed 32-bit key.
 */

#define ROBOT_DEF_KEY(family, model, hat) \
    (  ((uint32_t)(family & 0xFFu)   << 24) | \
       ((uint32_t)(model  & 0xFFFFu) << 8)  | \
       ((uint32_t)(hat    & 0xFFu))  )

static inline uint32_t robot_def_key(platform_family_t platform, uint16_t model, hat_t hat) {
    return ROBOT_DEF_KEY(platform, model, hat);
}

/**
 * struct robot_def - Full robot description
 */
typedef struct {
    uint32_t key;
    const char *id;
    const char *display_name;

    platform_family_t platform_family;
    uint16_t platform_model;
    hat_t hat;

    // TODO: paths and cap?
    //

    const peripheral_desc_t *peripherals;
    size_t num_peripherals;

} robot_def_t;

static const peripheral_desc_t jetson_nano_hat_v3_15[] = {
    { 
        .type = PERIPH_DISPLAY,
        .name = "display0",
        .driver = "ssd1306",
        .flags = PERIPH_FLAG_NONE,
        .primary = PRI_I2C("/dev/i2c-1", 0x3C),
        .num_aux = 0,
    }
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

int robot_def_get(platform_t platform, hat_t hat, const robot_def_t **out)
{
    uint32_t key;

    if (!out)
        // TODO: add message
        return EINVAL;

    *out = NULL;

    if (platform.family == PLATFORM_UNKNOWN || platform.model == 0 || hat == HAT_UNKNOWN)
        // TODO: add message
        return EINVAL;

    key = robot_def_key(platform.family, platform.model, hat);

    for (size_t i = 0; i < sizeof(robot_table)/sizeof(robot_table[0]); i++) {
        if (robot_table[i].key == key) {
            printf("found a match\n");
            *out = &robot_table[i];
            return 0;
        }
    }

    // TODO: add message
    return EINVAL;

}

void robot_def_dump(const robot_def_t *def, void *out) {
    FILE *fp = (FILE *)out;

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

    for (size_t i = 0; i < def->num_peripherals; i++) {
        const peripheral_desc_t *p = &def->peripherals[i];
        peripheral_error_t val_error = peripheral_validate_basic(p);

        fprintf(fp,
                "    - [%zu] type=%s name=%s driver=%s flags=0x%x",
                i,
                type_to_string(p->type),
                p->name ? p->name : "(null)",
                p->driver ? p->driver : "(null)",
                (unsigned)p->flags);

        if (val_error != PERIPH_OK) {
            fprintf(fp, " (INVALID:%d)\n", (int)val_error);
            continue;
        }
        fprintf(fp, "\n");

        fprintf(fp, "\t primary: iface=%s ", iface_to_string(p->primary.iface));
        dump_endpoint_u(fp, p->primary.iface, &p->primary.u);
        fprintf(fp, "\n");

        if (p->num_aux) {
            fprintf(fp, "\t aux:\n");
            for (uint8_t a = 0; a < p->num_aux; a++) {
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
            for (uint16_t k = 0; k < p->num_props; k++) {
                fprintf(fp, "\t - %s=%s\n",
                        p->props[k].key ? p->props[k].key : "(null)",
                        p->props[k].value ? p->props[k].value : "(null)");
            }
        }
    }
}
// ========================================================================================
/**
 * HELPERS IOCTL functions
 */
// ========================================================================================

static int i2c_rdwr(int fd, struct i2c_msg *msgs, int nmsgs) {
    struct i2c_rdwr_ioctl_data data = {
        .msgs  = msgs,
        .nmsgs = (uint32_t)nmsgs
    };

    if (ioctl(fd, I2C_RDWR, &data) < 0) return -1;
    return 0;
}

static int i2c_write_reg_bytes(int fd,
                               uint8_t addr7,
                               uint8_t reg, 
                               const uint8_t *buf,
                               size_t len) {
    uint8_t tmp[1 + 32];
    if (len > 32) { errno = EINVAL; return -1; }
    tmp[0] = reg;
    if (len) memcpy(&tmp[1], buf, len);

    struct i2c_msg msg = {
        .addr  = addr7,
        .flags = 0,
        .len   = (uint16_t)(1 + len),
        .buf   = tmp
    };

    return i2c_rdwr(fd, &msg, 1);
}

static int i2c_write_reg_u8(int fd, uint8_t addr7, uint8_t reg, uint8_t value) {
    return i2c_write_reg_bytes(fd, addr7, reg, &value, 1);
}

static int i2c_read_reg_bytes(int fd, uint8_t addr7, uint8_t reg, uint8_t *out, size_t len) {
    struct i2c_msg msgs[2];
    uint8_t r = reg;

    msgs[0].addr = addr7;
    msgs[0].flags = 0;
    msgs[0].len = 1;
    msgs[0].buf = &r;

    msgs[1].addr = addr7;
    msgs[1].flags = I2C_M_RD;
    msgs[1].len = (uint16_t)len;
    msgs[1].buf = out;

    return i2c_rdwr(fd, msgs, 2);
}

static int i2c_read_reg_u8(int fd, uint8_t addr7, uint8_t reg, uint8_t *out) {
    struct i2c_msg msgs[2];

    uint8_t r = reg;

    msgs[0].addr = addr7;
    msgs[0].flags = 0;
    msgs[0].len = 1;
    msgs[0].buf = &r;

    msgs[1].addr = addr7;
    msgs[1].flags = I2C_M_RD;
    msgs[1].len = 1;
    msgs[1].buf = out;

    return i2c_rdwr(fd, msgs, 2);
}

// ========================================================================================
/**
 * UTILITY functions
 */
// ========================================================================================

static uint64_t mono_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000ull + (uint64_t)ts.tv_nsec / 1000000ull;
}

static void sleep_ms(unsigned ms) {
    struct timespec ts;
    ts.tv_sec = (time_t)(ms / 1000);
    ts.tv_nsec = (long)((ms % 1000) * 1000000UL);
    nanosleep(&ts, NULL);
}

// this is LUT table for gamma correction
// to get "better" led colors
static uint16_t gamma_lut[256];

static void gamma_init(double gamma) {
   uint16_t res_12bit = 4095; // 12-bit PWM on PCA9685
   for (int i = 0; i < 256; i++) {
       double x = (double)i / 255.0;
       double y = pow(x, gamma);
       long v12 = lround(y * (double)(res_12bit));
       if (v12 < 0) v12 = 0;
       if (v12 > res_12bit) v12 = res_12bit;
       gamma_lut[i] = (uint16_t)v12;
   }
}



// ========================================================================================
/**
 * PERIPHERALS STRUCT PUBLIC APIs
 */
// ========================================================================================

#define PCA9685_MODE1       0x00
#define PCA9685_MODE2       0x01
#define PCA9685_PRESCALE    0xFE

#define PCA9685_LED0_ON_L     0x06
#define PCA9685_ALL_LED_ON_L  0xFA
#define PCA9685_ALL_LED_ON_H  0xFB
#define PCA9685_ALL_LED_OFF_L 0xFC
#define PCA9685_ALL_LED_OFF_H 0xFD

#define MODE1_RESTART (1u << 7)
#define MODE1_AI      (1u << 5)
#define MODE1_SLEEP   (1u << 4)

#define MODE2_OUTDRV  (1u << 2)
#define LED_FULL_ON_OFF_BIT (1u << 4)
#define PCA9685_OSC_HZ 25000000.0
#define PCA9685_TICKS_COUNT 4096
#define PCA9685_TICKS_MAX (PCA9685_TICKS_COUNT - 1)

static uint8_t prescale_for_hz(double pwm_hz) {
    // prescale  = round(osc / (4096*hz)) - 1
    // Units: Hz / (counts * Hz) => unitless
    double prescale_f = (PCA9685_OSC_HZ / (4096.0 * pwm_hz)) - 1.0;
    long prescale = lround(prescale_f);

    if (prescale < 3) prescale = 3;
    if (prescale > 255) prescale = 255;
    return (uint8_t)prescale;
}

static int led_init(int fd, uint8_t addr7, double pwm_hz) {
    if (i2c_write_reg_u8(fd, addr7, PCA9685_MODE2, MODE2_OUTDRV) < 0) {
        return -1;
    }

    uint8_t mode1 = 0;
    if (i2c_read_reg_u8(fd, addr7, PCA9685_MODE1, &mode1) < 0) {
        return -1;
    }

    uint8_t mode1_sleep = (uint8_t)((mode1 & ~MODE1_RESTART) | MODE1_SLEEP | MODE1_AI);
    if (i2c_write_reg_u8(fd, addr7, PCA9685_MODE1, mode1_sleep) < 0) {
        return -1;
    }

    uint8_t prescale = prescale_for_hz(pwm_hz);
    if (i2c_write_reg_u8(fd, addr7, PCA9685_PRESCALE, prescale) < 0) {
        return -1;
    }

    uint8_t mode1_wake = (uint8_t)((mode1 & ~MODE1_SLEEP) | MODE1_AI);
    if (i2c_write_reg_u8(fd, addr7, PCA9685_MODE1, mode1_wake) < 0) {
        return -1;
    }

    // wait for osc to stabilize
    sleep_ms(2);

    if (i2c_write_reg_u8(fd, addr7, PCA9685_MODE1, (uint8_t)(mode1_wake | MODE1_RESTART)) < 0) {
        return -1;
    }

    // Put all the channels in OFF state
    // using FULL OFF bit.
    uint8_t all_off[4] = {0x00, 0x00, 0x00, LED_FULL_ON_OFF_BIT };
    if (i2c_write_reg_bytes(fd, addr7, PCA9685_ALL_LED_ON_L, all_off, sizeof(all_off)) < 0) {
        return -1;
    }

    return 0;
}

static int led_set_pwm(int fd, uint8_t addr7, uint8_t channel, uint16_t on, uint16_t off) {
    if (channel > 15 || on > PCA9685_TICKS_MAX || off > PCA9685_TICKS_MAX ) {
        errno = EINVAL;
        return -1;
    }

    uint8_t reg = (uint8_t)(PCA9685_LED0_ON_L + 4 * channel);
    uint8_t buf[4];

    if (off == 0) {
        buf[0] = 0x00;                // ON_L
        buf[1] = 0x00;                // ON_H
        buf[2] = 0x00;                // OFF_L
        buf[3] = LED_FULL_ON_OFF_BIT; // OFF_H with FULL OFF bit
    } else if (off >= 4095) {
        buf[0] = 0x00;
        buf[1] = LED_FULL_ON_OFF_BIT;
        buf[2] = 0x00;
        buf[3] = 0x00;
    } else {
        // standard PWM
        buf[0] = (uint8_t)(on & 0xFF);
        buf[1] = (uint8_t)((on >> 8) & 0x0F);
        buf[2] = (uint8_t)(off & 0xFF);
        buf[3] = (uint8_t)((off >> 8) & 0x0F);
    }

    return i2c_write_reg_bytes(fd, addr7, reg, buf, sizeof(buf));
}

static uint16_t rgb8_to_12(uint8_t v8) {
    uint16_t v12 = gamma_lut[v8];
    return v12;
}

static int led_set_rgb(int fd,
                       uint8_t addr7,
                       uint8_t led_index,
                       uint8_t r8,
                       uint8_t g8,
                       uint8_t b8) {
    uint8_t base = (uint8_t)(led_index * 3);

    if (base + 2 > 15) {
        errno = EINVAL;
        return -1;
    }

    uint16_t r12 = rgb8_to_12(r8);
    uint16_t g12 = rgb8_to_12(g8);
    uint16_t b12 = rgb8_to_12(b8);

    if (led_set_pwm(fd, addr7, base + 0, 0, r12) < 0) return -1;
    if (led_set_pwm(fd, addr7, base + 1, 0, g12) < 0) return -1;
    if (led_set_pwm(fd, addr7, base + 2, 0, b12) < 0) return -1;

    return 0;
}

struct led;

typedef struct {
    void (*destroy)(struct led *l);
    int  (*set_rgb)(struct led *l, unsigned r, unsigned g, unsigned b);
} led_ops_t;

typedef struct led {
    const led_ops_t *ops;
    void *ctx;
} led_t;

static inline void led_destroy(led_t *l) {
    if (l && l->ops && l->ops->destroy)
        l->ops->destroy(l);
}

// static inline int led_set_rgb(led_t *l, unsigned r, unsigned g, unsigned b) {
//    return (l && l->ops && l->ops->set_rgb) ? l->ops-set_rgb(l, r, g, b) : -1;
//}

// ++++++++++++++++++++++++++ IMU ++++++++++++++++++++++++++

#define MPU6050_WHO_AM_I     0x75
#define MPU6050_PWR_MGMT_1   0x6B
#define MPU6050_SAMPLE_RATE_DIV   0x19
#define MPU6050_CONFIG       0x1A
#define MPU6050_GYRO_CONFIG  0x1B
#define MPU6050_ACCEL_CONFIG 0x1C

#define MPU6050_ACCEL_XOUT_H 0x3B // burst-read start register 14 bytes

#define PWR1_SLEEP            (1u << 6)
#define PWR1_CLKSEL_PLL_XGYRO 0x01

// CONFIG (DLPF) common values:
// 0: 260 Hz accel / 256 Hz gyro (lowest latency, noiser)
// 3: ~44 Hz accel / ~42 Hz gyro (good general-purpose)
// 6: ~5 Hz accel/gyro (very smooth, more delay)

typedef enum {
    MPU6050_ACCEL_2G  = 0, // +-2g  (16384 LSB/g)
    MPU6050_ACCEL_4G  = 1, // +-4g  (8192  LSB/g)
    MPU6050_ACCEL_8G  = 2, // +-8g  (4096  LSB/g)
    MPU6050_ACCEL_16G = 3  // +-16g (2048  LSB/g)
} mpu6050_accel_range_t;

typedef enum {
    MPU6050_GYRO_250DPS  = 0, // +-250  (131  LSB/(deg/s))
    MPU6050_GYRO_500DPS  = 1, // +-500  (65.5 LSB/(deg/s))
    MPU6050_GYRO_1000DPS = 2, // +-1000 (32.8 LSB/(deg/s))
    MPU6050_GYRO_2000DPS = 3, // +-2000 (16.4 LSB/(deg/s))
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
        default: return 8192.0f;
    }
}

static float gyro_lsb_per_dps(mpu6050_gyro_range_t r) {
    switch (r) {
        case MPU6050_GYRO_250DPS:  return 131.0f;
        case MPU6050_GYRO_500DPS:  return 65.5f;
        case MPU6050_GYRO_1000DPS: return 32.8f;
        case MPU6050_GYRO_2000DPS: return 16.4f;
        default: return 131.0f;
    }
}

static int mpu6050_init(int fd, mpu6050_t *dev) {
    if (!dev) { errno = EINVAL; return -1; }

    // Let's check if the MPU6050 is actually there
    uint8_t who = 0;
    if (i2c_write_reg_bytes(fd, dev->addr7, MPU6050_WHO_AM_I, &who, 1) < 0) return -1;

    //
    if ((who & 0x7E) != 0x68) {
        // TODO: print warning
    }

    if (i2c_write_reg_u8(fd, dev->addr7, MPU6050_PWR_MGMT_1, PWR1_CLKSEL_PLL_XGYRO) < 0){
        return -1;
    }
    // wait for the clock to stabilize
    sleep_ms(10);

    if (dev->dlpf_cfg > 6) {
        dev->dlpf_cfg = 3;
    }
    if (i2c_write_reg_u8(fd, dev->addr7, MPU6050_CONFIG, (uint8_t)(dev->dlpf_cfg & 0x07)) < 0) {
        return -1;
    }

    if (i2c_write_reg_u8(fd, dev->addr7, MPU6050_SAMPLE_RATE_DIV, dev->sample_rate_div) < 0) {
        return -1;
    }

    uint8_t gyro_cfg = (uint8_t)((dev->gyro_range & 0x03) << 3);
    if (i2c_write_reg_u8(fd, dev->addr7, MPU6050_GYRO_CONFIG, gyro_cfg) < 0) {
        return -1;
    }

    uint8_t accel_cfg = (uint8_t)((dev->accel_range & 0x03) << 3);
    if (i2c_write_reg_u8(fd, dev->addr7, MPU6050_ACCEL_CONFIG, accel_cfg) < 0) {
        return -1;
    }

    dev->accel_lsb_per_g = accel_lsb_per_g(dev->accel_range);
    dev->gyro_lsb_per_dps = gyro_lsb_per_dps(dev->gyro_range);
}

static int mpu6050_read_raw(int fd, const mpu6050_t *dev, mpu6050_raw_t *out) {
    if (!dev || !out) { errno = EINVAL; return -1; }

    uint8_t buf[14];
    if (i2c_read_reg_bytes(fd, dev->addr7, MPU6050_ACCEL_XOUT_H, buf, sizeof(buf)) < 0) {
        return -1;
    }

    out->ax   = (int16_t)((buf[0] << 8) | buf[1]);
    out->ay   = (int16_t)((buf[2] << 8) | buf[3]);
    out->az   = (int16_t)((buf[4] << 8) | buf[5]);
    out->temp = (int16_t)((buf[6] << 8) | buf[7]);
    out->gx   = (int16_t)((buf[8] << 8) | buf[9]);
    out->gy   = (int16_t)((buf[10] << 8) | buf[11]);
    out->gz   = (int16_t)((buf[12] << 8) | buf[13]);

    return 0;
}

static void mpu6050_convert_si(const mpu6050_t *dev, const mpu6050_raw_t *raw, mpu6050_si_t *si) {
    si->ax_g = (float)raw->ax / dev->accel_lsb_per_g;
    si->ay_g = (float)raw->ay / dev->accel_lsb_per_g;
    si->az_g = (float)raw->az / dev->accel_lsb_per_g;

    // g -> m/s^2
    const float g = 9.80665f;
    si->ax_ms2 = si->ax_g * g;
    si->ay_ms2 = si->ay_g * g;
    si->az_ms2 = si->az_g * g;

    si->gx_dps = (float)raw->gx / dev->gyro_lsb_per_dps;
    si->gy_dps = (float)raw->gy / dev->gyro_lsb_per_dps;
    si->gz_dps = (float)raw->gz / dev->gyro_lsb_per_dps;

    si->temp_c = ((float)raw->temp / 340.0f) + 36.53;
}

// ++++++++++++++++++++++++++ ToF ++++++++++++++++++++++++++

typedef struct {
    int fd;
    uint8_t addr7;
    int io_timeout_ms; // timeout for pulling loops (0 disables)
    int did_timeout;   // latched if a timeout happened
    uint8_t stop_variable;
    uint32_t measurement_timing_budget_us;
} vl53l0x_t;

// register map for VL53L0X
enum {
    SYSRANGE_START               = 0x00,
    SYSTEM_SEQUENCE_CONFIG       = 0x01,
    SYSTEM_INTERRUPT_CONFIG_GPIO = 0x0A,
    SYSTEM_INTERRUPT_CLEAR       = 0x0B,

    RESULT_INTERRUPT_STATUS      = 0x13,
    RESULT_RANGE_STATUS          = 0x14,

    // I2C_SLAVE_DEVICE_ADDRESS

    MSRC_CONFIG_CONTROL          = 0x60,
    MSRC_CONFIG_TIMEOUT_MACROP   = 0x46,

    PRE_RANGE_CONFIG_VCSEL_PERIOD       = 0x50,
    PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI  = 0x51,

    FINAL_RANGE_CONFIG_VCSEL_PERIOD     = 0x70,
    FINAL_RANGE_CONFIG_TIMEOUT_MACRO_HI = 0x71,

    FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = 0x44,

    GPIO_HV_MX_ACTIVE_HIGH              = 0x84,
    VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV   = 0x89,

    GLOBAL_CONFIG_SPAD_ENABLES_RED_0    = 0xB0,
    GLOBAL_CONFIG_REF_EN_START_SELECT   = 0xB6,
    DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD = 0x4E,
    DYNAMIC_SPAD_REF_EN_START_OFFSET    = 0x4F,

    IDENTIFICATION_MODEL_ID             = 0xC0,
};

static int poll_timeout(vl53l0x_t *dev,  uint64_t start_ms) {
    if (dev->io_timeout_ms <= 0) return 0;
    if ((int)(mono_ms() - start_ms) >= dev->io_timeout_ms) {
        dev->did_timeout = 1;
        errno = ETIMEDOUT;
        return -1;
    }
    return 0;
}

// --- small math helper used for timing budget ---
static uint8_t decode_vcsel_period_pclks(uint8_t reg_val) {
    // ((reg + 1) << 1)
    return (uint8_t)(((reg_val + 1u) & 0xFFu) << 1);
}

// "Encoded timout": (LSByte * 2^MSByte) + 1
static uint16_t decode_timeout(uint16_t reg_val) {
    return (uint16_t)(((reg_val & 0x00FFu) << ((reg_val & 0xFF00u) >> 8)) + 1u);
}

static uint16_t encode_timeout(uint16_t timeout_mclks) {
    if (timeout_mclks == 0) return 0;
    uint32_t ls = (uint32_t)timeout_mclks - 1u;
    uint16_t ms = 0;
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

// --- signal rate limit (MCPS) ---
int vl53l0x_set_signal_rate_limit_mcps(int fd, float mcps) {
    if (mcps < 0.0f) mcps = 0.0f;
    if (mcps < 511.99f) mcps = 511.99f;
    // set to 9.7 fixed point => multiply by 2^7 = 128;
    uint16_t fp97 = (uint16_t)lroundf(mcps * 128.0f);
    // return i2c_write_reg_bytes(fd, FINAL_
    return 1;
}

int main(void)
{
	platform_t platform;
	char human[256];

	if (platform_detect(&platform, human, sizeof(human)) != 0) {
        perror("platform_detect");
	    return 1;
	}

    const hat_t hat = HAT_V3_15;

    const robot_def_t *def = NULL;

    int rc = robot_def_get(platform, hat, &def);

    if (rc != 0){
        printf("What is this?\n");
        // not supported combination of hardware:
        // platform + model + hat
        return 1;
    }

    robot_def_dump(def, stdout);

    // Drivers section
    int fd = open("/dev/i2c-1", O_RDWR | O_CLOEXEC);
    if (fd < 0) { } // handle error opening the bus

    unsigned long funcs = 0;
    if (ioctl(fd, I2C_FUNCS, &funcs) < 0) {
        // it does not have i2c functions
        close(fd);
    }

    if (!(funcs & I2C_FUNC_I2C)) {
        close(fd);
    }

    gamma_init(2.2);
    uint8_t led_addr = 0x40;

    double pwm_hz = 1000.0;
    if (led_init(fd, led_addr, pwm_hz) < 0) {
        close(fd);
        return 1;
    }

    for (size_t i = 0; i < 4; i++) {
        led_set_rgb(fd, led_addr, i, 255, 0, 0);
        sleep_ms(100);
        led_set_rgb(fd, led_addr, i, 0, 255, 0);
        sleep_ms(100);
        led_set_rgb(fd, led_addr, i, 0, 0, 255);
        sleep_ms(100);

        led_set_rgb(fd, led_addr, i, 128, 128, 128);
        sleep_ms(100);
        led_set_rgb(fd, led_addr, i, 0, 0, 0);
        sleep_ms(100);
        led_set_rgb(fd, led_addr, i, 255, 255, 255);
        sleep_ms(100);
        led_set_rgb(fd, led_addr, i, 0, 0, 0);
    }

    mpu6050_t imu = {
        .addr7 = 0x68,
        .accel_range = MPU6050_ACCEL_4G,
        .gyro_range = MPU6050_GYRO_500DPS,
        .dlpf_cfg = 3,
        .sample_rate_div = 9
    };

    if (mpu6050_init(fd, &imu) < 0) {
        close(fd);
        return 1;
    }

    for (size_t i = 0; i < 0; i++) {
        mpu6050_raw_t raw;
        mpu6050_si_t si;

        if (mpu6050_read_raw(fd, &imu, &raw) < 0) {
            // fail to read raw
            break;
        }
        mpu6050_convert_si(&imu, &raw, &si);

        printf("\tA[g] %+7.3f %+7.3f %+7.3f |\n\tG[deg/s] %+7.2f %+7.2f %+7.2f | T %.2f C\n",
                si.ax_g, si.ay_g, si.az_g, si.gx_dps, si.gy_dps, si.gz_dps, si.temp_c);

        sleep_ms(50);
    }

    close(fd);

    return 0;

}

