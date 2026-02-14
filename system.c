/*
 * platform_detect.c - Detect platform family + model from a running Linux system
 *
 * Uses:
 *   - Device Tree (preferred on embedded): /proc/device-tree/model, /proc/device-tree/compatible
 *     Note: userspace should follow /proc/device-tree (stable ABI). :contentReference[oaicite:4]{index=4}
 *   - DMI (common on x86): /sys/class/dmi/id/* (symlink to /sys/devices/virtual/dmi/id). :contentReference[oaicite:5]{index=5}
 *
 * Build:
 *   gcc -std=c11 -Wall -Wextra -O2 platform_detect.c -o platform_detect
 */

#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

/**
 * enum platform_family - High-level platform family.
 * @PLATFORM_FAMILY_UNKNOWN: Unknown or not detected.
 * @PLATFORM_FAMILY_JETSON: NVIDIA Jetson family.
 * @PLATFORM_FAMILY_RASPBERRY: Raspberry Pi family.
 * @PLATFORM_FAMILY_X86: x86/amd64 PCs/servers.
 * @PLATFORM_FAMILY_OTHER: Other/uncategorized.
 */
typedef enum {
	PLATFORM_FAMILY_UNKNOWN   = 0,
	PLATFORM_FAMILY_JETSON    = 1,
	PLATFORM_FAMILY_RASPBERRY = 2,
	PLATFORM_FAMILY_X86       = 3,
	PLATFORM_FAMILY_OTHER     = 255
} platform_family_t;

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

typedef enum {
	HAT_UNKNOWN   = 0,
    HAT_V3_15     = 1,
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

	/* Also try hints from compatible (common on Jetson). :contentReference[oaicite:6]{index=6} */
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

/* Note: DMI strings are normal text files, not DT; we can reuse read_dt_string safely. */
static int read_dmi_strings(char *vendor, size_t vendorsz, char *product, size_t productsz)
{
	int ok1, ok2;

	ok1 = read_sysfs_string_first(vendor, vendorsz,
	                              "/sys/class/dmi/id/sys_vendor",
	                              "/sys/devices/virtual/dmi/id/sys_vendor");
	ok2 = read_sysfs_string_first(product, productsz,
	                              "/sys/class/dmi/id/product_name",
	                              "/sys/devices/virtual/dmi/id/product_name");
	return (ok1 == 0 && ok2 == 0) ? 0 : -1;
}

/* ---------- Public API ---------- */

/**
 * platform_detect - Detect platform family + model on Linux.
 * @out: output platform struct
 * @human: optional output buffer for a human-readable model string (may be NULL)
 * @humansz: size of @human
 *
 * Detection order:
 *   1) Device Tree (/proc/device-tree/...) for embedded boards. :contentReference[oaicite:7]{index=7}
 *   2) DMI sysfs for x86. :contentReference[oaicite:8]{index=8}
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

	out->family = PLATFORM_FAMILY_UNKNOWN;
	out->model  = 0;
	if (human && humansz) human[0] = '\0';

	/* 1) Device Tree: model string is usually enough for Jetson/RPi. :contentReference[oaicite:9]{index=9} */
	if (read_dt_string("/proc/device-tree/model", model, sizeof(model)) == 0) {

		if (contains_ci(model, "NVIDIA Jetson") || dt_compatible_contains("nvidia,jetson")) {
			out->family = PLATFORM_FAMILY_JETSON;
			out->model  = (uint16_t)parse_jetson_model(model);
			if (human && humansz) {
				strncpy(human, model, humansz - 1);
				human[humansz - 1] = '\0';
			}
			return 0;
		}

		if (contains_ci(model, "Raspberry Pi")) {
			out->family = PLATFORM_FAMILY_RASPBERRY;
			out->model  = (uint16_t)parse_rpi_model(model);
			if (human && humansz) {
				strncpy(human, model, humansz - 1);
				human[humansz - 1] = '\0';
			}
			return 0;
		}

		/* Some other DT-based board */
		out->family = PLATFORM_FAMILY_OTHER;
		out->model  = 0;
		if (human && humansz) {
			strncpy(human, model, humansz - 1);
			human[humansz - 1] = '\0';
		}
		return 0;
	}

	/* 2) x86 fallback via DMI */
	{
		char vendor[128] = {0};
		char product[128] = {0};

		if (read_dmi_strings(vendor, sizeof(vendor), product, sizeof(product)) == 0) {
			out->family = PLATFORM_FAMILY_X86;
			out->model  = 0; /* define x86 models later if you want */
			if (human && humansz) {
				/* "Vendor Product" */
				snprintf(human, humansz, "%s %s", vendor, product);
				rstrip(human);
			}
			return 0;
		}
	}

	/* Could not detect anything useful */
	return -1;
}

static const char *family_to_string(platform_family_t f)
{
	switch (f) {
	case PLATFORM_FAMILY_JETSON:    return "jetson";
	case PLATFORM_FAMILY_RASPBERRY: return "raspberry";
	case PLATFORM_FAMILY_X86:       return "x86";
	case PLATFORM_FAMILY_OTHER:     return "other";
	default:                        return "unknown";
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

/**
 * struct single peripheral description.
 *
 * The idea: this is *metadata* the rest of your
 * stack can use to bind to the right driver
 * and OS resources.
 */
typedef struct {
    peripheral_type_t type;
    const char *name;

    peripheral_iface_t iface;
    const char *bus;
    int addr;

    const char *path;
    const char *driver;
} peripheral_desc_t;

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
    { PERIPH_DISPLAY, "display0", IFACE_I2C, "i2c-1", 0x3C, "/dev/i2c-1", "ssd1306" }
};

#define ROBOT_ENTRY(FAMILY, MODEL, HAT, ID, NAME, PERIPHS) \
    { \
        .key = ROBOT_DEF_KEY((FAMILY), (uint16_t)(MODEL), (HAT)), \
        .id = (ID), \
        .display_name = (NAME), \
        .peripherals = (PERIPHS), \
        .num_peripherals = sizeof(PERIPHS)/sizeof((PERIPHS)[0]), \
    }

static const robot_def_t robot_table[] = {
    ROBOT_ENTRY(
        PLATFORM_FAMILY_JETSON,
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
        return -1; // -EINVAL;

    *out = NULL;

    if (platform.family == PLATFORM_FAMILY_UNKNOWN || platform.model == 0 || hat == HAT_UNKNOWN)
        return -1;

    key = robot_def_key(platform.family, platform.model, hat);

    for (size_t i = 0; i < sizeof(robot_table)/sizeof(robot_table[0]); i++) {
        if (robot_table[i].key == key) {
            printf("found a match\n");
            *out = &robot_table[i];
            return 0;
        }
    }

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

    fprintf(fp, "  peripherals: %zu\n", def->num_peripherals);

    for (size_t i = 0; i < def->num_peripherals; i++) {
        const peripheral_desc_t *p = &def->peripherals[i];

        fprintf(fp,
                "    - [%zu] type=%d name=%s iface=%d bus=%s addr=%d path=%s driver=%s\n",
                i,
                p->type, // TODO: convert to string
                p->name ? p->name : "(null)",
                p->iface, // TODO: convert to string
                p->bus ? p->bus : "(null)",
                p->addr,
                p->path ? p->path : "(null)",
                p->driver ? p->driver : "(null)");

    }
}

int main(void)
{
	platform_t platform;
	char human[256];

	if (platform_detect(&platform, human, sizeof(human)) != 0) {
        perror("platform_detect");
	    return 1;
	}
    printf("family=%s (%u), model_code=%u, model_str=\"%s\"\n",
           family_to_string(platform.family),
           (unsigned)platform.family,
           (unsigned)platform.model,
           human);

    const hat_t hat = HAT_V3_15;

    const robot_def_t *def = NULL;

    int rc = robot_def_get(platform, hat, &def);

    if (rc == 0){
        printf("Nice robot\n");

    } else {
        printf("What is this?\n");
        // not supported combination of hardware
        return 1;
    }

    robot_def_dump(def, stdout);
    
    return 0;

}

