#include <stdio.h>

#define FOXY_BATTERY_IMPLEMENTATION
#include "battery.h"

int main(void) {
    foxy_battery_status s;

    int rc = foxy_battery_get(&s); /* reads /run/batteryd/status */
    if (rc != FOXY_BATTERY_OK) {
        fprintf(stderr, "foxy-battery: %s\n", foxy_battery_strerror(rc));
        return 1;
    }

    if (s.online) {
        printf("battery: %d%%, %dmV, %dmA\n",
               s.percent, s.voltage_mv, s.current_ma);
    } else {
        printf("battery offline");
        if (s.error[0]) printf(": %s", s.error);
        printf("\n");
    }

    // foxy_battery_shutdown(NULL);        /* sends SHUTDOWN */

    return 0;
}
