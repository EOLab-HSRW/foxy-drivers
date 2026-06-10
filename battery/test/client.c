#include <stdio.h>

#define BATTERYD_IMPLEMENTATION
#include "battery.h"

int main(void) {
    batteryd_status s;

    int rc = batteryd_get(&s); /* reads /run/batteryd/status */
    if (rc != BATTERYD_OK) {
        fprintf(stderr, "batteryd: %s\n", batteryd_strerror(rc));
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

    // batteryd_shutdown(NULL);        /* sends SHUTDOWN */

    return 0;
}
