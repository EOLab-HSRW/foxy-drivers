#define FOXY_IMPLEMENTATION
#include "foxy.h"

int main(void) {

    robot_t robot = robot_init();

    if (!robot_ok(&robot)) {
        // TODO: add message
        // fail to init robot
        return 1;
    }

    robot_def_dump(robot.def, stdout);

    led_t leds = led_init_name(&robot, "leds_front_and_rear");
    if (!leds.ctx) {
        // TODO: add message
        // fail to init led driver
        robot_deinit(&robot);
        return 1;
    }

    for (uint8_t i = 0; i < 4; i++) {
        led_set_rgb(leds, i, 255, 0, 0); sleep_ms(100);
        led_set_rgb(leds, i, 0, 255, 0); sleep_ms(100);
        led_set_rgb(leds, i, 0, 0, 255); sleep_ms(100);
        led_set_rgb(leds, i, 0, 0, 0); sleep_ms(100);
        led_set_rgb(leds, i, 255, 255, 255); sleep_ms(100);
        led_set_rgb(leds, i, 0, 0, 0); sleep_ms(100);
    }
    led_deinit(&robot, &leds);

    imu_t imu = imu_init_name(&robot, "imu0");

    if (!imu.ctx) {
        robot_deinit(&robot);
        return 1;
    }

    for (size_t i = 0; i < 50; i++) {
        imu_sample_t s = imu_read(imu);
        printf("\tA[ms^2]  %+7.3f %+7.3f %+7.3f |\n"
               "\tG[deg/s] %+7.2f %+7.2f %+7.2f |\n"
               "\tM[uT] %+7.2f %+7.2f %+7.2f |\n"
               "T %.2f C\n",
                s.accel_ms2[0], s.accel_ms2[1], s.accel_ms2[2],
                s.gyro_dps[0], s.gyro_dps[1], s.gyro_dps[2],
                s.mag_uT[0], s.mag_uT[1], s.mag_uT[2],
                s.temp_c);
        sleep_ms(50);
    }

    gpio_t button = gpio_init_name(&robot, "top_button");
    gpio_set_as_input(button);
    for (size_t i = 0; i < 10; i++) {
        int v = gpio_read(button);
        printf("Button state: %d\n", v);
        sleep_ms(500);
    }

    gpio_t button_led = gpio_init_name(&robot, "top_button_led");
    gpio_set_as_output(button_led);
    for (size_t i = 0; i < 10; i++) {
        gpio_write(button_led, 1);
        sleep_ms(100);
        gpio_write(button_led, 0);
        sleep_ms(100);
    }

    gpio_t hat_led = gpio_init_name(&robot, "hat_builtin_led");
    gpio_set_as_output(hat_led);
    for (size_t i = 0; i < 10; i++) {
        gpio_write(hat_led, 1);
        sleep_ms(100);
        gpio_write(hat_led, 0);
        sleep_ms(100);
    }

    motor_t m1 = motor_init_name(&robot, "motor1");
    motor_t m2 = motor_init_name(&robot, "motor2");
    motor_set(m1, -0.40f);
    motor_set(m2, -0.40f);
    sleep_ms(500);
    motor_brake(m1);
    motor_brake(m2);

    return 0;
}
