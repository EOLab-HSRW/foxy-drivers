#define FOXY_IMPLEMENTATION
#include "foxy/foxy.h"

int main(void) {

    robot_t robot = robot_init();

    if (!robot_ok(&robot)) {
        printf("Fail to init robot description\n");
        return 1;
    }

    robot_def_dump(robot.def, stdout);

    led_t leds = led_init_name(&robot, "leds_front_and_rear");
    if (!leds.ctx) {
        printf("Fail to init led driver\n");
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
        printf("Fail to init imu driver\n");
        robot_deinit(&robot);
        return 1;

    }

    for (size_t i = 0; i < 50; i++) {
        imu_sample_t s = imu_read(imu);
        printf("\tA[ms^2]  %+7.3f %+7.3f %+7.3f |\n"
               "\tG[rad/s] %+7.2f %+7.2f %+7.2f |\n"
               "\tM[uT] %+7.2f %+7.2f %+7.2f |\n"
               "T %.2f C\n",
                s.accel_ms2[0], s.accel_ms2[1], s.accel_ms2[2],
                s.gyro_rads[0], s.gyro_rads[1], s.gyro_rads[2],
                s.mag_uT[0], s.mag_uT[1], s.mag_uT[2],
                s.temp_c);
        sleep_ms(50);
    }
    imu_deinit(&robot, &imu);

    tof_t tof = tof_init_name(&robot, "tof0");
    if (!tof.ctx) {
        printf("Fail to init VL53L0X driver\n");
        robot_deinit(&robot);
        return 1;
    }
    for (size_t i = 0; i < 50; i++) {
        int distance_mm = tof_read_mm(tof);
        if (distance_mm < 0) {
            printf("Fail to read VL53L0X distance: %d\n", distance_mm);
            break;
        }
        printf("Distance: %d mm\n", distance_mm);
        sleep_ms(50);
    }
    tof_deinit(&robot, &tof);

    gpio_t button = gpio_init_name(&robot, "top_button");

    if (!button.ctx) {
        printf("Fail to init button driver\n");
        robot_deinit(&robot);
        return 1;
    }
    gpio_set_as_input(button);
    for (size_t i = 0; i < 10; i++) {
        int v = gpio_read(button);
        printf("Button state: %d\n", v);
        sleep_ms(500);
    }
    gpio_deinit(&robot, &button);

    gpio_t button_led = gpio_init_name(&robot, "top_button_led");
    if (!button_led.ctx) {
        printf("Fail to init button led driver\n");
        robot_deinit(&robot);
        return 1;
    }
    gpio_set_as_output(button_led);
    for (size_t i = 0; i < 10; i++) {
        gpio_write(button_led, 1);
        sleep_ms(100);
        gpio_write(button_led, 0);
        sleep_ms(100);

    }
    gpio_deinit(&robot, &button_led);

    gpio_t hat_led = gpio_init_name(&robot, "hat_builtin_led");
    if (!hat_led.ctx) {
        printf("Fail to init HAT led driver\n");
        robot_deinit(&robot);
        return 1;
    }
    gpio_set_as_output(hat_led);
    for (size_t i = 0; i < 10; i++) {
        gpio_write(hat_led, 1);
        sleep_ms(100);
        gpio_write(hat_led, 0);
        sleep_ms(100);
    }
    gpio_deinit(&robot, &hat_led);

    motor_t m1 = motor_init_name(&robot, "motor1");
    motor_t m2 = motor_init_name(&robot, "motor2");

    if (!m1.ctx || !m2.ctx) {
        printf("Fail to init motor driver\n");
        if (m1.ctx) motor_deinit(&robot, &m1);
        if (m2.ctx) motor_deinit(&robot, &m2);
        robot_deinit(&robot);
        return 1;
    }

    motor_set(m1, -0.40f);
    motor_set(m2, -0.40f);
    sleep_ms(500);
    motor_brake(m1);
    motor_brake(m2);
    motor_deinit(&robot, &m1);
    motor_deinit(&robot, &m2);
    robot_deinit(&robot);

    return 0;
}
