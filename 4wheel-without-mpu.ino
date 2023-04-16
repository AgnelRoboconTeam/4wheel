#include <XBOXUSB.h>
#include "CytronMotorDriver.h"

#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>

#include <Wire.h>

#define PI_BY_2 1.57079633

USB Usb;
XBOXUSB Xbox(&Usb);

// XBox input sensitivity.
#define INPUT_SENSITIVITY 15

// To enable / disable bot.
bool enabled = false;

// Set maximium PWM. This is tweaked by the joystick controller.
short max_pwm = 250;
bool limit_rotation_pwm = false;

// Configure the motor driver.
CytronMD fr_motor(PWM_DIR, 6, 23); // PWM, DIR
CytronMD fl_motor(PWM_DIR, 4, 24); // PWM, DIR
CytronMD br_motor(PWM_DIR, 7, 25); // PWM, DIR
CytronMD bl_motor(PWM_DIR, 5, 22); // PWM, DIR

// double_map works like map() but accepts `double' type.
double double_map(double x, double in_min, double in_max, double out_min, double out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// drive_wheels takes values for all 4 wheels and drives them. The
// values are scaled to [-255,255] before passing to the motor driver.
void drive_wheels(short fl, short fr, short bl, short br) {
    // Get the maximum, minimum value.
    short max_sp = max(max(max(fl, fr), max(bl, br)), max_pwm);
    short min_sp = min(min(min(fl, fr), min(bl, br)), -max_pwm);

    // Scale all the motor speeds.
    fl = map(fl, min_sp, max_sp, -max_pwm, max_pwm);
    fr = map(fr, min_sp, max_sp, -max_pwm, max_pwm);
    br = map(br, min_sp, max_sp, -max_pwm, max_pwm);
    bl = map(bl, min_sp, max_sp, -max_pwm, max_pwm);

    Serial.print(F("FL: "));
    Serial.print(fl);
    Serial.print(F("\tFR: "));
    Serial.print(fr);
    Serial.print(F("\tBL: "));
    Serial.print(bl);
    Serial.print(F("\tBR: "));
    Serial.print(br);
    Serial.print("\t");

    // Drive the motors.
    fl_motor.setSpeed(fl);
    br_motor.setSpeed(br);
    bl_motor.setSpeed(bl);
    fr_motor.setSpeed(fr);
}

void setup() {
    Serial.begin(115200);
#if !defined(_MIPSEL_)
    // Wait for serial port to connect - used on Leonardo, Teensy and
    // other boards with built-in USB CDC serial connection
    while (!Serial);
#endif

    if (Usb.Init() == -1) {
        Serial.print(F("OSC did not start.\n"));
        while (1); // Halt.
    }
    Serial.print(F("Xbox Wireless Receiver Library Started.\n"));
}

void loop() {
    Usb.Task();

    // Return if Xbox is not connected.
    if (!Xbox.Xbox360Connected) {
        drive_wheels(0, 0, 0, 0); // Stop motors.
        Serial.println();
        return;
    }

    // start, stop bot.
    if (Xbox.getButtonClick(BACK)) {
        enabled = false;
        drive_wheels(0, 0, 0 ,0);
        Serial.println();
    }
    if (Xbox.getButtonClick(START))
        enabled = true;

    if (!enabled) return;

    // PWM decrement, increments - by 50.
    if (Xbox.getButtonClick(LB))
        // Don't decrement after 50 PWM.
        max_pwm = max_pwm <= 50 ? 50 : max_pwm - 50;

    if (Xbox.getButtonClick(RB))
        // Don't increment after 250 PWM.
        max_pwm = max_pwm >= 250 ? 50 : max_pwm + 50;

    // Toogle max PWM for rotation.
    if (Xbox.getButtonClick(A))
        limit_rotation_pwm = true;

    // Initialize fl, fr, bl, br.
    short fl, fr, bl, br;
    fl = fr = bl = br = 0;

    // Get rt, lt values.
    short rotation_max_pwm = limit_rotation_pwm == true ? 100 : max_pwm;
    short rt = map(Xbox.getButtonPress(RT), -255, 255, -rotation_max_pwm, rotation_max_pwm);
    short lt = map(Xbox.getButtonPress(LT), -255, 255, -rotation_max_pwm, rotation_max_pwm);

    // Rotate clockwise.
    if (rt > INPUT_SENSITIVITY) {
        fl += rt;
        bl += rt;
        fr -= rt;
        br -= rt;
    }
    // Rotate anti-clockwise.
    if (lt > INPUT_SENSITIVITY) {
        fl -= lt;
        bl -= lt;
        fr += lt;
        br += lt;
    }

    // Max values of Vx, Vy = 32768 (2^15).
    short vx = map(Xbox.getAnalogHat(LeftHatX), -32767, 32767, -max_pwm, max_pwm); // X component.
    short vy = map(Xbox.getAnalogHat(LeftHatY), -32767, 32767, -max_pwm, max_pwm); // Y component.

    if (abs(vx) > INPUT_SENSITIVITY || abs(vy) > INPUT_SENSITIVITY) {

        if (abs(vx) < INPUT_SENSITIVITY) vx = 0;
        if (abs(vy) < INPUT_SENSITIVITY) vy = 0;

        /*
         * We are mapping the joystick's plane to the robot. The equations
         * are of the form `fl = vx + vy'. Now this means that when we
         * push the joystick to max at 45 degree angle it sets `fl' to
         * `255 * 2' because `vx' & `vy' will be max.
         *
         * A naive solution would be to simply set all `> 255' values to
         * `255' but that means the joystick's plane is not mapped to the
         * robot.
         *
         * We need to have a weight attached to `vx' & `vy' such that at
         * 45 degress both have 0.5 as weight and so on..
         *
         * The equation becomes something like `fl = (vx * a) + (vy * b)'.
         * The weight must be something dependent upon both variables. We
         * reach this conclusion by assuming the weight is something that
         * is only dependent upon the other variable (i.e. `a' depends on
         * `vy' only). Now, if we push the joystick halfway through 45
         * degree, `a' should be 0.5 & it should be 0.5 even if it's
         * pushed to max. `vy' is different in both cases, only the angle
         * remains the same. Thus the weights must depend on the angle.
         *
         * We get the angle by taking arctan(vy / vx). Here we take
         * absolute values because we only need output in range [0,
         * PI_BY_2]. If we first take arctan and then take absolute value
         * of the result then the range is [0, PI].
         *
         * angle is in range [0, PI_BY_2] and we need weights in range [0,
         * 1] so angle is mapped with double_map.
         */
        double angle = atan2(abs(vy), abs(vx));
        double angle_mapped = double_map(angle, 0, PI_BY_2, 0, 1);

        // `vx' weight will be (1.0 - weight of `vy').
        vy = vy * angle_mapped;
        vx = vx * (1.0 - angle_mapped);

        Serial.print(F("Angle Mapped: "));
        Serial.print(angle_mapped);
        Serial.print(F("\t"));

        // Vector equations.
        //
        // We shouldn't need constrain function over here, if everything
        // has been done right above then these values must be in range
        // [-255, 255].
        fl += constrain(vy + vx, -max_pwm, max_pwm);
        fr += constrain(vy - vx, -max_pwm, max_pwm);
        bl += constrain(vy - vx, -max_pwm, max_pwm);
        br += constrain(vy + vx, -max_pwm, max_pwm);
    }

    Serial.print(F("Max PWM: "));
    Serial.print(max_pwm);
    Serial.print("\t");

    Serial.print(F("LeftHatX: "));
    Serial.print(vx);
    Serial.print("\t");

    Serial.print(F("LeftHatY: "));
    Serial.print(vy);
    Serial.print("\t");

    drive_wheels(fl, fr, bl ,br);
    Serial.println();
}
