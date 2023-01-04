#include <XBOXUSB.h>
#include "CytronMotorDriver.h"

#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>

#include <Wire.h>
#include <MPU6050.h>

#define PI_BY_2 1.57079633

MPU6050 mpu;
USB Usb;
XBOXUSB Xbox(&Usb);

// XBox input sensitivity.
#define INPUT_SENSITIVITY 15

// Maximum yaw deviation tolerated.
#define YAW_THRESHOLD 2

// Stores the current yaw, the code makes sure that this stays the
// same.
float current_yaw = 0;

// To enable / disable bot.
bool enabled = false;

// Configure the motor driver.
CytronMD fr_motor(PWM_DIR, 5, 4); // PWM, DIR
CytronMD fl_motor(PWM_DIR, 6, 7); // PWM, DIR
CytronMD br_motor(PWM_DIR, 3, 2); // PWM, DIR
CytronMD bl_motor(PWM_DIR, 8, 22); // PWM, DIR

// double_map works like map() but accepts `double' type.
double double_map(double x, double in_min, double in_max, double out_min, double out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// drive_wheels takes values for all 4 wheels and drives them. The
// values are scaled to [-255,255] before passing to the motor driver.
void drive_wheels(short fl, short fr, short bl, short br) {
    // Get the maximum, minimum value.
    short max_sp = max(max(max(fl, fr), max(bl, br)), 255);
    short min_sp = min(min(min(fl, fr), min(bl, br)), -255);

    // Scale all the motor speeds.
    fl = map(fl, min_sp, max_sp, -255, 255);
    fr = map(fr, min_sp, max_sp, -255, 255);
    br = map(br, min_sp, max_sp, -255, 255);
    bl = map(bl, min_sp, max_sp, -255, 255);

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
    // Initialize MPU6050
    while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
        {
            Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
            delay(500);
        }

    // Calibrate gyroscope. The calibration must be at rest.
    // If you don't want calibrate, comment this line.
    mpu.calibrateGyro();

    // Set threshold sensivty. Default 3.
    // If you don't want use threshold, comment this line or set 0.
    mpu.setThreshold(3);
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
        return;
    }

    // start, stop bot.
    if (Xbox.getButtonClick(BACK)) {
        enabled = false;
        drive_wheels(0, 0, 0 ,0);
    }
    if (Xbox.getButtonClick(START))
        enabled = true;

    if (!enabled) return;

    // Initialize fl, fr, bl, br.
    short fl, fr, bl, br;
    fl = fr = bl = br = 0;

    // Get rt, lt values.
    short rt = Xbox.getButtonPress(RT);
    short lt = Xbox.getButtonPress(LT);

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
    short vx = map(Xbox.getAnalogHat(LeftHatX), -32767, 32767, -255, 255); // X component.
    short vy = map(Xbox.getAnalogHat(LeftHatY), -32767, 32767, -255, 255); // Y component.

    if (abs(vx) > INPUT_SENSITIVITY || abs(vy) > INPUT_SENSITIVITY) {
        Serial.print(F("LeftHatX: "));
        Serial.print(vx);
        Serial.print("\t");

        Serial.print(F("LeftHatY: "));
        Serial.print(vy);
        Serial.print("\t");

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
        fl += constrain(vy + vx, -255, 255);
        fr += constrain(vy - vx, -255, 255);
        bl += constrain(vy - vx, -255, 255);
        br += constrain(vy + vx, -255, 255);
    }

    // Read normalized values.
    Vector norm = mpu.readNormalizeGyro();

    float timeStep = 0.01;
    // Pitch, Roll and Yaw values
    float pitch = 0;
    float roll = 0;
    float yaw = 0;

    // Calculate Pitch, Roll and Yaw.
    pitch = pitch + norm.YAxis * timeStep;
    roll = roll + norm.XAxis * timeStep;
    yaw = yaw + norm.ZAxis * timeStep;

    // Set current_yaw as the bot is being rotated manually.
    if (rt > INPUT_SENSITIVITY || lt > INPUT_SENSITIVITY)
        current_yaw = yaw;

    // If the bot deviates too much from current_yaw then we need to add
    // some error correction.
    if (YAW_THRESHOLD < abs(current_yaw - yaw)) {
        short diff = constrain(abs(current_yaw - yaw), 20, 255);

        if ((current_yaw - yaw) > 0) {
            // clockwise.
            fl += diff;
            bl += diff;
            fr -= diff;
            br -= diff;
        } else {
            // anti-clockwise.
            fl -= diff;
            bl -= diff;
            fr += diff;
            br += diff;
        }
    }

    Serial.print("\t");
    Serial.print(F("FL: "));
    Serial.print(fl);
    Serial.print(F("\tFR: "));
    Serial.print(fr);
    Serial.print(F("\tBL: "));
    Serial.print(bl);
    Serial.print(F("\tBR: "));
    Serial.print(br);
    Serial.print("\t");

    // Output raw
    Serial.print(" Pitch = ");
    Serial.print(pitch);
    Serial.print(" Roll = ");
    Serial.print(roll);
    Serial.print(" Yaw = ");
    Serial.print(yaw);
    Serial.print(" CYaw = ");
    Serial.print(current_yaw);
    Serial.println();

    drive_wheels(fl, fr, bl ,br);
}
