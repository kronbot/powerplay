package org.firstinspires.ftc.teamcode.lib;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Utils {
    // very low constant
    // used for floating point comparations
    public static double EPS = 0.1;

    public static double MOTOR_ACCELERATION_POWER = 0.3;
    public static double MOTOR_POWER_OFFSET = 1;

    public static double TICKS_PER_CM = 635;

    public static double INTAKE_MIN_POSITION = 0.0;
    public static double INTAKE_MAX_POSITION = 1.0;

    public static double ARM_MAX_POWER = 0.75;
    public static double ARM_MIN_POWER = 0.1;
    public static double ARM_REST_POWER = 0.3;

    public static double ARM_REVERSE_COORDINATES = 215;
    public static double ARM_MAX_COORDINATES = 400;

    public static double ARM_MIN_COORDINATES = 0;

    public static double SERVO_COORDINATES = 260;
    public static double SERVO_START = 0;
    public static double SERVO_END = 1;
    public static double SERVO_CONSTANT = 0.7;
    public static double ARM_CUT_SPEED = 5;
    public static double ARM_BLIND_SPOT = 2;

    /**
     * Mapping function used to return the value inside an interval applied
     * to another interval
     *
     * @param x         the value the function will convert
     * @param inputMin  min or left margin for the first interval
     * @param inputMax  max or right margin for the first interval
     * @param outputMin min or left margin for the second interval
     * @param outputMax max or right margin for the second interval
     * @return the value converted
     */
    public static double map(
            double x,
            double inputMin, double inputMax,
            double outputMin, double outputMax
    ) {
        return (x - inputMin) / (inputMax - inputMin) * (outputMax - outputMin) + outputMin;
    }

    public static double distance(double x1, double y1, double x2, double y2) {
        return Math.sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
    }
}
