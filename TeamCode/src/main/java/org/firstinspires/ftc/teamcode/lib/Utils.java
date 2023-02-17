package org.firstinspires.ftc.teamcode.lib;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Utils {
    // very low constant
    // used for floating point comparations
    public static double EPS = 0.1;

    public static double MOTOR_ACCELERATION_POWER = 0.3;
    public static double MOTOR_POWER_OFFSET = 1;

    public static double SLIDE_POWER = 1;
    public static double SLIDE_REST = 0.05;
    public static double SLIDE_REST_SLIDING = 0.001;

    public static Integer SLIDE_GROUND_COORDINATE = 100;
    public static Integer SLIDE_FIRST_COORDINATE = 1500;
    public static Integer SLIDE_SECOND_COORDINATE = 2250;
    public static Integer SLIDE_THIRD_COORDINATE = 3475;


    public static double TICKS_PER_CM = 635;

    /**
     * Mapping function used to return the value inside an interval applied
     * to another interval
     *
     * @param x          the value the function will convert
     * @param inputMin   min or left margin for the first interval
     * @param inputMax   max or right margin for the first interval
     * @param outputMin  min or left margin for the second interval
     * @param outputMax  max or right margin for the second interval
     * @return           the value converted
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
