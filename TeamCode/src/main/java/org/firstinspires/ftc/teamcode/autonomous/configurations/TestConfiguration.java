package org.firstinspires.ftc.teamcode.autonomous.configurations;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.lib.autonomous.AutonomousConfiguration;

@Config
public class TestConfiguration implements AutonomousConfiguration {
    public static double lateralDistance = 19.26;
    public static double verticalOffsetPerRadian = 8737.4;
    public static double acceptableError = 0.5;
    public static double maxSpeed = 0.3;
    public static long coordinateDelay = 200;
    public static double kP = 0.1;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double angleKp = 0.01;
    public static double angleKi = 0.0;
    public static double angleKd = 0.0;


    @Override
    public double getLateralDistance() {
        return lateralDistance;
    }

    @Override
    public double getVerticalOffsetPerRadian() {
        return verticalOffsetPerRadian;
    }

    @Override
    public double getAcceptableError() {
        return acceptableError;
    }

    @Override
    public double getMaxSpeed() {
        return maxSpeed;
    }

    @Override
    public long getCoordinateDelay() {
        return coordinateDelay;
    }

    @Override
    public double getKp() {
        return kP;
    }

    @Override
    public double getKi() {
        return kI;
    }

    @Override
    public double getKd() {
        return kD;
    }

    @Override
    public double getAngleKp() {
        return angleKp;
    }

    @Override
    public double getAngleKi() {
        return angleKi;
    }

    @Override
    public double getAngleKd() {
        return angleKd;
    }
}
