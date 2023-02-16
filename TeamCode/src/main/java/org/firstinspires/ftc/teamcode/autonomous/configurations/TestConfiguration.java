package org.firstinspires.ftc.teamcode.autonomous.configurations;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.lib.autonomous.AutonomousConfiguration;

@Config
public class TestConfiguration implements AutonomousConfiguration {
    public static double lateralDistance = 18.26;
    public static double verticalOffsetPerRadian = 8737.4;
    public static double acceptableError = 0.00;
    public static double maxSpeed = 0.8;
    public static long coordinateDelay = 200;
    public static double kP = 0.0002;
    public static double kI = 0.000075;
    public static double kD = 0.0000001;
    public static double angleKp = 0.8;
    public static double angleKi = 0.00005;
    public static double angleKd = 0.0000005;
    public static double forwardAngleOffset = 0.025;
    public static double backwardAngleOffset = 0.01;


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

    @Override
    public double getForwardAngleOffset() {return forwardAngleOffset;}

    @Override
    public double getBackwardAngleOffset() {return backwardAngleOffset;}
}
