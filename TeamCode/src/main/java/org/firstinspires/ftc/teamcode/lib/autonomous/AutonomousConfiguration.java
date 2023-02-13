package org.firstinspires.ftc.teamcode.lib.autonomous;

// holds calibration values
public interface AutonomousConfiguration {
    double getLateralDistance(); // i guess
    double getVerticalOffsetPerRadian();
    double getAcceptableError();
    double getMaxSpeed();
    double getKp();
    double getKi();
    double getKd();
    double getAngleKp();
    double getAngleKi();
    double getAngleKd();
}
