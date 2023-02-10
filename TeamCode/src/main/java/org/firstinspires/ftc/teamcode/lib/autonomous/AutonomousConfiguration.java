package org.firstinspires.ftc.teamcode.lib.autonomous;

// holds calibration values
public interface AutonomousConfiguration {
    double getLateralDistance(); // i guess
    double getRadianPerTicks();
    double getVerticalOffsetPerRadian();
    double getAcceptableError();
    double getKp();
    double getKi();
    double getKd();
}
