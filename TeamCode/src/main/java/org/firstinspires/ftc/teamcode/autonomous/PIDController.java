package org.firstinspires.ftc.teamcode.autonomous;

public interface PIDController {
    double getSpeed(double current);
    void setTarget(double target);
    double getTarget();
}
