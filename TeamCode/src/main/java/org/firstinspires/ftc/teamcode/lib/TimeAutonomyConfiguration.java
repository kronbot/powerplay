package org.firstinspires.ftc.teamcode.lib;

public interface TimeAutonomyConfiguration {
    double getSpeed();
    double getCPS(); // centimeters per second
    double getDPS(); // degrees per second
    double getIntakeUpdateSeconds();
    double getError();
    double getErrorUntil();
}
