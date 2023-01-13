package org.firstinspires.ftc.teamcode.lib;

public class PowerplayTimeAutonomyConfiguration implements TimeAutonomyConfiguration {
    private final double SPEED = 0.65;
    private final double cps = 112.75;
    private final double dps = 12;
    private final double intakeUpdateSeconds = 0.6;
    private final double error = 0.166665;
    private final double errorUntil = 0.365;

    @Override
    public double getSpeed() {
        return SPEED;
    }

    @Override
    public double getCPS() {
        return cps;
    }

    @Override
    public double getDPS() {
        return dps;
    }

    @Override
    public double getIntakeUpdateSeconds() {
        return intakeUpdateSeconds;
    }

    @Override
    public double getError() {
        return error;
    }

    @Override
    public double getErrorUntil() {
        return errorUntil;
    }
}
