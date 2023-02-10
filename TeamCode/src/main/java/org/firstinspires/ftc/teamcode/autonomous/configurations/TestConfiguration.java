package org.firstinspires.ftc.teamcode.autonomous.configurations;

import org.firstinspires.ftc.teamcode.lib.autonomous.AutonomousConfiguration;

public class TestConfiguration implements AutonomousConfiguration {
    @Override
    public double getLateralDistance() {
        return 0;
    }

    @Override
    public double getRadianPerTicks() {
        return 0;
    }

    @Override
    public double getVerticalOffsetPerRadian() {
        return 0;
    }

    @Override
    public double getAcceptableError() {
        return 0;
    }

    @Override
    public double getKp() {
        return 0;
    }

    @Override
    public double getKi() {
        return 0;
    }

    @Override
    public double getKd() {
        return 0;
    }
}
