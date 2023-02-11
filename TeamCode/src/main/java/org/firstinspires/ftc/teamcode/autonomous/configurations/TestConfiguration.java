package org.firstinspires.ftc.teamcode.autonomous.configurations;

import org.firstinspires.ftc.teamcode.lib.autonomous.AutonomousConfiguration;

public class TestConfiguration implements AutonomousConfiguration {
    // all you have to do is sa pui valori aici din teste

    @Override
    public double getLateralDistance() {
        return 19.26;
    }

    @Override
    public double getRadianPerTicks() {
        return 0;
    }

    @Override
    public double getVerticalOffsetPerRadian() {
        return 8737.4;
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