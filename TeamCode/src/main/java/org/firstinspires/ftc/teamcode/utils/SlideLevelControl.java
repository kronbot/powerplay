package org.firstinspires.ftc.teamcode.utils;

import org.firstinspires.ftc.teamcode.KronBot;

public class SlideLevelControl {
    // default motor power
    private static double power = 1;
    private static double restPower = 0.05;

    private Utils.State currentState = Utils.State.REST;
    private Integer zeroCoordinate = 0;
    private Integer firstCoordinate = 2100;
    private Integer secondCoordinate = 3100;
    private Integer thirdCoordinate = 4100;

    private final KronBot robot;

    public SlideLevelControl(KronBot robot) {
        this.robot = robot;
    }

    public Utils.State getCurrentState() {
        return currentState;
    }

    public void setCurrentState(Utils.State currentState) {
        if (this.currentState != Utils.State.REST)
            robot.controlSlide(restPower);
        this.currentState = currentState;
        if (currentState == Utils.State.REST) {
            return;
        }

        robot.controlSlideWithEncoder(power, getStateCoordinate(currentState));
    }

    public Integer getStateCoordinate(Utils.State state) {
        if (state == null)
            throw new IllegalArgumentException("State is null :(");
        switch (state) {
            case FIRST:
                return firstCoordinate;
            case SECOND:
                return secondCoordinate;
            case THIRD:
                return thirdCoordinate;
            case ZERO:
                return zeroCoordinate;
            default:
                throw new IllegalArgumentException("State is invalid :(");
        }
    }

    public void updateState(boolean first, boolean second, boolean third, boolean zero) {
        if (first)
            setCurrentState(Utils.State.THIRD);
        if (second)
            setCurrentState(Utils.State.SECOND);
        if (third)
            setCurrentState(Utils.State.FIRST);
        if (zero)
            setCurrentState(Utils.State.ZERO);
    }
}