package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.KronBot;

public class SlideControl {
    // stores the states with the coordinates
    private enum State {
        FIRST,
        SECOND,
        THIRD,
        GROUND,
        REST
    }

    private class StateManager {
        private State currentState = State.REST;
        private Integer firstCoordinate = 1400;
        private Integer secondCoordinate = 2100;
        private Integer thirdCoordinate = 2900;

        public Integer getStateCoordinate(State state) {
            if (state == null)
                throw new IllegalArgumentException("State is null :(");
            switch (state) {
                case GROUND:
                    return 0;
                case FIRST:
                    return firstCoordinate;
                case SECOND:
                    return secondCoordinate;
                case THIRD:
                    return thirdCoordinate;
                default:
                    throw new IllegalArgumentException("State is invalid :(");
            }
        }

        public State getCurrentState() {
            return currentState;
        }

        public void setCurrentState(State currentState) {
            if (this.currentState != State.REST)
                robot.slideDc.setPower(restPower);
            this.currentState = currentState;
            if (currentState == State.REST)
                return;
            robot.slideDc.setTargetPosition(getStateCoordinate(currentState));
            robot.slideDc.setPower(power);
            if (robot.slideDc.getCurrentPosition() > getStateCoordinate(currentState))
                robot.slideDc.setPower(-power);
            robot.slideDc.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    private KronBot robot;
    private Telemetry telemetry;
    private StateManager stateManager;

    private static final double power = 1;
    private static final double restPower = Utils.SLIDE_REST;

    public SlideControl(KronBot robot, Telemetry telemetry) {
        this.robot = robot;
        this.telemetry = telemetry;
        this.stateManager = new StateManager();
    }

    private void showDebugTelemetry() {
        telemetry.addData("Slide busy", robot.slideDc.isBusy());
        telemetry.addData("Slide coordinate", robot.slideDc.getCurrentPosition());
        if (stateManager.getCurrentState() != State.REST)
            telemetry.addData(
                    "Slide target coordinate",
                    stateManager.getStateCoordinate(stateManager.getCurrentState())
            );

        String stateName;
        switch (stateManager.getCurrentState()) {
            case GROUND:
                stateName = "GROUND";
                break;
            case FIRST:
                stateName = "FIRST";
                break;
            case SECOND:
                stateName = "SECOND";
                break;
            case THIRD:
                stateName = "THIRD";
                break;
            case REST:
                stateName = "REST";
                break;
            default:
                stateName = "INVALID";
                break;
        }
        telemetry.addData("Slide state", stateName);
    }

    public void control(
            boolean first,
            boolean second,
            boolean third,
            boolean ground,
            boolean debug
    ) {
        if (debug)
            showDebugTelemetry();
        loop(first, second, third, ground);
    }

    public void loop(
            boolean first,
            boolean second,
            boolean third,
            boolean ground
    ) {
        // updating the state
        if (first)
            stateManager.setCurrentState(State.FIRST);
        if (second)
            stateManager.setCurrentState(State.SECOND);
        if (third)
            stateManager.setCurrentState(State.THIRD);
        if (ground) {
            intake(false, true);
            stateManager.setCurrentState(State.GROUND);
        }

        // checking if the current state is finished
        if (!robot.slideDc.isBusy() && stateManager.getCurrentState() != State.REST)
            stateManager.setCurrentState(State.REST);
    }

    public void slide(boolean up, boolean down) {
        if (up)
            robot.controlSlide(0.5);
        else if (down)
            robot.controlSlide(-0.5);
        else
            robot.controlSlide(0.05);
    }

    public void intake(boolean open, boolean close) {
        if (open)
            robot.controlIntake(0);
        else if (close)
            robot.controlIntake(1);
    }
}