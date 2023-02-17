package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.KronBot;

public class SlideControl {
    // stores the states with the coordinates
    public enum State {
        FIRST,
        SECOND,
        THIRD,
        GROUND,
        REST,
        CUSTOM,
    }

    private class StateManager {
        private State currentState = State.REST;

        private Integer groundCoordinate = Utils.SLIDE_GROUND_COORDINATE;
        private Integer firstCoordinate = Utils.SLIDE_FIRST_COORDINATE;
        private Integer secondCoordinate = Utils.SLIDE_SECOND_COORDINATE;
        private Integer thirdCoordinate = Utils.SLIDE_THIRD_COORDINATE;


        public Integer getStateCoordinate(State state) {
            if (state == null)
                throw new IllegalArgumentException("State is null :(");
            switch (state) {
                case GROUND:
                    return groundCoordinate;
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

        public synchronized void setCurrentState(State currentState) {
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

        public synchronized void setCurrentStateToCustom(Integer coordinate) {
            if (this.currentState != State.REST)
                robot.slideDc.setPower(restPower);
            this.currentState = State.CUSTOM;

            robot.slideDc.setTargetPosition(coordinate);
            robot.slideDc.setPower(power);
            if (robot.slideDc.getCurrentPosition() > coordinate)
                robot.slideDc.setPower(-power);
            robot.slideDc.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    private KronBot robot;
    private Telemetry telemetry;
    private StateManager stateManager;

    private static final double power = Utils.SLIDE_POWER;
    private static final double restPower = Utils.SLIDE_REST;
    private static final double restPowerSliding = Utils.SLIDE_REST_SLIDING;

    private boolean intakePressed = true;
    private boolean closeIntakeAtPress = false;

    private Integer minCoordinate = 50;
    private Integer maxCoordinate = 4100;

    public SlideControl(KronBot robot, Telemetry telemetry) {
        this.robot = robot;
        this.telemetry = telemetry;
        this.stateManager = new StateManager();
    }

    public void showDebugTelemetry() {
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
            case CUSTOM:
                stateName = "CUSOM";
                break;
            default:
                stateName = "INVALID";
                break;
        }
        telemetry.addData("Slide state", stateName);
    }

    public void control(Gamepad gamepad, boolean debug) {
        if (debug)
            showDebugTelemetry();
        loop(gamepad, false);
    }

    public State getState() {
        return stateManager.getCurrentState();
    }

    public synchronized void setState(State state) {
        if (state.equals(State.GROUND))
            robot.controlIntake(1);
        stateManager.setCurrentState(state);
    }

    public synchronized void setCoordinate(Integer coordinate) {
        stateManager.setCurrentStateToCustom(coordinate);
    }

    // for autonomous
    public void loop() {
        // checking if the current state is finished
        if (!robot.slideDc.isBusy() && stateManager.getCurrentState() != State.REST)
            stateManager.setCurrentState(State.REST);
    }

    public void loop(Gamepad gamepad, boolean usingEnds) {
        // updating the state
        if (gamepad.a) {
            if (robot.intakePosition() == 0)
                robot.controlIntake(1);

            stateManager.setCurrentState(State.GROUND);
        } else if (gamepad.b || gamepad.x || gamepad.y) {
            if (robot.intakePosition() == 1 && closeIntakeAtPress)
                robot.controlIntake(0);

            if (gamepad.b)
                stateManager.setCurrentState(State.FIRST);
            else if (gamepad.x)
                stateManager.setCurrentState(State.SECOND);
            else if (gamepad.y)
                stateManager.setCurrentState(State.THIRD);
        }

        if (gamepad.right_trigger > 0 && gamepad.left_trigger < Utils.EPS &&
                (robot.slideDc.getCurrentPosition() <= maxCoordinate || !usingEnds)) {
            robot.controlSlide(slidePower(gamepad.right_trigger));
            return;
        } else if (gamepad.left_trigger > 0 && gamepad.right_trigger < Utils.EPS &&
                robot.slideDc.getCurrentPosition() >= minCoordinate) {
            robot.controlSlide(-slidePower(gamepad.left_trigger));
            return;
        } else if (robot.slideDc.getMode() == DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
            robot.slideDc.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            stateManager.setCurrentState(State.REST);
            robot.slideDc.setPower(restPowerSliding);
        }

        // checking if the current state is finished
        if (!robot.slideDc.isBusy() && stateManager.getCurrentState() != State.REST)
            stateManager.setCurrentState(State.REST);
    }

    private double slidePower(double power) {
        if (power < 0.75)
            return Utils.map(power, 0, 0.75, 0, 0.5);
        return Utils.map(power, 0.75, 1, 0.5, 1);
    }

    public void intake(boolean action) {
        if (action && intakePressed) {
            if (Double.compare(robot.intakePosition(), 1.0) == 0)
                robot.controlIntake(0);
            else if (Double.compare(robot.intakePosition(), 0.0) == 0)
                robot.controlIntake(1);
            intakePressed = false;
        } else if (!action)
            intakePressed = true;
    }
}