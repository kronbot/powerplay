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
        REST
    }

    private class StateManager {
        private State currentState = State.REST;
        private Integer firstCoordinate = 1400;
        private Integer secondCoordinate = 2150;
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
    private static final double restPowerSliding = Utils.SLIDE_REST_SLIDING;

    private boolean intakePressed = true;

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
            default:
                stateName = "INVALID";
                break;
        }
        telemetry.addData("Slide state", stateName);
    }

    public void control(Gamepad gamepad, boolean debug) {
        if (debug)
            showDebugTelemetry();
        loop(gamepad);
    }

    public State getState() {
        return stateManager.getCurrentState();
    }

    public void setState(State state) {
        if (state.equals(State.GROUND))
            robot.controlIntake(1);
        stateManager.setCurrentState(state);
    }

    // for autonomous
    public void loop() {
        // checking if the current state is finished
        if (!robot.slideDc.isBusy() && stateManager.getCurrentState() != State.REST)
            stateManager.setCurrentState(State.REST);
    }

    public void loop(Gamepad gamepad) {
        // updating the state
        if (gamepad.b) {
//            if (robot.intakePosition() == 1)
//                robot.controlIntake(0);
            stateManager.setCurrentState(State.FIRST);

        }
        if (gamepad.x)
            stateManager.setCurrentState(State.SECOND);
        if (gamepad.y)
            stateManager.setCurrentState(State.THIRD);
        if (gamepad.a) {
            if (robot.intakePosition() == 0)
                robot.controlIntake(1);
            stateManager.setCurrentState(State.GROUND);
        }

        if (gamepad.right_trigger > 0 && gamepad.left_trigger < Utils.EPS) {
            robot.slideDc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.controlSlide(gamepad.right_trigger);
            return;
        } else if (gamepad.left_trigger > 0 && gamepad.right_trigger < Utils.EPS) {
            robot.slideDc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.controlSlide(-gamepad.left_trigger);
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