package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.utils.SlideLevelControl.State;

@TeleOp(name = "Slide control prototype")
public class SlideControlPrototype extends OpMode {

    // default motor power
    private static double power = 0.5;
    private static double restPower = 0.05;

    private class StateManager {
        private State currentState = State.REST;
        private Integer firstCoordinate = 2100;
        private Integer secondCoordinate = 3100;
        private Integer thirdCoordinate = 4100;

        public State getCurrentState() {
            return currentState;
        }

        public void setCurrentState(State currentState) {
            if (this.currentState != State.REST)
                robot.slideDc.setPower(restPower);
            this.currentState = currentState;
            if (currentState == State.REST) {
                return;
            }

            robot.slideDc.setTargetPosition(getStateCoordinate(currentState));
            robot.slideDc.setPower(-power);
            robot.slideDc.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

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
    }

    private KronBot robot;
    private StateManager stateManager;

    public SlideControlPrototype() {
        robot = new KronBot();
        stateManager = new StateManager();
    }

    @Override
    public void init() {
        robot.initHardwareMap(hardwareMap);
        robot.slideDc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void loop() {
        telemetry.addData("Position", robot.slideDc.getCurrentPosition());
        updateState();

        // checking if the current state is finished
        if (!robot.slideDc.isBusy() && stateManager.getCurrentState() != State.REST)
            stateManager.setCurrentState(State.REST);
    }

    // updates the state based on gamepad input
    public void updateState() {
        if (gamepad1.dpad_up)
            stateManager.setCurrentState(State.THIRD);
        if (gamepad1.dpad_left)
            stateManager.setCurrentState(State.SECOND);
        if (gamepad1.dpad_right)
            stateManager.setCurrentState(State.FIRST);
    }
}
