package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.KronBot;

public class AutonomousBuilder {
    private final LinearOpMode opMode;
    private final KronBot robot;
    private final ElapsedTime runtime = new ElapsedTime();
    private final TimeAutonomyConfiguration configuration;
    private final SlideControl slideControl;

    public class SlideControlRunnable implements Runnable {
        @Override
        public void run() {
            while (!opMode.opModeIsActive()) {}
            while (opMode.opModeIsActive()) {
                synchronized (slideControl) {
                    slideControl.showDebugTelemetry();
                    opMode.telemetry.update();

                    slideControl.loop();
                }
            }
        }
    }

    public AutonomousBuilder(
            LinearOpMode opMode,
            TimeAutonomyConfiguration configuration,
            KronBot robot
    ) {
        this.opMode = opMode;
        this.configuration = configuration;
        this.robot = robot;
        this.slideControl = new SlideControl(robot, opMode.telemetry);
    }

    public void initialize(HardwareMap hardwareMap) {
        robot.initHardwareMap(hardwareMap);
        Thread slideThread = new Thread(new SlideControlRunnable());
        slideThread.start();

        slideControl.intake(false);
        delay(configuration.getIntakeUpdateSeconds(), "Initialization");
        slideControl.setState(SlideControl.State.GROUND);
        delayUntilSlideIsResting();
    }

    public void uninitialize() {
        slideControl.intake(false);
        delay(configuration.getIntakeUpdateSeconds(), "Uninitialization");
        slideControl.setState(SlideControl.State.GROUND);
        delayUntilSlideIsResting();
    }

    public void forward(double seconds) {
        if (seconds < configuration.getErrorUntil()) {
            robot.drive(1.0, 1.0 - configuration.getError(), 1.0, 1.0 - configuration.getError(), configuration.getSpeed());
            delay(seconds, "Going forward");
            robot.stopMotors();
            return;
        }
        robot.drive(1.0, 1.0 - configuration.getError(), 1.0, 1.0 - configuration.getError(), configuration.getSpeed());
        delay(configuration.getErrorUntil(), "Going forward:");
        robot.drive(1.0, 1.0, 1.0, 1.0, configuration.getSpeed());
        delay(seconds - configuration.getErrorUntil(), "Going forward:");
        robot.stopMotors();
    }

    public void backward(double seconds) {
        if (seconds < configuration.getErrorUntil()) {
            robot.drive(1.0, 1.0 - configuration.getErrorUntil(), 1.0, 1.0 - configuration.getError(), -configuration.getSpeed());
            delay(seconds, "Going backward:");
            robot.stopMotors();
            return;
        }
        robot.drive(1.0, 1.0 - configuration.getErrorUntil(), 1.0, 1.0 - configuration.getError(), -configuration.getSpeed());
        delay(configuration.getErrorUntil(), "Going backward:");
        robot.drive(1.0, 1.0, 1.0, 1.0, -configuration.getSpeed());
        delay(seconds - configuration.getErrorUntil(), "Going backward:");
        robot.stopMotors();
    }

    public void translateLeft(double seconds) {
        if (seconds < configuration.getErrorUntil()) {
            robot.drive(-1.0 + configuration.getErrorUntil(), 1.0, 1.0 - configuration.getErrorUntil(), -1.0, configuration.getSpeed());
            delay(seconds, "Translate left:");
            robot.stopMotors();
            return;
        }
        robot.drive(-1.0 + configuration.getErrorUntil(), 1.0, 1.0 - configuration.getErrorUntil(), -1.0, configuration.getSpeed());
        delay(configuration.getErrorUntil(), "Translate left:");
        robot.drive(-1.0, 1.0, 1.0, -1.0, configuration.getSpeed());
        delay(seconds - configuration.getErrorUntil(), "Translate left:");
        robot.stopMotors();
    }

    public void translateRight(double seconds) {
        if (seconds < configuration.getErrorUntil()) {
            robot.drive(1.0, -1.0 + configuration.getErrorUntil(), -1.0, 1.0 - configuration.getErrorUntil(), configuration.getSpeed());
            delay(seconds, "Translate right:");
            robot.stopMotors();
            return;
        }
        robot.drive(1.0, -1.0 + configuration.getErrorUntil(), -1.0, 1.0 - configuration.getErrorUntil(), configuration.getSpeed());
        delay(configuration.getErrorUntil(), "Translate right:");
        robot.drive(1.0, -1.0, -1.0, 1.0, configuration.getSpeed());
        delay(seconds - configuration.getErrorUntil(), "Translate right:");
        robot.stopMotors();
    }

    public void rotateClockwise(double seconds) {
        robot.drive(1.0, -1.0, 1.0, -1.0, configuration.getSpeed());
        delay(seconds, "Rotate clockwise:");
        robot.stopMotors();
    }

    public void rotateCounterClockwise(double seconds) {
        robot.drive(-1.0, 1.0, -1.0, 1.0, configuration.getSpeed());
        delay(seconds, "Rotate counter clockwise:");
        robot.stopMotors();
    }

    public void setSlideState(SlideControl.State state) {
        synchronized (slideControl) {
            slideControl.setState(state);
        }
    }

    public void toggleIntake() {
        if (Double.compare(robot.intakePosition(), 1.0) == 0)
            robot.controlIntake(0.0);
        else if (Double.compare(robot.intakePosition(), 0.0) == 0)
            robot.controlIntake(1.0);
        runtime.reset();
        delay(configuration.getIntakeUpdateSeconds(), "Intake:");
    }

    public void delay(double seconds) {
        runtime.reset();
        while (opMode.opModeIsActive() && runtime.seconds() < seconds) {
            opMode.telemetry.addData("Delay:", runtime.seconds());
            opMode.telemetry.update();
        }
    }

    public void delay(double seconds, String message) {
        runtime.reset();
        while (opMode.opModeIsActive() && runtime.seconds() < seconds) {
            opMode.telemetry.addData(message, runtime.seconds());
            opMode.telemetry.update();
        }
    }

    public void delayUntilSlideIsResting() {
        while (opMode.opModeIsActive() && slideControl.getState() != SlideControl.State.REST) {
            opMode.telemetry.addData("Slide moving:", true);
            opMode.telemetry.update();
        }
    }
}
