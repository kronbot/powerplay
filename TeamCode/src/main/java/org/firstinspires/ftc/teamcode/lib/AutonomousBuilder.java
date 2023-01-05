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
                slideControl.showDebugTelemetry();
                opMode.telemetry.update();

                slideControl.loop();
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
        opMode.waitForStart();

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
            delay(configuration.getErrorUntil(), "Going forward");
            return;
        }
        robot.drive(1.0, 1.0 - configuration.getError(), 1.0, 1.0 - configuration.getError(), configuration.getSpeed());
        delay(configuration.getErrorUntil(), "Going forward:");
        robot.drive(1.0, 1.0, 1.0, 1.0, configuration.getSpeed());
        delay(seconds - configuration.getErrorUntil(), "Going forward:");
    }

    public void backward(double seconds) {
        robot.drive(1.0, 1.0, 1.0, 1.0, -configuration.getSpeed());
        delay(seconds, "Going backward:");
    }

    public void translateLeft(double seconds) {
        robot.drive(1.0, -1.0, -1.0, 1.0, configuration.getSpeed());
        delay(seconds, "Going forward:");
    }

    public void translateRight(double seconds) {
        robot.drive(-1.0, 1.0, 1.0, -1.0, configuration.getSpeed());
        delay(seconds, "Going forward:");
    }

    public void rotateClockwise(double seconds) {
        robot.drive(1.0, -1.0, 1.0, -1.0, configuration.getSpeed());
        delay(seconds, "Rotate clockwise:");
    }

    public void rotateCounterClockwise(double seconds) {
        robot.drive(-1.0, 1.0, -1.0, 1.0, configuration.getSpeed());
        delay(seconds, "Rotate counter clockwise:");
    }

    public void setSlideState(SlideControl.State state) {
        slideControl.setState(state);
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
