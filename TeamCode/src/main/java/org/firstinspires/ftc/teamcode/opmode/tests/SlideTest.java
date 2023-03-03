package org.firstinspires.ftc.teamcode.opmode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.KronBot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.lib.SlideControl;

@Autonomous(name = "Slide Test", group = "Tests")
@Config
public class SlideTest extends LinearOpMode {
    private final KronBot robot = new KronBot();
    private SlideControl slideControl;

    private class SlideControlRunnable implements Runnable {
        private boolean running = true;

        @Override
        public void run() {
            while (running)
                slideControl.loop(true);
        }

        public synchronized void stop() {
            running = false;
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot.initHardwareMap(hardwareMap);
        slideControl = new SlideControl(robot, telemetry);
        SlideControlRunnable slideControlRunnable = new SlideControlRunnable();
        Thread slideControlThread = new Thread(slideControlRunnable);

        if (isStopRequested()) return;
        waitForStart();
        slideControlThread.start();

        while (opModeIsActive()) {
            slideControl.setState(SlideControl.State.SECOND);
            robot.drive(1, 1, 1, 1, 0.3);
            while (opModeIsActive() && slideControl.getState() != SlideControl.State.REST);
            robot.stopMotors();
            slideControl.setState(SlideControl.State.GROUND);
            robot.drive(1, 1, 1, 1, -0.3);
            sleep(3000);
            robot.stopMotors();
        }

        slideControlRunnable.stop();
    }

}
