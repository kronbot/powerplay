package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.KronBot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.lib.SlideControl;
import org.firstinspires.ftc.teamcode.lib.SlideControlRunnable;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name = "Left autonomous", group = "Autonomous")
public class LeftAutonomous extends LinearOpMode {
    private KronBot robot;
    private SlideControl slideControl;
    private SlideControlRunnable slideControlRunnable;

    public static double prepareInitialX = 40, prepareInitialY = 0;
    public static double initialJunctionX = 48, initialJunctionY = -12.5, initialJunctionHeading = -30;
    public static Integer firstConeCoordinate = 700;

    public static double getConeX = 35, getConeY = -40, getConeHeading = 90;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new KronBot();
        robot.initHardwareMap(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        slideControl = new SlideControl(robot, telemetry);
        slideControlRunnable = new SlideControlRunnable(slideControl);
        Thread slideControlThread = new Thread(slideControlRunnable);

        TrajectorySequence prepareInitialCone = drive
                .trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                .addTemporalMarker(0.2, () -> slideControl.setState(SlideControl.State.THIRD))
                .lineTo(new Vector2d(prepareInitialX, prepareInitialY))
                .build();
        TrajectorySequence putFirstCone = drive
                .trajectorySequenceBuilder(new Pose2d(prepareInitialX, prepareInitialY, 0))
                .lineToSplineHeading(new Pose2d(initialJunctionX, initialJunctionY, Math.toRadians(initialJunctionHeading)))
                .build();
        TrajectorySequence goToCones = drive
                .trajectorySequenceBuilder(new Pose2d(initialJunctionX, initialJunctionY, Math.toRadians(initialJunctionHeading)))
                .setReversed(true)
                .lineTo(new Vector2d(initialJunctionX - 2, initialJunctionY - 2))
                .setReversed(false)
                .lineToSplineHeading(new Pose2d(getConeX, getConeY, getConeHeading))
                .build();

        if (isStopRequested()) return;
        waitForStart();
        slideControlThread.start();

        robot.controlIntake(0.0);
        sleep(300);

        drive.followTrajectorySequence(prepareInitialCone);
        while (
                opModeIsActive() &&
                slideControl.getState() != SlideControl.State.REST
        );
        drive.followTrajectorySequence(putFirstCone);

        sleep(200);
        robot.controlIntake(1.0);
        slideControl.setCoordinate(firstConeCoordinate);

        sleep(200);
        drive.followTrajectorySequence(goToCones);

//        while (
//                opModeIsActive() &&
//                slideControl.getState() != SlideControl.State.REST
//        );

        while (!isStopRequested() && opModeIsActive())
            drive.update();

        slideControlRunnable.stop();
    }
}
