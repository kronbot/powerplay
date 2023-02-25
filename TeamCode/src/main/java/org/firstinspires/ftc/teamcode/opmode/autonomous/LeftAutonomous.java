package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.KronBot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.lib.SlideControl;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name = "Left autonomous")
public class LeftAutonomous extends LinearOpMode {
    private KronBot robot;
    private SlideControl slideControl;

    public static double prepareInitialX = 40, prepareInitialY = 0;
    public static double initialJunctionX = 50, initialJunctionY = -10, initialJunctionHeading = 30;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new KronBot();
        robot.initHardwareMap(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        slideControl = new SlideControl(robot, telemetry);

        TrajectorySequence prepareInitialCone = drive
                .trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                .addTemporalMarker(0.2, () -> slideControl.setState(SlideControl.State.THIRD))
                .lineTo(new Vector2d(prepareInitialX, prepareInitialY))
                .build();
        TrajectorySequence putFirstCone = drive
                .trajectorySequenceBuilder(new Pose2d(prepareInitialX, prepareInitialY, 0))
                .lineToSplineHeading(new Pose2d(initialJunctionX, initialJunctionY, Math.toRadians(initialJunctionHeading)))
                .addTemporalMarker(0.2, () -> slideControl.setState(SlideControl.State.GROUND))
                .build();

        if (isStopRequested()) return;
        waitForStart();

        drive.followTrajectorySequence(prepareInitialCone);
        sleep(500);
//        while (opModeIsActive() && slideControl.getState() != SlideControl.State.REST) {
//            slideControl.showDebugTelemetry();
//        }
        drive.followTrajectorySequence(putFirstCone);

        while (!isStopRequested() && opModeIsActive())
            drive.update();
    }
}
