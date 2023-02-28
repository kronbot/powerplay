package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.KronBot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.lib.SlideControl;
import org.firstinspires.ftc.teamcode.lib.TagDetection;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;

@Config
@Autonomous(name = "Left autonomous", group = "Autonomous")
public class LeftAutonomous extends LinearOpMode {
    public static double prepareInitialX = 35, prepareInitialY = -8;
    public static double initialJunctionX = 58, initialJunctionY = -13.5, initialJunctionHeading = -30;
    //   private SlideControlRunnable slideControlRunnable;
    public static Integer firstConeCoordinate = 500;
    public static Integer secondConeCoordinate = 600;
    public static double getConeX = 54.5, getConeY = 19.5, getConeHeading = 90;
    public static double junctionX = 55, junctionY = -18.5, junctionHeading = -90;
    public static double junctionStraightX = 56.5;
    public static double parkingX = 53, parkingYPos1 = 15, parkingYPos2 = -5, parkingYPos3 = -30, parkingHeading = 90;
    private KronBot robot;
    private SlideControl slideControl;
    private TagDetection tagDetection;
    private AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new KronBot();
        robot.initHardwareMap(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        slideControl = new SlideControl(robot, telemetry);
//        slideControlRunnable = new SlideControlRunnable(slideControl);
//        Thread slideControlThread = new Thread(slideControlRunnable);

        tagDetection = new TagDetection(hardwareMap, telemetry);

        TrajectorySequence prepareInitialCone = drive
                .trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                .addTemporalMarker(1, () -> slideControl.setState(SlideControl.State.THIRD))
                .lineTo(new Vector2d(prepareInitialX, prepareInitialY))
                .build();
        TrajectorySequence putFirstCone = drive
                .trajectorySequenceBuilder(new Pose2d(prepareInitialX, prepareInitialY, 0))
                .lineToSplineHeading(new Pose2d(initialJunctionX, initialJunctionY, Math.toRadians(initialJunctionHeading)))
                .build();
        TrajectorySequence goToCones = drive
                .trajectorySequenceBuilder(new Pose2d(initialJunctionX, initialJunctionY, Math.toRadians(initialJunctionHeading)))
                .setReversed(true)
                .lineTo(new Vector2d(initialJunctionX - 2, initialJunctionY + 2))
                .lineToSplineHeading(new Pose2d(getConeX, getConeY, Math.toRadians(getConeHeading)))
                .build();
        TrajectorySequence goToJunction = drive
                .trajectorySequenceBuilder(new Pose2d(getConeX, getConeY, Math.toRadians(getConeHeading)))
                .setReversed(true)
                .lineTo(new Vector2d(junctionX, junctionY))
                .turn(Math.toRadians(junctionHeading))
//                .lineToSplineHeading(new Pose2d(junctionX, junctionY, Math.toRadians(junctionHeading)))
                .addTemporalMarker(0.5, () -> slideControl.setState(SlideControl.State.THIRD))
                .lineTo(new Vector2d(junctionStraightX, junctionY))
                .build();

        TrajectorySequence parkingPos1 = drive.trajectorySequenceBuilder(new Pose2d(junctionStraightX, junctionY, Math.toRadians(0)))
                .lineToSplineHeading(new Pose2d(parkingX, parkingYPos1, Math.toRadians(parkingHeading)))
                .build();
        TrajectorySequence parkingPos2 = drive.trajectorySequenceBuilder(new Pose2d(junctionStraightX, junctionY, Math.toRadians(0)))
                .lineToSplineHeading(new Pose2d(parkingX, parkingYPos2, Math.toRadians(parkingHeading)))
                .build();
        TrajectorySequence parkingPos3 = drive.trajectorySequenceBuilder(new Pose2d(junctionStraightX, junctionY, Math.toRadians(0)))
                .lineToSplineHeading(new Pose2d(parkingX, parkingYPos3, Math.toRadians(parkingHeading)))
                .build();

        if (isStopRequested()) return;

//        slideControlThread.start();

        while (!isStarted() && !isStopRequested()) {
            tagOfInterest = tagDetection.detectTag();
            if (tagOfInterest != null) {
                telemetry.addData("tag", tagOfInterest.id);
                telemetry.update();
            }
        }

        robot.controlIntake(0.0);

        drive.followTrajectorySequence(prepareInitialCone);

        drive.followTrajectorySequence(putFirstCone);
        robot.controlIntake(1.0);
        slideControl.setCoordinate(firstConeCoordinate);

        drive.followTrajectorySequence(goToCones);
        robot.controlIntake(0.0);
        sleep(500);
        slideControl.setCoordinate(firstConeCoordinate + 750);
        sleep(200);

        drive.followTrajectorySequence(goToJunction);
        robot.controlIntake(1.0);
        slideControl.setCoordinate(secondConeCoordinate);

        if (tagOfInterest != null) {
            if (tagOfInterest.id == 1)
                drive.followTrajectorySequence(parkingPos1);
            else if (tagOfInterest.id == 2)
                drive.followTrajectorySequence(parkingPos2);
            else
                drive.followTrajectorySequence(parkingPos3);
        } else
            drive.followTrajectorySequence(parkingPos2);

        while (!isStopRequested() && opModeIsActive()) ;
//            drive.update();

//        slideControlRunnable.stop();
    }
}
