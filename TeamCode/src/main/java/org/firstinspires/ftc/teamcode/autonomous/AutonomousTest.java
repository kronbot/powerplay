package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
@Config
@Autonomous(name = "random")
public class AutonomousTest extends LinearOpMode{
    @Override
    public void runOpMode(){
        SampleMecanumDrive drivetrain=new SampleMecanumDrive(hardwareMap);
        waitForStart();
        Trajectory  FirstCone= drivetrain.trajectoryBuilder(new Pose2d(0,0,Math.toRadians(90)))
                .lineTo(new Vector2d(40,8))
                .lineToSplineHeading(new Pose2d(60, 10, Math.toRadians(50)))
                .build();
        drivetrain.followTrajectory(FirstCone);
    }
}