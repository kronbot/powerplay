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
public class Autonomiecaca extends LinearOpMode{
    @Override
    public void runOpMode(){
        SampleMecanumDrive drivetrain=new SampleMecanumDrive(hardwareMap);
        waitForStart();
        Trajectory  FirstCone= drivetrain.trajectoryBuilder(new Pose2d(0,0,0))
                .lineTo(new Vector2d(40,5))
                .splineToSplineHeading(new Pose2d(62,15,Math.toRadians(-40)),Math.toRadians(-40))
                .build();
        drivetrain.followTrajectory(FirstCone);
    }
}