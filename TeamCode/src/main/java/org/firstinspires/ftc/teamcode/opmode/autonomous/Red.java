package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.KronBot;

@Config
@Autonomous(name = "Red", group = "Autonomous")
public class Red extends LinearOpMode {
    private KronBot robot;

    public static Integer forward = 100;
    public static Integer backward = 850;
    public static Integer right = 1575;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new KronBot();
        robot.initHardwareMap(hardwareMap);

        waitForStart();

//        robot.drive(-1, -1, -1, -1, 0.5);
//        sleep(forward);
//        robot.stopMotors();
//        sleep(200);
        robot.clawServo.setPosition(0);
        sleep(1000);
        robot.drive(-1, -1, -1, -1, 0.5);
        sleep(backward);
        robot.stopMotors();
//        sleep(500);
//        robot.drive(-1, 1,  1, -1, 0.5);
//        sleep(right);
//        robot.stopMotors();
//        sleep(200);
//        robot.clawServo.setPosition(1);
    }
}
