package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.KronBot;
import org.firstinspires.ftc.teamcode.autonomous.configurations.TestConfiguration;
import org.firstinspires.ftc.teamcode.lib.autonomous.AutonomousManager;
import org.firstinspires.ftc.teamcode.lib.autonomous.GlobalCoordinatePosition;

@Config
@Autonomous
public class PIDTuner extends LinearOpMode {
    public static double DISTANCE = 50;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private final KronBot robot = new KronBot();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.initHardwareMap(hardwareMap);
        GlobalCoordinatePosition position = new GlobalCoordinatePosition(robot.leftEncoder, robot.rightEncoder, robot.frontEncoder, new TestConfiguration(), telemetry);
        Thread positionThread = new Thread(position);
        waitForStart();

        positionThread.start();

        while (opModeIsActive()) {
            telemetry.addData("x", position.getX());
            telemetry.addData("y", position.getY());
            telemetry.addData("angle", position.getAngle());
        }

    }
}
