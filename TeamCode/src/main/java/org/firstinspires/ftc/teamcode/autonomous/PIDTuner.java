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

@Config
@Autonomous
public class PIDTuner extends LinearOpMode {
    public static double DISTANCE = 50;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private final KronBot robot = new KronBot();

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());
        robot.initHardwareMap(hardwareMap);
        AutonomousManager autonomousManager = new AutonomousManager(new TestConfiguration(), robot, this.telemetry);

        waitForStart();

        while (opModeIsActive()) {
            if (!gamepad1.a) {
                autonomousManager.linear(20);
                autonomousManager.linear(-20);
            }
        }

        autonomousManager.stop();
    }
}
