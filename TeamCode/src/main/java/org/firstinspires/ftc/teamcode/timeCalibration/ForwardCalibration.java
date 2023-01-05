package org.firstinspires.ftc.teamcode.timeCalibration;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.KronBot;
import org.firstinspires.ftc.teamcode.lib.AutonomousBuilder;
import org.firstinspires.ftc.teamcode.lib.PowerplayTimeAutonomyConfiguration;
import org.firstinspires.ftc.teamcode.lib.TimeAutonomyConfiguration;

@Autonomous
public class ForwardCalibration extends LinearOpMode {
    private KronBot robot = new KronBot();
    private TimeAutonomyConfiguration configuration = new PowerplayTimeAutonomyConfiguration();
    private AutonomousBuilder autonomousBuilder = new AutonomousBuilder(this, configuration, robot);

    @Override
    public void runOpMode() throws InterruptedException {
        autonomousBuilder.initialize(hardwareMap);
        autonomousBuilder.forward(1.0);
    }
}
