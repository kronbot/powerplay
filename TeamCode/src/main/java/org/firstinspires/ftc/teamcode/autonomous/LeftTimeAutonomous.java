package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.KronBot;
import org.firstinspires.ftc.teamcode.lib.AutonomousBuilder;
import org.firstinspires.ftc.teamcode.lib.PowerplayTimeAutonomyConfiguration;
import org.firstinspires.ftc.teamcode.lib.SlideControl;
import org.firstinspires.ftc.teamcode.lib.TimeAutonomyConfiguration;

@Autonomous
public class LeftTimeAutonomous extends LinearOpMode {
    private final KronBot robot = new KronBot();
    private final TimeAutonomyConfiguration configuration = new PowerplayTimeAutonomyConfiguration();
    private final AutonomousBuilder autonomousBuilder = new AutonomousBuilder(this, configuration, robot);

    @Override
    public void runOpMode() throws InterruptedException {
        autonomousBuilder.initialize(hardwareMap);

        autonomousBuilder.toggleIntake();
        autonomousBuilder.setSlideState(SlideControl.State.FIRST);
        autonomousBuilder.delay(0.25);
        autonomousBuilder.forward(0.7760532150776053);
        autonomousBuilder.rotateClockwise(0.5);
        autonomousBuilder.delay(5.0);
        autonomousBuilder.toggleIntake();

        autonomousBuilder.uninitialize();
    }
}
