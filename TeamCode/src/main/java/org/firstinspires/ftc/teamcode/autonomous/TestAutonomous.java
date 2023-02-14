package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.KronBot;
import org.firstinspires.ftc.teamcode.autonomous.configurations.TestConfiguration;
import org.firstinspires.ftc.teamcode.lib.Utils;
import org.firstinspires.ftc.teamcode.lib.autonomous.GlobalCoordinatePosition;

@Config
@Autonomous
public class TestAutonomous extends LinearOpMode {
    public static double DISTANCE = 70;
    private final KronBot robot = new KronBot();
    private final TestConfiguration configuration = new TestConfiguration();
    private GlobalCoordinatePosition position;


    @Override
    public void runOpMode() throws InterruptedException {
        robot.initHardwareMap(hardwareMap);
        position = new GlobalCoordinatePosition(
            robot.leftEncoder,
            robot.rightEncoder,
            robot.frontEncoder,
            configuration,
            telemetry
        );
        waitForStart();
        Thread positionThread = new Thread(position);
        positionThread.start();

        while (opModeIsActive()) {
            linear(DISTANCE);
            Thread.sleep(200);
            linear(-DISTANCE);
            Thread.sleep(200);
        }

        position.stop();
    }

    public void linear(double distance) {
        final double targetX = position.getX() + Math.sin(position.getAngle()) * distance;
        final double targetY = position.getY() + Math.cos(position.getAngle()) * distance;

        double lastAngleError = 0, lastError = 0;
        double angleError = 0, error = Utils.distance(position.getX(), position.getY(), targetX, targetY);
        double angleIntegralSum = 0, integralSum = 0;

        boolean backwards = distance < 0;

        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && Math.abs(error) > configuration.getAcceptableError()) {
            integralSum += error;
            angleIntegralSum += angleError;
            double distanceDerivative = (error - lastError) / timer.seconds();
            double angleDerivative = (angleError - lastAngleError) / timer.seconds();

            double distancePower = (
                    (configuration.getKp() * error) +
                            (configuration.getKi() * integralSum) +
                            (configuration.getKd() * distanceDerivative)
            );

            double anglePower = (
                    (configuration.getAngleKp() * angleError) +
                            (configuration.getAngleKi() * angleIntegralSum) +
                            (configuration.getAngleKd() * angleDerivative)
            );

            double leftPower = Utils.map(distancePower - anglePower, 0, 1, 0, configuration.getMaxSpeed());
            double rightPower = Utils.map(distancePower + anglePower, 0, 1, 0, configuration.getMaxSpeed());

            telemetry.addData("backwards", backwards);
            telemetry.addData("error", error);
            telemetry.addData("distance", Math.hypot(targetX - position.getX(), targetY - position.getY()));
            telemetry.addData("target x", targetX);
            telemetry.addData("target y", targetY);
            telemetry.addData("x", position.getX());
            telemetry.addData("y", position.getY());
            telemetry.addData("angle", position.getAngle());
            telemetry.addData("angle error", angleError);
            telemetry.addData("integral sum", integralSum);
            telemetry.addData("angle integral sum", angleIntegralSum);
            telemetry.addData("distance derivative", distanceDerivative);
            telemetry.addData("angle derivative", angleDerivative);
            telemetry.addData("distance power", distancePower);
            telemetry.addData("angle power", anglePower);
            telemetry.addData("left", leftPower);
            telemetry.addData("right", rightPower);
            telemetry.update();

            robot.drive(leftPower, rightPower, leftPower, rightPower, backwards ? -1 : 1);

            lastAngleError = angleError;
            lastError = error;
            error = Utils.distance(position.getX(), position.getY(), targetX, targetY);
            angleError = Math.atan2(targetY - position.getY(), targetX - position.getX());
            timer.reset();
        }
    }
}
