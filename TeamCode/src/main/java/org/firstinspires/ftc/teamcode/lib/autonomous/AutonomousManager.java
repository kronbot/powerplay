package org.firstinspires.ftc.teamcode.lib.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.analysis.integration.IterativeLegendreGaussIntegrator;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.KronBot;
import org.firstinspires.ftc.teamcode.lib.RobotControl;
import org.firstinspires.ftc.teamcode.lib.Utils;

// some IoC might be nice...
public class AutonomousManager {
    private final AutonomousConfiguration configuration;
    private final KronBot robot;
    private final GlobalCoordinatePosition position;
    private final Telemetry telemetry;
    private final Thread positionThread;
    private final LinearOpMode opMode;

    public AutonomousManager(
            AutonomousConfiguration configuration,
            KronBot robot,
            Telemetry telemetry,
            LinearOpMode opMode
    ) {
        this.configuration = configuration;
        this.robot = robot;
        this.position = new GlobalCoordinatePosition(
                robot.leftEncoder,
                robot.rightEncoder,
                robot.frontEncoder,
                configuration,
                telemetry
        );
        this.telemetry = telemetry;
        this.positionThread = new Thread(position);
        this.opMode = opMode;
    }

    public void initialize() {
        positionThread.run();
    }

    public void linear(double distance) {
        final double targetX = position.getX() + Math.sin(position.getAngle()) * distance;
        final double targetY = position.getY() + Math.cos(position.getAngle()) * distance;

        double lastAngleError = 0, lastError = 0;
        double angleError = 0, error = Utils.distance(position.getX(), position.getY(), targetX, targetY);
        double angleIntegralSum = 0, integralSum = 0;

        ElapsedTime timer = new ElapsedTime();
        int times = 0;
        telemetry.addData("error", error);
        while (opMode.opModeIsActive() && Math.abs(error) > configuration.getAcceptableError()) {
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

            double leftPower = Utils.map(distancePower + anglePower, 0, 1, 0, configuration.getMaxSpeed());
            double rightPower = Utils.map(distancePower - anglePower, 0, 1, 0, configuration.getMaxSpeed());

            this.telemetry.addData("times", times);
            this.telemetry.addData("error", error);
            this.telemetry.addData("distance", Math.hypot(targetX - position.getX(), targetY - position.getY()));
            this.telemetry.addData("target x", targetX);
            this.telemetry.addData("target y", targetY);
            this.telemetry.addData("x", position.getX());
            this.telemetry.addData("y", position.getY());
            this.telemetry.addData("angle", position.getAngle());
            this.telemetry.addData("angle error", angleError);
            this.telemetry.addData("integral sum", integralSum);
            this.telemetry.addData("angle integral sum", angleIntegralSum);
            this.telemetry.addData("distance derivative", distanceDerivative);
            this.telemetry.addData("angle derivative", angleDerivative);
            this.telemetry.addData("distance power", distancePower);
            this.telemetry.addData("angle power", anglePower);
            this.telemetry.addData("left", leftPower);
            this.telemetry.addData("right", rightPower);
            this.telemetry.addData("thread alive", positionThread.isAlive());
            this.telemetry.update();

            robot.drive(leftPower, rightPower, leftPower, rightPower, configuration.getMaxSpeed());

            lastAngleError = angleError;
            lastError = error;
            error = Utils.distance(position.getX(), position.getY(), targetX, targetY);
            angleError = Math.atan2(targetY - position.getY(), targetX - position.getX());
            times++;
            timer.reset();
        }
    }

    public void rotate(double degrees) {
        double target = position.getAngle() + degrees;
        double lastError = 0, error = target - position.getAngle();
        double integralSum = 0;
        ElapsedTime timer = new ElapsedTime();
        while (Math.abs(error) < configuration.getAcceptableError()) {
            error = target - position.getAngle();

            integralSum += error;
            double derivative = (error - lastError) / timer.seconds();
            double power = (
                    (configuration.getAngleKp() * error) +
                    (configuration.getAngleKi() * integralSum) +
                    (configuration.getAngleKd() * derivative)
            );

            robot.drive(power, -power, power, -power, configuration.getMaxSpeed());

            lastError = error;
            timer.reset();
        }
    }

    public void stop() {
        position.stop();
    }
}
