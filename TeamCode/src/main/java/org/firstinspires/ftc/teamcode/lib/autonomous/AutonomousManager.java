package org.firstinspires.ftc.teamcode.lib.autonomous;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.KronBot;
import org.firstinspires.ftc.teamcode.lib.RobotControl;
import org.firstinspires.ftc.teamcode.lib.Utils;

// some IoC might be nice...
public class AutonomousManager {
    private final AutonomousConfiguration configuration;
    private final KronBot robot;
    private final GlobalCoordinatePosition position;
    private final ElapsedTime timer = new ElapsedTime();

    public AutonomousManager(
            AutonomousConfiguration configuration,
            KronBot robot,
            Telemetry telemetry
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

        Thread positionThread = new Thread(position);
        positionThread.start();
    }

    public void linear(double distance) {
        double targetX = position.getX() + Math.cos(position.getAngle()) * distance;
        double targetY = position.getY() + Math.sin(position.getAngle()) * distance;

        double lastAngleError = 0, lastError = 0;
        double angleError, error = Utils.distance(position.getX(), position.getY(), targetX, targetY);
        double angleIntegralSum = 0, integralSum = 0;

        ElapsedTime timer = new ElapsedTime();

        while (Math.abs(error) < configuration.getAcceptableError()) {
            double x = position.getX();
            double y = position.getY();
            error = Utils.distance(x, y, targetX, targetY);
            angleError = Math.atan2(targetY - y, targetX - x);

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

            double leftPower = distancePower - anglePower;
            double rightPower = distancePower + anglePower;

            robot.drive(leftPower, rightPower, leftPower, rightPower, configuration.getMaxSpeed());

            lastAngleError = angleError;
            lastError = error;
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
