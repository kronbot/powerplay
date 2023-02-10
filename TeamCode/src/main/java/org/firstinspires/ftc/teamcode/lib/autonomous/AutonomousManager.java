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
    private final RobotControl robotControl;

    private enum State {
        LINEAR,
        TRANSLATE,
        ROTATE,
        WAIT
    }

    private State currentState = State.WAIT;
    private double lastX = 0, lastY = 0, lastAngle = 0;
    private double targetDirectionX, targetDirectionY, targetAngle;
    private double targetX, targetY;

    // for PID
    private final ElapsedTime timer = new ElapsedTime();
    private double integralSum = 0;
    private double lastError = 0;

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
        this.robotControl = new RobotControl(robot, null);
    }

    public void initalize() {
        Thread positionThread = new Thread(position);
        positionThread.start();
    }

    private void driveDirection(double x, double y, double power) {
        // normalizing the vector and applying to force
        double mag = Math.sqrt(x * x + y * y);
        x = x / mag * power;
        y = y / mag * power;
        robotControl.drive(x, y);
    }

    public void update() {
        if (currentState == State.WAIT)
            return;

        double x = position.getX(), y = position.getY(), angle = position.getAngle();
        double error = Utils.distance(x, y, targetX, targetY);
        if (error < configuration.getAcceptableError()) {
            setState(State.WAIT);
        }

        double derivative = (error - lastError) / timer.seconds();
        integralSum += error * timer.seconds();
        double power = (configuration.getKp() * error) + (configuration.getKi() * integralSum) + (configuration.getKd() * derivative);
        driveDirection(targetDirectionX, targetDirectionY, power);

        lastX = x;
        lastY = y;
        lastAngle = angle;
    }

    public void linear(double distance) {
        if (currentState != State.WAIT)
            return;
        double x = position.getX(),
                y = position.getY(),
                angle = position.getAngle();
        targetDirectionX = Math.cos(angle);
        targetDirectionY = Math.sin(angle);
        targetAngle = Math.atan2(targetDirectionY, targetDirectionX);
        targetX = x + targetDirectionX * distance;
        targetY = y + targetDirectionY * distance;

        integralSum = 0;
        lastError = 0;
        setState(State.LINEAR);
        timer.reset();
    }

    public void translate(double distance) {
        if (currentState != State.WAIT)
            return;
        setState(State.TRANSLATE);
    }

    public void rotate(double radians) {
        if (currentState != State.WAIT)
            return;
        setState(State.ROTATE);
    }

    public void setState(State newState) {
        currentState = newState;
        lastX = position.getX();
        lastY = position.getY();
        lastAngle = position.getAngle();
    }
}
