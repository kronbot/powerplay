package org.firstinspires.ftc.teamcode.autonomous;

import org.firstinspires.ftc.teamcode.KronBot;

// some IoC might be nice...
public class AutonomousManager {
    private final AutonomousConfiguration configuration;
    private final KronBot robot;

    private final GlobalCoordinatePosition position;
    private final PIDController controller;

    private enum State {
        LINEAR,
        TRANSLATE,
        ROTATE,
        WAIT
    }

    private State currentState = State.WAIT;
    private double current = 0;
    private double lastX = 0, lastY = 0, lastAngle = 0;

    public AutonomousManager(
            AutonomousConfiguration configuration,
            KronBot robot
    ) {
        this.configuration = configuration;
        this.robot = robot;
        this.position = new GlobalCoordinatePosition(
                robot.leftEncoder,
                robot.rightEncoder,
                robot.frontEncoder,
                configuration
        );
        this.controller = new PIDControllerImpl(configuration);
    }

    public void initalize() {
        Thread positionThread = new Thread(position);
        positionThread.start();
    }

    public void update() {
        if (currentState == State.WAIT)
            return;

        if (Math.abs(controller.getTarget() - current) <= configuration.getAcceptableError()) {
            currentState = State.WAIT;
            return;
        }

        double x = position.getX(), y = position.getY(), angle = position.getAngle();
        if (currentState == State.ROTATE)
            current += Math.abs(angle - lastAngle);
        else
            current += Math.hypot(x - lastX, y - lastY);

        double speed = (controller.getTarget() > 0 ? 1 : -1) * controller.getSpeed(current);
        switch (currentState) {
            case LINEAR:
                robot.drive(1, 1, -1, -1, speed);
                break;
            case TRANSLATE:
                robot.drive(1, -1, -1, 1, speed);
                break;
            case ROTATE:
                robot.drive(1, -1, 1, -1, speed);
                break;
        }

        lastX = x;
        lastY = y;
        lastAngle = angle;
    }

    public void linear(double distance) {
        if (currentState != State.WAIT)
            return;
        controller.setTarget(distance);
        setState(State.LINEAR);
    }

    public void translate(double distance) {
        if (currentState != State.WAIT)
            return;
        controller.setTarget(distance);
        setState(State.TRANSLATE);
    }

    public void rotate(double radians) {
        if (currentState != State.WAIT)
            return;
        controller.setTarget(radians);
        setState(State.ROTATE);
    }

    public void setState(State newState) {
        currentState = newState;
        current = 0;
        lastX = position.getX();
        lastY = position.getY();
        lastAngle = position.getAngle();
    }
}
