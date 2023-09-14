package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.KronBot;

public class ArmControl {
    private final KronBot robot;
    private final Telemetry telemetry;

    private boolean intakeActive = false;

    private Integer minCoordinate = 0;
    private Integer maxCoordinate = 4500;

    private static final double maxPower = Utils.ARM_MAX_POWER;
    private static final double minPower = Utils.ARM_MIN_POWER;
    private static final double restPower = Utils.ARM_REST_POWER;

    private boolean stop;

    private boolean enable = false;

    public ArmControl(KronBot robot, Telemetry telemetry) {
        this.robot = robot;
        this.telemetry = telemetry;
    }

    public void showDebugTelemetry() {
        telemetry.addData("Arm busy", robot.armDc.isBusy());
        telemetry.addData("Arm coordinate", -robot.armDc.getCurrentPosition());
        telemetry.addData("Arm target coordinate", robot.armDc.getTargetPosition());
        telemetry.addData("Intake position", robot.clawPosition());
    }

    public void control(Gamepad gamepad, boolean debug) {
        if (debug)
            showDebugTelemetry();
        loop(gamepad);
    }

    public void control(Gamepad gamepad) {
        loop(gamepad);
    }

    public void claw(boolean action) {
        if (action && intakeActive) {
            if (Double.compare(robot.clawPosition(), 1.0) == 0)
                robot.controlClaw(0);
            else if (Double.compare(robot.clawPosition(), 0.0) == 0)
                robot.controlClaw(1);
            intakeActive = false;
        } else if (!action)
            intakeActive = true;
    }

    boolean pressed = false;

    public void enableAutoServo(boolean action) {
        if (action && !pressed) {
            enable = !enable;
            pressed = true;
        } else
            pressed = false;
    }

    public void intake() {
        if (-robot.armDc.getCurrentPosition() < Utils.SERVO_COORDINATES)
            robot.controlIntake(1);
        else {
            robot.controlIntake(Utils.map(-robot.armDc.getCurrentPosition(),
                    Utils.ARM_REVERSE_COORDINATES, Utils.ARM_MAX_COORDINATES, Utils.SERVO_START, Utils.SERVO_END / 2) / Utils.SERVO_CONSTANT);
        }
    }

    private double getMaxPower(double power) {
        return Utils.map(power, Utils.ARM_REVERSE_COORDINATES, Utils.ARM_MAX_COORDINATES, maxPower / 3, -Utils.ARM_REST_POWER / 2);
    }

    private double getMinPower(double power) {
        return Utils.map(power, Utils.ARM_REVERSE_COORDINATES, Utils.ARM_MIN_COORDINATES, maxPower / Utils.ARM_CUT_SPEED, -Utils.ARM_REST_POWER / 3);
    }

    public void loop(Gamepad gamepad) {
        if (enable)
            intake();
        if (gamepad.right_trigger > Utils.EPS && -robot.armDc.getCurrentPosition() < Utils.ARM_MAX_COORDINATES) {
            if (-robot.armDc.getCurrentPosition() < Utils.ARM_REVERSE_COORDINATES)
                robot.controlArm(-Utils.map(gamepad.right_trigger, 0, 1, minPower, maxPower));
            else
                robot.controlArm(-getMaxPower(-robot.armDc.getCurrentPosition()) * gamepad.right_trigger);

        } else if (gamepad.left_trigger > Utils.EPS && -robot.armDc.getCurrentPosition() > Utils.ARM_MIN_COORDINATES) {
            if (-robot.armDc.getCurrentPosition() > Utils.ARM_REVERSE_COORDINATES)
                robot.controlArm(Utils.map(gamepad.left_trigger, 0, 1, minPower, maxPower));
            else
                robot.controlArm(getMinPower(-robot.armDc.getCurrentPosition()) * gamepad.left_trigger);
        } else {
            if (-robot.armDc.getCurrentPosition() < Utils.ARM_REVERSE_COORDINATES - Utils.ARM_BLIND_SPOT)
                robot.controlArm(-Utils.ARM_REST_POWER);
            else if (-robot.armDc.getCurrentPosition() > Utils.ARM_REVERSE_COORDINATES + Utils.ARM_BLIND_SPOT)
                robot.controlArm(Utils.ARM_REST_POWER);
            else
                robot.controlArm(0);
        }
    }

    public void throwPlane(boolean action) {
        if (action)
            robot.throwServo.setPosition(1);
    }

    public void moveLeft(boolean action) {
        if (action && robot.intakeServo.getPosition() < 0.99)
            robot.intakeServo.setPosition(robot.intakeServo.getPosition() + 0.01);
    }

    public void moveRight(boolean action) {
        if (action && robot.intakeServo.getPosition() > 0.01)
            robot.intakeServo.setPosition(robot.intakeServo.getPosition() - 0.01);
    }
}
