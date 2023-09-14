package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.Encoder;

public class KronBot {
    public DcMotor frontLeftDc;
    public DcMotor frontRightDc;
    public DcMotor backLeftDc;
    public DcMotor backRightDc;

    public DcMotor armDc;
    public Servo clawServo;
    public Servo intakeServo;

    public Servo throwServo;

    public WebcamName webcam;

    public void initHardwareMap(HardwareMap hardwareMap) {
        frontLeftDc = hardwareMap.dcMotor.get("frontLeft");
        frontRightDc = hardwareMap.dcMotor.get("frontRight");
        backLeftDc = hardwareMap.dcMotor.get("backLeft");
        backRightDc = hardwareMap.dcMotor.get("backRight");

        frontLeftDc.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftDc.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftDc.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDc.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDc.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDc.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armDc = hardwareMap.dcMotor.get("arm");
        armDc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armDc.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        clawServo = hardwareMap.servo.get("claw");
        intakeServo = hardwareMap.servo.get("intake");
        throwServo = hardwareMap.servo.get("throw");

        webcam = hardwareMap.get(WebcamName.class, "Webcam 1");

        intakeServo.setPosition(1);
        throwServo.setPosition(0);
    }

    public void drive(double frontLeft, double frontRight, double backLeft, double backRight, double power) {
        frontLeftDc.setPower(frontLeft * power);
        frontRightDc.setPower(frontRight * power);
        backLeftDc.setPower(backLeft * power);
        backRightDc.setPower(backRight * power);
    }

    public void stopMotors() {
        frontLeftDc.setPower(0);
        frontRightDc.setPower(0);
        backLeftDc.setPower(0);
        backRightDc.setPower(0);
    }

    public void controlArm(double power) {
        armDc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armDc.setPower(power);
    }

    public void controlArm(double power, int position) {
        armDc.setTargetPosition(position);
        armDc.setPower(power);
        if (armDc.getCurrentPosition() > position)
            armDc.setPower(-power);
        armDc.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void resetArmEncoder() {
        armDc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void controlClaw(double power) {
        clawServo.setPosition(power);
    }

    public double clawPosition() {
        return clawServo.getPosition();
    }

    public void controlIntake(double power) {
        intakeServo.setPosition(power);
    }

    public double intakePosition() {
        return intakeServo.getPosition();
    }
}

