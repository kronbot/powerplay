package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.Encoder;

@TeleOp(name = "buci")
public class buci extends OpMode {
    private final KronBot robot;

    private Encoder leftEncoder;
    private Encoder rightEncoder;
    private Encoder frontEncoder;

    public buci() {
        robot = new KronBot();
    }

    @Override
    public void init() {
        robot.initHardwareMap(hardwareMap);

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "frontLeft"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "backLeft"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "frontRight"));

        robot.resetEncoders();

        frontEncoder.setDirection(Encoder.Direction.REVERSE);


    }

    @Override
    public void loop() {
        telemetry.addData("leftEncoder", leftEncoder.getCurrentPosition());
        telemetry.addData("rightEncoder", rightEncoder.getCurrentPosition());
        telemetry.addData("frontEncoder", frontEncoder.getCurrentPosition());

    }
}