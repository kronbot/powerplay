package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.Encoder;

@TeleOp(name = "buci")
public class buci extends OpMode {
    private Encoder leftEncoder;
    private Encoder rightEncoder;
    private Encoder frontEncoder;

    @Override
    public void init() {
        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "backLeft"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "frontRight"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "backRight"));
    }

    @Override
    public void loop() {
        telemetry.addData("leftEncoder", leftEncoder.getCurrentPosition());
        telemetry.addData("rightEncoder", rightEncoder.getCurrentPosition());
        telemetry.addData("frontEncoder", frontEncoder.getCurrentPosition());

    }
}
