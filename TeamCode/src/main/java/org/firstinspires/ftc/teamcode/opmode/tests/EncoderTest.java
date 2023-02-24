package org.firstinspires.ftc.teamcode.opmode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.KronBot;
import org.firstinspires.ftc.teamcode.util.Encoder;

@TeleOp(name = "Encoder Test", group = "Tests") // BUCI (Bipedal Unit for Control and Interaction)
public class EncoderTest extends OpMode {
    private final KronBot robot;

    private Encoder leftEncoder;
    private Encoder rightEncoder;
    private Encoder frontEncoder;

    public EncoderTest() {
        robot = new KronBot();
    }

    @Override
    public void init() {
        robot.initHardwareMap(hardwareMap);

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftEncoder"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightEncoder"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "frontEncoder"));

        frontEncoder.setDirection(Encoder.Direction.REVERSE);
    }

    @Override
    public void loop() {
        telemetry.addData("leftEncoder", leftEncoder.getCurrentPosition());
        telemetry.addData("rightEncoder", rightEncoder.getCurrentPosition());
        telemetry.addData("frontEncoder", frontEncoder.getCurrentPosition());
    }
}
