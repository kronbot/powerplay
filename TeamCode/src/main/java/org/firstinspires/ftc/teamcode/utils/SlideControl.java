package org.firstinspires.ftc.teamcode.utils;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.KronBot;

public class SlideControl {
    private final KronBot robot;
    private final Telemetry telemetry;

    public SlideControl(KronBot robot, Telemetry telemetry) {
        this.robot = robot;
        this.telemetry = telemetry;
    }

    public void slide(boolean up, boolean down) {
        if (up)
            robot.controlSlide(0.5);
        else if (down)
            robot.controlSlide(-0.5);
        else
            robot.controlSlide(0.05);
    }

    public void intake(boolean open, boolean close) {
        if (open)
            robot.controlIntake(0);
        else if (close)
            robot.controlIntake(1);
    }
}
