package org.firstinspires.ftc.teamcode.lib;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.KronBot;

import java.util.List;

public class CameraControl {
    private static final String VUFORIA_KEY = "ARfXSg3/////AAABmeAedbcQY08Hq6RpQG93FA8zpqyJCYfLEr4REOzdT2VrPv9iQDD2dR7I2Mj+Q0u1V+nd/binuIaCYlNosxD+UW1F4qOKkG8LtlXIvF0pwVHMbg8pm3apX3RyOWdZEk+Jx9Dnsv6cdIehvgdkNZGleEEIcUxBsO0WXS/pUPrdu/xfEmw61qtKcmrnaRQ+uzTCyhcp9G24swdYg9R6k7OAw93N+DCbYqcib+mD3smZcVnZn7nQDYv0MWJCsGYr5bsAvrH/SMPz7BWeErSGZZkCYFIKkZcrCvS2OHOqQ7Fs4k6cg/mgx8kVNS09hFlR7OX8kjclwFodb/j6Az+R0Q7jMaKudDfT3a9UOQMyw2U7oGea";

    private static final String TFOD_MODEL_ASSET = "model.tflite";

    private static final String[] LABELS = {
            "TeamElement"
    };

    private final KronBot robot;
    private final Telemetry telemetry;

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    public CameraControl(KronBot robot, Telemetry telemetry) {
        this.robot = robot;
        this.telemetry = telemetry;

        init();

        if (tfod != null)
            tfod.activate();
    }

    private void init() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = robot.webcam;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromFile(TFOD_MODEL_ASSET, LABELS);
    }

    public int detect() {
        if (tfod != null) {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null && !updatedRecognitions.isEmpty()) {
                Recognition recognition = updatedRecognitions.get(0);
                if (recognition.getRight() + recognition.getLeft() < 500)
                    return 1;
                return 2;
            } else
                return 3;
        }
        return 0;
    }
}
