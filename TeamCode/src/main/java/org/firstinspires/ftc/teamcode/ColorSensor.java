package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;
import android.util.Size;
import com.qualcomm.hardware.rev.RevColorSensorV3;

@TeleOp (name = "ColorSensor", group = "Concept")
public class ColorSensor extends LinearOpMode {

    @Override
    public void runOpMode(){

        PredominantColorProcessor colorSensor = new PredominantColorProcessor.Builder()
                .setRoi(ImageRegion.entireFrame())
                .setSwatches(
                        PredominantColorProcessor.Swatch.ARTIFACT_GREEN,
                        PredominantColorProcessor.Swatch.ARTIFACT_PURPLE)
                .build();

        RevColorSensorV3 color = hardwareMap.get(RevColorSensorV3.class, "color");

        telemetry.setMsTransmissionInterval(100);  // Speed up telemetry updates, for debugging.
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        // WARNING:  To view the stream preview on the Driver Station, this code runs in INIT mode.
        while (opModeIsActive() || opModeInInit()){

            telemetry.addLine("Preview on/off: 3 dots, Camera Stream\n");

            PredominantColorProcessor.Result result = colorSensor.getAnalysis();

            //purple
            if (result.closestSwatch == PredominantColorProcessor.Swatch.ARTIFACT_PURPLE){
                telemetry.addLine("Purple");
            }

            //green
            if (result.closestSwatch == PredominantColorProcessor.Swatch.ARTIFACT_GREEN){
                telemetry.addLine("Green");
            }

            // Display the Color Sensor result.
            telemetry.addData("Best Match", result.closestSwatch);
            telemetry.addLine(String.format("RGB   (%3d, %3d, %3d)",
                    result.RGB[0], result.RGB[1], result.RGB[2]));
            telemetry.addLine(String.format("HSV   (%3d, %3d, %3d)",
                    result.HSV[0], result.HSV[1], result.HSV[2]));
            telemetry.addLine(String.format("YCrCb (%3d, %3d, %3d)",
                    result.YCrCb[0], result.YCrCb[1], result.YCrCb[2]));
            telemetry.update();

            sleep(20);

        }


    }



}
