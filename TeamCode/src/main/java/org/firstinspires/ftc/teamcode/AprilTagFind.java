

package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import android.graphics.Color; // For converting RGBA to HSV
import java.util.List;

@TeleOp(name = "AprilTagFind", group = "Concept")

public class AprilTagFind extends LinearOpMode {
    String BotSide = "";
    double minXValue=0;
    int minXID=0;
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera


    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;
    private AprilTagProcessor aprilTag2;
    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;
    private VisionPortal visionPortal2;
    @Override
    public void runOpMode() {

        initAprilTag();

        int i = 0;
        while (i<100) {
           telemetryAprilTag();
            telemetry.addLine(String.format("\n min (ID %d)",minXID));
            telemetry.addLine(String.format("min %6.0f", minXValue));
            telemetry.update();
           i = i + 1;
           sleep(10);
        }
        int b = 0;
        while (b<100) {
            telemetryAprilTag();
            telemetry.addLine(String.format("\n min (ID %d)",minXID));
            telemetry.addLine(String.format("min %6.0f", minXValue));
            telemetry.update();
            b = b + 1;
            sleep(10);
        }

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                telemetry.update();

            }
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

    }   // end method runOpMode()

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .build();

        aprilTag2 = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                 .build();


        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(1280, 800))
                .addProcessor(aprilTag)
                .build();

        visionPortal2 = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"))
                .setCameraResolution(new Size(1280, 720))
                .addProcessor(aprilTag2)
                .build();


        telemetry.setMsTransmissionInterval(100);  // Speed up telemetry updates, for debugging.
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()


    /**
     * Add telemetry about AprilTag detections.
     */
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        minXValue=0;
        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
                //     telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                //telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
               // telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
               // telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            }

           // sleep(6000);




//            switch (detection.id) {
//                case 23:
//                    telemetry.addLine("PPG");
//
//                    break;
//                case 22:
//                    telemetry.addLine("PGP");
//
//                    break;
//                case 21:
//                    telemetry.addLine("GPP");
//
//                    break;
//                case 24:
//                    telemetry.addLine("Red");
//
//                    telemetry.addLine("you are on blue side");
//                    break;
//                case 20:
//                    telemetry.addLine("Blue");
//                    //id20Detected = true;
//                    telemetry.addLine("you are on red side");
//                    break;
//                default:
//                    telemetry.addLine("unknown");
//                    break;
//                }

            if (detection.id==24) {
                BotSide="blue";
            } else if (detection.id==20) {
                BotSide="red";
            }

            if (BotSide!="" && (detection.id==21 || detection.id==22 || detection.id==23)) {
                if (minXValue == 0) {
                    minXID = detection.id;
                    minXValue = detection.center.x;
                } else if (BotSide == "red" && minXValue > detection.center.x) {
                    minXID = detection.id;
                    minXValue = detection.center.x;
                } else if (BotSide == "blue" && minXValue < detection.center.x) {
                    minXID = detection.id;
                    minXValue = detection.center.x;
                }



            }
        }
    }
  }
