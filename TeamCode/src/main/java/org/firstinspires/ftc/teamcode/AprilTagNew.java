package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import android.util.Size;
import java.util.List;
//
@TeleOp(name = "AprilTagNew", group = "Concept")
public class AprilTagNew extends LinearOpMode {

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private WebcamName webcam1, webcam2;

    @Override
    public void runOpMode() {
        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");  // Logitech
        webcam2 = hardwareMap.get(WebcamName.class, "Webcam 2");  // ArduCam

        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .build();

        telemetry.addLine("Ready — press start to begin switching.");
        telemetry.update();
        waitForStart();

        // Start with Webcam 1
        openPortal(webcam1);
        sleep(500); // give it a moment to initialize

        while (opModeIsActive()) {
            showDetections("Webcam 1", aprilTag.getDetections());
            telemetry.update();
            sleep(1000);

            // Switch to Webcam 2
            switchCamera(webcam2);
            sleep(500);
            showDetections("Webcam 2", aprilTag.getDetections());
            telemetry.update();
            sleep(1000);

            // Switch back to Webcam 1
            switchCamera(webcam1);
            sleep(500);
        }

        visionPortal.close();
    }

    private void openPortal(WebcamName cam) {
        visionPortal = new VisionPortal.Builder()
                .setCamera(cam)
                .setCameraResolution(new Size(640, 480))
                .addProcessor(aprilTag)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();
    }

    private void switchCamera(WebcamName cam) {
        visionPortal.close();
        openPortal(cam);
    }

    private void showDetections(String label, List<AprilTagDetection> detections) {
        telemetry.addLine("---- " + label + " ----");
        telemetry.addData("Detections", detections.size());
        for (AprilTagDetection d : detections) {
            telemetry.addLine(String.format("ID %d  (%.1f, %.1f, %.1f)", d.id, d.ftcPose.x, d.ftcPose.y, d.ftcPose.z));
        }
    }
}
