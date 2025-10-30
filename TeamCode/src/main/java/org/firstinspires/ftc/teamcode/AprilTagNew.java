package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;

import java.util.List;

@TeleOp(name = "AprilTagNew", group = "Vision")
public class AprilTagNew extends LinearOpMode {

    private VisionPortal portal1;
    private VisionPortal portal2;

    private AprilTagProcessor tagProcessor1;
    private AprilTagProcessor tagProcessor2;

    @Override
    public void runOpMode() {

        // --- Initialize both cameras with completely independent objects ---
        tagProcessor1 = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .build();

        tagProcessor2 = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .build();

        portal1 = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(tagProcessor1)
                .build();

        portal2 = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"))
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(tagProcessor2)
                .build();

        telemetry.addLine(">>> Cameras initialized. Press START <<<");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            displayCameraData(tagProcessor1, "Camera 1 (ArduCam)");
            displayCameraData(tagProcessor2, "Camera 2 (Logitech)");
            telemetry.update();
            sleep(50);
        }

        // --- Close both portals on stop ---
        if (portal1 != null) portal1.close();
        if (portal2 != null) portal2.close();
    }

    private void displayCameraData(AprilTagProcessor processor, String label) {
        List<AprilTagDetection> detections = processor.getDetections();

        telemetry.addLine("==============");
        telemetry.addLine(label);
        telemetry.addData("Detections", detections.size());

        if (detections.isEmpty()) {
            telemetry.addLine("No AprilTags detected.");
            return;
        }

        for (AprilTagDetection tag : detections) {
            if (tag.metadata != null) {
                telemetry.addLine(String.format("ID %d: %s", tag.id, tag.metadata.name));
                telemetry.addLine(String.format("XYZ (in): %.1f, %.1f, %.1f",
                        tag.ftcPose.x, tag.ftcPose.y, tag.ftcPose.z));
                telemetry.addLine(String.format("Yaw/Pitch/Roll (deg): %.1f, %.1f, %.1f",
                        tag.ftcPose.yaw, tag.ftcPose.pitch, tag.ftcPose.roll));
            } else {
                telemetry.addLine(String.format("ID %d (Unknown tag)", tag.id));
                telemetry.addLine(String.format("Center: %.0f, %.0f px",
                        tag.center.x, tag.center.y));
            }
        }
    }
}
