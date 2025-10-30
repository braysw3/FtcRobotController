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

    // One AprilTag processor per camera
    private AprilTagProcessor aprilTag1;
    private AprilTagProcessor aprilTag2;

    // One VisionPortal per camera
    private VisionPortal visionPortal1;
    private VisionPortal visionPortal2;

    private String botSide = "";
    private double minXValue = 0;
    private int minXID = 0;

    @Override
    public void runOpMode() {

        initAprilTagDual();

        telemetry.addLine(">>> Press START to begin detecting AprilTags <<<");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            telemetryAprilTagDual();
            telemetry.update();
            sleep(50);
        }

        // Close both cameras to save CPU
        visionPortal1.close();
        visionPortal2.close();
    }

    /** Initialize both webcams and AprilTag processors. */
    private void initAprilTagDual() {

        // Processor for Webcam 1 (ArduCam)
        aprilTag1 = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .build();

        // Processor for Webcam 2 (Logitech)
        aprilTag2 = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .build();

        // Build VisionPortal for Webcam 1
        visionPortal1 = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG) // 👈 ensures MJPEG
                .addProcessor(aprilTag1)
                .build();

        // Build VisionPortal for Webcam 2
        visionPortal2 = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"))
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(aprilTag2)
                .build();

        telemetry.setMsTransmissionInterval(100);
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
    }

    /** Display AprilTag detections from both cameras. */
    private void telemetryAprilTagDual() {

        List<AprilTagDetection> detections1 = aprilTag1.getDetections();
        List<AprilTagDetection> detections2 = aprilTag2.getDetections();

        telemetry.addLine("=== CAMERA 1 (ArduCam) ===");
        telemetry.addData("Tags Detected", detections1.size());
        showDetections(detections1);

        telemetry.addLine();
        telemetry.addLine("=== CAMERA 2 (Logitech) ===");
        telemetry.addData("Tags Detected", detections2.size());
        showDetections(detections2);
    }

    /** Helper: print detection info for a single camera. */
    private void showDetections(List<AprilTagDetection> detections) {

        for (AprilTagDetection detection : detections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format(
                        "\n(ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format(
                        "XYZ: %6.1f %6.1f %6.1f in",
                        detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format(
                        "PRY: %6.1f %6.1f %6.1f deg",
                        detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
            } else {
                telemetry.addLine(String.format(
                        "\n(ID %d) Unknown", detection.id));
                telemetry.addLine(String.format(
                        "Center: %6.0f %6.0f px",
                        detection.center.x, detection.center.y));
            }
        }

        if (detections.isEmpty()) {
            telemetry.addLine("No tags detected.");
        }
    }
}
