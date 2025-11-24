package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name="Auto1")
public class Auto1 extends LinearOpMode
{
    //#region Variables and Constants
    // Hardware map variables
    private DcMotor frontleft = null;
    private DcMotor frontright = null;
    private DcMotor rearleft = null;
    private DcMotor rearright = null;
    private DcMotor intake = null;
    private CRServo left = null;
    private CRServo middle = null;
    private CRServo right = null;

    // Logger setup
    private Datalogger datalog;
    private Datalogger.GenericField df1, df2, df3, df4, df5, df6, df7, df8, df9, df16, df17, df18, df19;


    //#endregion

    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer

    @Override public void runOpMode()
    {
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        odo.setOffsets(-83, -133, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();


        initializeHardware();
        waitForStart();

        if (opModeIsActive())
        {
            odo.update();
            telemetry.update();

            //*****************************************************************/

            driveToTarget(1679,0,0);

        }

    }



public void driveToTarget(double targetXmm, double targetYmm, double targetHeadingDeg) {
    // Function to DRIVE to target position
    // =====================================================
    // PID-BASED DRIVE TO TARGET FUNCTION (X, Y, HEADING)
    // Units: X, Y in millimeters | Heading in degrees
    // =====================================================
    // ==============================
    // Tunable PID Drive To Target (MM + Heading)


    // --- Tunable PID constants ---
    double kPTrans = 0.05;
    double kITrans = 0.0;
    double kDTrans = 0.0;

    double kPRot = 0.08;
    double kIRot = 0.0;
    double kDRot = 0.005;

    // --- Control limits ---
    double maxPower = 1;
    double minPower = 0.1;
    double maxRotPower = 1;
    double minRotPower = 0.05;

    double allowableErrorMM = 15.0;
    double allowableErrorDeg = 2.0;

    // --- PID state ---
    double prevErrorX = 0, prevErrorY = 0, prevErrorH = 0;
    double integralX = 0, integralY = 0, integralH = 0;
    double prevTime = getRuntime();

    while (opModeIsActive()) {
        odo.update();
        Pose2D pos = odo.getPosition();

        double currentX = pos.getX(DistanceUnit.MM);
        double currentY = pos.getY(DistanceUnit.MM);

        // ✅ FIXED: Invert heading to make clockwise positive
        double currentHeading = -pos.getHeading(AngleUnit.DEGREES);

        // --- Compute errors ---
        double errorX = targetXmm - currentX;
        double errorY = targetYmm - currentY;
        double errorH = targetHeadingDeg - currentHeading;


        // ✅ Properly wrap heading error (-180° to +180°)
        errorH = (errorH + 540) % 360 - 180;


        double distanceMM = Math.hypot(errorX, errorY);
        if (distanceMM < allowableErrorMM && Math.abs(errorH) < allowableErrorDeg) break;

        // --- Time delta ---
        double currentTime = getRuntime();
        double dt = currentTime - prevTime;
        prevTime = currentTime;

        // --- PID Translation ---
        integralX += errorX * dt;
        integralY += errorY * dt;
        double derivativeX = (errorX - prevErrorX) / dt;
        double derivativeY = (errorY - prevErrorY) / dt;
        prevErrorX = errorX;
        prevErrorY = errorY;

        double powerX = kPTrans * errorX + kITrans * integralX + kDTrans * derivativeX;
        double powerY = kPTrans * errorY + kITrans * integralY + kDTrans * derivativeY;

        // --- PID Rotation ---
        integralH += errorH * dt;
        double derivativeH = (errorH - prevErrorH) / dt;
        prevErrorH = errorH;

        double powerH = kPRot * errorH + kIRot * integralH + kDRot * derivativeH;

        // ✅ Clamp minimum rotation so it doesn’t give up turning
        if (Math.abs(powerH) < minRotPower && Math.abs(errorH) > allowableErrorDeg) {
            powerH = Math.signum(powerH) * minRotPower;
        }
        powerH = Range.clip(powerH, -maxRotPower, maxRotPower);

        // --- FIX: Swap X/Y and preserve CW heading positive ---
        double botHeadingRad = Math.toRadians(currentHeading);

        // swap powerX/powerY positions compared to before
        double rotX = powerY * Math.cos(botHeadingRad) - powerX * Math.sin(botHeadingRad);
        double rotY = powerY * Math.sin(botHeadingRad) + powerX * Math.cos(botHeadingRad);


        // --- Wheel power math ---
        double frontLeftPower  = rotY + rotX + powerH;
        double rearLeftPower   = rotY - rotX + powerH;
        double frontRightPower = rotY - rotX - powerH;
        double rearRightPower  = rotY + rotX - powerH;

        // Normalize
        double max = Math.max(1.0, Math.max(Math.abs(frontLeftPower),
                Math.max(Math.abs(frontRightPower),
                        Math.max(Math.abs(rearLeftPower), Math.abs(rearRightPower)))));
        frontLeftPower  = Range.clip(frontLeftPower / max,  -maxPower, maxPower);
        frontRightPower = Range.clip(frontRightPower / max, -maxPower, maxPower);
        rearLeftPower   = Range.clip(rearLeftPower / max,   -maxPower, maxPower);
        rearRightPower  = Range.clip(rearRightPower / max,  -maxPower, maxPower);

        // --- Apply powers ---
        frontleft.setPower(frontLeftPower);
        frontright.setPower(frontRightPower);
        rearleft.setPower(rearLeftPower);
        rearright.setPower(rearRightPower);

        // --- Debugging output ---
        telemetry.addData("Target", "(%.1f, %.1f, %.1f°)", targetXmm, targetYmm, targetHeadingDeg);
        telemetry.addData("Current", "(%.1f, %.1f, %.1f°)", currentX, currentY, currentHeading);
        telemetry.addData("Error", "X: %.1f  Y: %.1f  H: %.1f°", errorX, errorY, errorH);
        telemetry.addData("Power", "FL: %.2f FR: %.2f RL: %.2f RR: %.2f", frontLeftPower, frontRightPower, rearLeftPower, rearRightPower);
        telemetry.update();

        df1.set(targetXmm);
        df2.set(targetYmm);
        df3.set(targetHeadingDeg);
        df4.set(currentX);
        df5.set(currentY);
        df6.set(currentHeading);
        df7.set(errorX);
        df8.set(errorY);
        df9.set(errorH);
        df16.set(frontLeftPower);
        df17.set(frontRightPower);
        df18.set(rearLeftPower);
        df19.set(rearRightPower);

        datalog.writeLine();
    }

    // Stop all motors
    frontleft.setPower(0);
    frontright.setPower(0);
    rearleft.setPower(0);
    rearright.setPower(0);
}




    // Initialize hardware and setup datalogger
    private void initializeHardware() {
        // Setup datalogger fields
        df1 = new Datalogger.GenericField("Target X (mm)");           // Target X position in meters
        df2 = new Datalogger.GenericField("Target Y (mm)");           // Target Y position in meters
        df3 = new Datalogger.GenericField("Target Heading (deg)");   // Target heading in degrees
        df4 = new Datalogger.GenericField("Current X (mm)");           // Current X position in meters
        df5 = new Datalogger.GenericField("Current Y (mm)");           // Current Y position in meters
        df6 = new Datalogger.GenericField("Current Heading (deg)");  // Current heading in degrees
        df7 = new Datalogger.GenericField("Error X (mm)");   // Global X error in meters
        df8 = new Datalogger.GenericField("Error Y (mm)");   // Global Y error in meters
        df9 = new Datalogger.GenericField("Error H (mm)"); // Distance to target in meters
        df16 = new Datalogger.GenericField("Front Left Power");       // Front left motor power
        df17 = new Datalogger.GenericField("Front Right Power");       // Front right motor power
        df18 = new Datalogger.GenericField("Rear Left Power");       // Rear left motor power
        df19 = new Datalogger.GenericField("Rear Right Power");       // Rear right motor power

        // Create datalogger
        datalog = new Datalogger.Builder()
                .setFilename("PID_Datalog")  // Name your file
                .setFields(df1, df2, df3, df4, df5, df6, df7, df8, df9, df16, df17, df18, df19)
                .setAutoTimestamp(Datalogger.AutoTimestamp.DECIMAL_SECONDS)
                .build();

        // Initialize the hardware variables and set the direction and zero power behavior
        frontleft = hardwareMap.get(DcMotor.class, "frontleft");  // Control Hub - Motors - 2 - REV Robotics UltraPlanetary HD Hex Motor
        frontright = hardwareMap.get(DcMotor.class, "frontright"); // Control Hub - Motors - 1 - REV Robotics UltraPlanetary HD Hex Motor
        rearleft = hardwareMap.get(DcMotor.class, "rearleft");  // Control Hub - Motors - 3 - REV Robotics UltraPlanetary HD Hex Motor
        rearright = hardwareMap.get(DcMotor.class, "rearright");  // Control Hub - Motors - 0 - REV Robotics UltraPlanetary HD Hex Motor
        intake = hardwareMap.get(DcMotor.class, "intake");
        left = hardwareMap.get(CRServo.class, "left");
        middle = hardwareMap.get(CRServo.class, "middle");
        right = hardwareMap.get(CRServo.class, "right");

        frontleft.setDirection(DcMotor.Direction.REVERSE);
        frontright.setDirection(DcMotor.Direction.FORWARD);
        rearleft.setDirection(DcMotor.Direction.REVERSE);
        rearright.setDirection(DcMotor.Direction.FORWARD);

        frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset encoders

        // Initialize ODOS
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(0,0,DistanceUnit.METER);
        odo.resetPosAndIMU();

        telemetry.addData("AUTO v9 CLIP - Ready team!", "Press Play Button");
        telemetry.update();

    }
}

