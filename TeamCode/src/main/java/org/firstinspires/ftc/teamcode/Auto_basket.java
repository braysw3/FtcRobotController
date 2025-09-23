package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.Datalogger;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.util.Locale;

@Autonomous(name="Auto_basket")
//@TeleOp(name="Auto_basket")
public class Auto_basket extends LinearOpMode
{
    //#region Variables and Constants
    // Hardware map variables
    private DcMotor frontleft = null;
    private DcMotor frontright = null;
    private DcMotor rearleft = null;
    private DcMotor rearright = null;
    private DcMotor extentionarm = null;
    private DcMotor liftarm = null;
    private DcMotor liftarm2 = null;
    private Servo armpivot = null;
    private Servo upperpivot = null;
    private Servo lowerpivot = null;
    private Servo clawpivot = null;
    private CRServo gripl = null;
    private CRServo gripr = null;
    private SparkFunOTOS myOtos;
    private SparkFunOTOS.Pose2D currentPose;

    // Motor control variables
    private double drive = 0;
    private double strafe = 0;
    private double turn = 0;
    private double frontLeftPower = 0;
    private double frontRightPower = 0;
    private double rearLeftPower = 0;
    private double rearRightPower = 0;
    private double max = 0;
    private VoltageSensor voltageSensor;

    // Static friction (minimum power to move) for fine adjustment phase
    private static double kS_forward = 0.20;
    private static double kS_strafe = 0.30;
    private static double kS_turn = 0.20;

    // Acceptable tolerances
    private static double DISTANCE_FINEADJUST_M = 0.30; //0.3
    private static double DISTANCE_TOLERANCE_M = 0.01;

    private static double HEADING_FINEADJUST_DEG = 10.0;  //10.0
    private static double HEADING_TOLERANCE_DEG = 1.0;

    // Speeds
    private static double FAST_POWER_TRANSLATION = 1.0;   // Full power for driving
    private static double FAST_POWER_ROTATION = 0.8;   // Full power for turning

    // Logger setup
    private Datalogger datalog;
    private Datalogger.GenericField df1, df2, df3, df4, df5, df6, df7, df8, df9, df10, df11, df12, df13, df14, df15, df16, df17, df18, df19, df20, df21, df22, df23;

    private ElapsedTime runtime = new ElapsedTime();
    private double savedTime = 0;

    //#endregion

    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer

    double oldTime = 0;

    @Override public void runOpMode()
    {
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");

        /*
        Set the odometry pod positions relative to the point that the odometry computer tracks around.
        The X pod offset refers to how far sideways from the tracking point the
        X (forward) odometry pod is. Left of the center is a positive number,
        right of center is a negative number. the Y pod offset refers to how far forwards from
        the tracking point the Y (strafe) odometry pod is. forward of center is a positive number,
        backwards is a negative number.
         */
        odo.setOffsets(-84.0, -168.0, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1

        /*
        Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
        the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
        If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
        number of ticks per unit of your odometry pod.
         */
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        //odo.setEncoderResolution(13.26291192, DistanceUnit.MM);


        /*
        Set the direction that each of the two odometry pods count. The X (forward) pod should
        increase when you move the robot forward. And the Y (strafe) pod should increase when
        you move the robot to the left.
         */
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);


        /*
        Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
        The IMU will automatically calibrate when first powered on, but recalibrating before running
        the robot is a good idea to ensure that the calibration is "good".
        resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
        This is recommended before you run your autonomous, as a bad initial calibration can cause
        an incorrect starting value for x, y, and heading.
         */
        //odo.recalibrateIMU();
        odo.resetPosAndIMU();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", odo.getXOffset(DistanceUnit.MM));
        telemetry.addData("Y offset", odo.getYOffset(DistanceUnit.MM));
        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Heading Scalar", odo.getYawScalar());
        telemetry.update();




        initializeHardware();
        waitForStart();

        if (opModeIsActive())
        {
            myOtos.resetTracking();



            //*****************************************************************/
            long startTime = System.currentTimeMillis();

            //test
            driveToTarget(0,1,90,0,false);


//            // Drop Pre-load sample
//            liftArmToPosition(2400);
//            sleep(300);
//            lowerpivot.setPosition(0.1);
//            armpivot.setPosition(0.5);
//            upperpivot.setPosition(0.9);
//            sleep(1000);
//            lowerpivot.setPosition(0.55);
//            clawpivot.setPosition(0.5);
//            driveToTarget(0.1074,0.3101,-16,2,false); //move to bar
//            gripl.setPower(1);
//            gripr.setPower(-1);
//            sleep(500);
//            gripl.setPower(0);
//            gripr.setPower(0);
//
//            // Pick up 1st sample
//            armpivot.setPosition(0.2);
//            sleep(500);
//            liftArmToPosition(500);
//            armpivot.setPosition(0.5);
//            lowerpivot.setPosition(0.45);
//            clawpivot.setPosition(0.5);
//            upperpivot.setPosition(0.55);
//            driveToTarget(0.5550,0.3122,-90, 2.0, false);
//            sleep(100);
//            gripl.setPower(-1);
//            gripr.setPower(1);
//            sleep(200);
//            liftArmToPosition(10);
//            sleep(500);
//            gripl.setPower(0);
//            gripr.setPower(0);
//
//            // Drop 1st sample
//            liftArmToPosition(2400);
//            lowerpivot.setPosition(0.1);
//            armpivot.setPosition(0.5);
//            upperpivot.setPosition(0.9);
//            sleep(1000);
//            lowerpivot.setPosition(0.55);
//            clawpivot.setPosition(0.5);
//            driveToTarget(0.1074,0.3101,-15,2,false); //move to bar
//            sleep(500);
//            gripl.setPower(1);
//            gripr.setPower(-1);
//            sleep(500);
//            gripl.setPower(0);
//            gripr.setPower(0);
//
//            // Pick up 2nd sample
//            armpivot.setPosition(0.2);
//            sleep(500);
//            liftArmToPosition(500);
//            armpivot.setPosition(0.5);
//            lowerpivot.setPosition(0.45);
//            clawpivot.setPosition(0.5);
//            upperpivot.setPosition(0.55);
//            driveToTarget(0.6400,0.5801,-90, 2.7, false);
//            sleep(500);
//            gripl.setPower(-1);
//            gripr.setPower(1);
//            sleep(200);
//            liftArmToPosition(10);
//            sleep(500);
//            gripl.setPower(0);
//            gripr.setPower(0);
//
//            // Drop 2nd sample
//            liftArmToPosition(2400);
//            armpivot.setPosition(0.5);
//            upperpivot.setPosition(0.9);
//            sleep(1000);
//            lowerpivot.setPosition(0.9);
//            sleep(900);
//            lowerpivot.setPosition(0.55);
//            clawpivot.setPosition(0.5);
//            driveToTarget(0.1331,0.3281,-13,2,false); //move to bar
//            sleep(500);
//            gripl.setPower(1);
//            gripr.setPower(-1);
//            sleep(400);
//            gripl.setPower(0);
//            gripr.setPower(0);
//
//            //Pick up 3rd sample
//            armpivot.setPosition(0.2);
//            sleep(500);
//            liftArmToPosition(500);
//            armpivot.setPosition(0.5);
//            lowerpivot.setPosition(0.4);
//            clawpivot.setPosition(0.7);
//            upperpivot.setPosition(0.6);
//            driveToTarget(0.7200,0.5499,-33.9, 2.7, false);
//            extendArmToPosition(620);
//            sleep(200);
//            gripl.setPower(-1);
//            gripr.setPower(1);
//            sleep(200);
//            liftArmToPosition(-10);
//            sleep(500);
//            gripl.setPower(0);
//            gripr.setPower(0);
//
//            // Drop 3rd sample
//            extendArmToPosition(0);
//            liftArmToPosition(2400);
//            sleep(1200);
//            clawpivot.setPosition(0.5);
//            armpivot.setPosition(0.5);
//            upperpivot.setPosition(0.5);
//            sleep(100);
//            upperpivot.setPosition(0.9);
//            sleep(900);
//            lowerpivot.setPosition(0.9);
//            sleep(900);
//            lowerpivot.setPosition(0.55);
//            sleep(100);
//            driveToTarget(0.1074,0.3101,-23,2,false); //move to bar
//            sleep(500);
//            gripl.setPower(1);
//            gripr.setPower(-1);
//            sleep(400);
//            gripl.setPower(0);
//            gripr.setPower(0);
//            armpivot.setPosition(0.2);
//            sleep(300);
//            liftArmToPosition(500);
//
//            //Park
//            driveToTarget(currentPose.x, currentPose.y,-90,2,true);
            // armpivot.setPosition(0.2);
            // sleep(500);
            // liftArmToPosition(0);
            // armpivot.setPosition(0.8);
            // //sleep(400);
            // lowerpivot.setPosition(0.7);
            // //sleep(200);
            // clawpivot.setPosition(0.35);
            // //sleep(300);
            // upperpivot.setPosition(0.5);
            // //sleep(300);
            // driveToTarget(1.2354,0.0223,-90,8,true);
            // driveToTarget(currentPose.x, -0.1,-90,2,true);

            long endTime = System.currentTimeMillis();
            long elapsedTime = endTime - startTime;
            while (opModeIsActive()) {
                // Log the position to the telemetry
                currentPose = myOtos.getPosition();
                telemetry.addData("X coordinate", currentPose.x);
                telemetry.addData("Y coordinate", currentPose.y);
                telemetry.addData("Heading angle", currentPose.h);
                telemetry.addData("Last Runtime:", savedTime);
                telemetry.addData("Lift Arm Encoder:", liftarm.getCurrentPosition());
                telemetry.addData("Lift Arm2 Encoder:", liftarm2.getCurrentPosition());
                telemetry.addData("Extend Arm Encoder:", extentionarm.getCurrentPosition());
                telemetry.addData("Segment Execution Time", elapsedTime + " ms");
                telemetry.update();
                sleep(50);

                 /*
            Request an update from the Pinpoint odometry computer. This checks almost all outputs
            from the device in a single I2C read.
             */
                odo.update();

            /*
            Optionally, you can update only the heading of the device. This takes less time to read, but will not
            pull any other data. Only the heading (which you can pull with getHeading() or in getPosition().
             */
                //odo.update(GoBildaPinpointDriver.ReadData.ONLY_UPDATE_HEADING);


            /*
            This code prints the loop frequency of the REV Control Hub. This frequency is effected
            by I²C reads/writes. So it's good to keep an eye on. This code calculates the amount
            of time each cycle takes and finds the frequency (number of updates per second) from
            that cycle time.
             */
                double newTime = getRuntime();
                double loopTime = newTime-oldTime;
                double frequency = 1/loopTime;
                oldTime = newTime;


            /*
            gets the current Position (x & y in mm, and heading in degrees) of the robot, and prints it.
             */
                Pose2D pos = odo.getPosition();
                String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
                telemetry.addData("Position", data);

            /*
            gets the current Velocity (x & y in mm/sec and heading in degrees/sec) and prints it.
             */
                String velocity = String.format(Locale.US,"{XVel: %.3f, YVel: %.3f, HVel: %.3f}", odo.getVelX(DistanceUnit.MM), odo.getVelY(DistanceUnit.MM), odo.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES));
                telemetry.addData("Velocity", velocity);


            /*
            Gets the Pinpoint device status. Pinpoint can reflect a few states. But we'll primarily see
            READY: the device is working as normal
            CALIBRATING: the device is calibrating and outputs are put on hold
            NOT_READY: the device is resetting from scratch. This should only happen after a power-cycle
            FAULT_NO_PODS_DETECTED - the device does not detect any pods plugged in
            FAULT_X_POD_NOT_DETECTED - The device does not detect an X pod plugged in
            FAULT_Y_POD_NOT_DETECTED - The device does not detect a Y pod plugged in
            FAULT_BAD_READ - The firmware detected a bad I²C read, if a bad read is detected, the device status is updated and the previous position is reported
            */
                telemetry.addData("Status", odo.getDeviceStatus());

                telemetry.addData("Pinpoint Frequency", odo.getFrequency()); //prints/gets the current refresh rate of the Pinpoint

                telemetry.addData("REV Hub Frequency: ", frequency); //prints the control system refresh rate
                telemetry.update();

            }



        }
    }

    // Function to DRIVE to target position
    private void driveToTarget(double targetX, double targetY, double targetHeading, double timeoutSeconds, boolean FASTER) {
        runtime.reset();
        String state = "DRIVE";

        // Determine voltage compensation factor (assuming nominal 14V)
        double nominalVoltage = 12.5;
        double currentVoltage = voltageSensor.getVoltage();
        double voltageScale = nominalVoltage / currentVoltage;

        double adjusted_kS_forward = kS_forward * voltageScale;
        double adjusted_kS_strafe  = kS_strafe * voltageScale;
        double adjusted_kS_turn    = kS_turn * voltageScale;


        while (opModeIsActive() && (runtime.seconds() < timeoutSeconds)) {

            // ---------------------
            // 1) Read current pose
            // ---------------------
            currentPose = myOtos.getPosition();
            double currentX = currentPose.x;    // global X
            double currentY = currentPose.y;    // global Y
            double currentH = currentPose.h;    // global heading in degrees

            // ---------------------------
            // 2) Calculate global errors
            // ---------------------------
            double errorX_global = targetX - currentX;
            double errorY_global = targetY - currentY;

            // distance to target in meters
            double distanceToTarget = Math.sqrt(errorX_global * errorX_global + errorY_global * errorY_global);

            // heading error (in degrees)
            double errorH_global = targetHeading - normalizeAngle0To360(targetHeading, currentH);
            if (Math.abs(errorH_global) > 180) {
                errorH_global = errorH_global - Math.signum(errorH_global) * 360;
            }

            // -------------------------
            // 3) Check if we are done?
            // -------------------------
            boolean atDistance = ((Math.round(distanceToTarget * 1000.0) / 1000.0) <= DISTANCE_TOLERANCE_M);
            boolean atHeading = ((Math.round(Math.abs(errorH_global) * 10.0) / 10.0) <= HEADING_TOLERANCE_DEG);
            if (atDistance && atHeading) {
                // We have reached target within tolerances
                break;
            }

            // --------------------------------------------------------
            // 4) Transform global XY errors into robot-relative frame
            // --------------------------------------------------------
            double headingRadians = Math.toRadians(currentH);
            double errorX_local =  (errorX_global * Math.cos(headingRadians)) + (errorY_global * Math.sin(headingRadians));
            double errorY_local = (-errorX_global * Math.sin(headingRadians)) + (errorY_global * Math.cos(headingRadians));

            // -------------------------------------
            // 5) Determine bang-bang "power" levels
            // -------------------------------------

            // --- Translation (X/Y) ---
            // We'll treat (errorX_local, errorY_local) as a vector.
            // Then choose either FAST_POWER_TRANSLATION or a "fine" power
            // (which at least overcomes friction).

            // 5a) Are we far away?
            boolean inFineAdjustTrans = (distanceToTarget <= DISTANCE_FINEADJUST_M) && !atDistance;

            if (FASTER){
                inFineAdjustTrans = false;
            }

            double translationPower;
            if (!inFineAdjustTrans) {
                // Far from target -> fast
                translationPower = FAST_POWER_TRANSLATION;
            } else {
                // Added code to prioritize driving fine adjust over turning fine adjust (one or the other)
                if (state.equals("DRIVE")) {
                    // Fine adjust -> pick a single min power that can move
                    // Angle from the "forward" axis
                    double angleFromForward = Math.atan2(errorX_local, errorY_local);

                    // cos and sin of that angle
                    double c = Math.cos(angleFromForward);
                    double s = Math.sin(angleFromForward);

                    // Interpolated friction offset, this is used to find a happy inbetween power value when driving and strafing slowly
                    translationPower = (adjusted_kS_forward * (c*c)) + (adjusted_kS_strafe * (s*s));
                } else {
                    translationPower = 0.0;
                }
            }

            // Compute direction (angle) of local XY error
            double translationAngle = Math.atan2(errorY_local, errorX_local);
            double magnitudeLocal   = Math.sqrt(errorX_local*errorX_local + errorY_local*errorY_local);

            // If we're below distance tolerance, zero out
            if (atDistance) {
                translationPower = 0.0;  // or leave it to minimal if you want
                state = "TURN";
            }

            // Then final local XY power (in local coords)
            double powerX_local = translationPower * Math.cos(translationAngle);
            double powerY_local = translationPower * Math.sin(translationAngle);

            // --- Rotation (heading) ---
            boolean inFineAdjustHead = (Math.abs(errorH_global) <= HEADING_FINEADJUST_DEG) && !atHeading;

            double powerH_local;
            if (!inFineAdjustHead) {
                // Big heading error -> rotate fast
                powerH_local = FAST_POWER_ROTATION * Math.signum(errorH_global);
            } else {
                // Added code to prioritize driving fine adjust over turning fine adjust (one or the other)
                if (state.equals("TURN")) {
                    // Fine heading adjust
                    powerH_local = adjusted_kS_turn * Math.signum(errorH_global);
                } else {
                    powerH_local = 0.0;
                }
            }


            // If we're within heading tolerance, zero out rotation
            if (atHeading) {
                powerH_local = 0.0;

                // Already adjusted to target distance before, now that turn target is reached, stop the loop
                if (state.equals("TURN")) {
                    break;
                }
            }

            // --------------------------------------------------------
            // 6) Mecanum mixing: combine local X, Y, and heading power
            // --------------------------------------------------------
            double frontLeftPower  = powerY_local + powerX_local - powerH_local;
            double frontRightPower = powerY_local - powerX_local + powerH_local;
            double rearLeftPower   = powerY_local - powerX_local - powerH_local;
            double rearRightPower  = powerY_local + powerX_local + powerH_local;

            // --------------------------------
            // 7) Normalize so we don't exceed 1
            // --------------------------------
            double maxPower = Math.max(
                    Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                    Math.max(Math.abs(rearLeftPower),  Math.abs(rearRightPower))
            );
            if (maxPower > 1.0) {
                frontLeftPower  /= maxPower;
                frontRightPower /= maxPower;
                rearLeftPower   /= maxPower;
                rearRightPower  /= maxPower;
            }

            // -----------------------------------------------
            // 8) Apply voltage compensation before sending powers
            // -----------------------------------------------
            frontleft.setPower(Range.clip(frontLeftPower * voltageScale, -1.0, 1.0));
            frontright.setPower(Range.clip(frontRightPower * voltageScale, -1.0, 1.0));
            rearleft.setPower(Range.clip(rearLeftPower * voltageScale, -1.0, 1.0));
            rearright.setPower(Range.clip(rearRightPower * voltageScale, -1.0, 1.0));

            // Log data
            df1.set(targetX);
            df2.set(targetY);
            df3.set(targetHeading);
            df4.set(currentX);
            df5.set(currentY);
            df6.set(currentH);
            df7.set(errorX_global);
            df8.set(errorY_global);
            df9.set(distanceToTarget);
            df10.set(errorH_global);
            df11.set(errorX_local);
            df12.set(errorY_local);
            df13.set(powerX_local);
            df14.set(powerY_local);
            df15.set(powerH_local);
            df16.set(frontLeftPower);
            df17.set(frontRightPower);
            df18.set(rearLeftPower);
            df19.set(rearRightPower);
            df20.set(voltageSensor.getVoltage());
            df21.set(runtime.seconds());
            df22.set(inFineAdjustTrans ? 1 : 0);
            df23.set(inFineAdjustHead ? 1 : 0);
            datalog.writeLine();

            idle();
        }

        // Stop all motors
        frontleft.setPower(0);
        frontright.setPower(0);
        rearleft.setPower(0);
        rearright.setPower(0);

        // Save the runtime of the operation
        savedTime = runtime.seconds();
    }

    // Fixes the laser odometry sensor angle from -180 to 180 to 0 to 360.
    // Sensor goes to +179.9 and a degree more turns into -179.9!, making it impossible to actually hit 180 degrees or even perform a 270 rotation.
    private double normalizeAngle0To360(double targetAngle, double currentAngle) {
        // Handle the special case around 180 degrees
        if (Math.abs(currentAngle) > 179.5 && Math.abs(currentAngle) < 180.5) {
            return Math.signum(currentAngle) * 180.0;
        }

        // Convert angles to continuous range
        double adjustedCurrent = currentAngle;
        if (currentAngle < -179.0 && targetAngle > 0) {
            adjustedCurrent += 360.0;
        } else if (currentAngle > 179.0 && targetAngle < 0) {
            adjustedCurrent -= 360.0;
        }

        return adjustedCurrent;
    }

    // Initialize hardware and setup datalogger
    private void initializeHardware() {
        // Setup datalogger fields
        df1 = new Datalogger.GenericField("Target X (m)");           // Target X position in meters
        df2 = new Datalogger.GenericField("Target Y (m)");           // Target Y position in meters
        df3 = new Datalogger.GenericField("Target Heading (deg)");   // Target heading in degrees
        df4 = new Datalogger.GenericField("Current X (m)");           // Current X position in meters
        df5 = new Datalogger.GenericField("Current Y (m)");           // Current Y position in meters
        df6 = new Datalogger.GenericField("Current Heading (deg)");  // Current heading in degrees
        df7 = new Datalogger.GenericField("Error X (global) (m)");   // Global X error in meters
        df8 = new Datalogger.GenericField("Error Y (global) (m)");   // Global Y error in meters
        df9 = new Datalogger.GenericField("Distance to Target (m)"); // Distance to target in meters
        df10 = new Datalogger.GenericField("Error Heading (global) (deg)"); // Global heading error in degrees
        df11 = new Datalogger.GenericField("Error X (local) (m)");   // Local X error in meters
        df12 = new Datalogger.GenericField("Error Y (local) (m)");   // Local Y error in meters
        df13 = new Datalogger.GenericField("Power X (local)");       // Local X power
        df14 = new Datalogger.GenericField("Power Y (local)");       // Local Y power
        df15 = new Datalogger.GenericField("Power H (local)");       // Local heading power
        df16 = new Datalogger.GenericField("Front Left Power");       // Front left motor power
        df17 = new Datalogger.GenericField("Front Right Power");       // Front right motor power
        df18 = new Datalogger.GenericField("Rear Left Power");       // Rear left motor power
        df19 = new Datalogger.GenericField("Rear Right Power");       // Rear right motor power
        df20 = new Datalogger.GenericField("Voltage (V)");           // Voltage
        df21 = new Datalogger.GenericField("Time (s)");               // Time in seconds
        df22 = new Datalogger.GenericField("inFineAdjustTrans");       // In fine adjust translation
        df23 = new Datalogger.GenericField("inFineAdjustHead");       // In fine adjust heading

        // Create datalogger
        datalog = new Datalogger.Builder()
                .setFilename("PID_Datalog")  // Name your file
                .setFields(df1, df2, df3, df4, df5, df6, df7, df8, df9, df10, df11, df12, df13, df14, df15, df16, df17, df18, df19, df20, df21, df22, df23)
                .setAutoTimestamp(Datalogger.AutoTimestamp.DECIMAL_SECONDS)
                .build();

        // Initialize the hardware variables and set the direction and zero power behavior
        frontleft = hardwareMap.get(DcMotor.class, "frontleft");  // Control Hub - Motors - 2 - REV Robotics UltraPlanetary HD Hex Motor
        frontright = hardwareMap.get(DcMotor.class, "frontright"); // Control Hub - Motors - 1 - REV Robotics UltraPlanetary HD Hex Motor
        rearleft = hardwareMap.get(DcMotor.class, "rearleft");  // Control Hub - Motors - 3 - REV Robotics UltraPlanetary HD Hex Motor
        rearright = hardwareMap.get(DcMotor.class, "rearright");  // Control Hub - Motors - 0 - REV Robotics UltraPlanetary HD Hex Motor
        extentionarm = hardwareMap.get(DcMotor.class, "extention arm");
        liftarm = hardwareMap.get(DcMotor.class, "lift arm");
        liftarm2 = hardwareMap.get(DcMotor.class, "lift arm 2");
        armpivot = hardwareMap.get(Servo.class, "armpivot");
        upperpivot = hardwareMap.get(Servo.class, "upperpivot");
        lowerpivot = hardwareMap.get(Servo.class, "lowerpivot");
        clawpivot = hardwareMap.get(Servo.class, "clawpivot");
        gripl = hardwareMap.get(CRServo.class, "gripl");
        gripr = hardwareMap.get(CRServo.class, "gripr");

        frontleft.setDirection(DcMotor.Direction.FORWARD);
        frontright.setDirection(DcMotor.Direction.REVERSE);
        rearleft.setDirection(DcMotor.Direction.FORWARD);
        rearright.setDirection(DcMotor.Direction.FORWARD);
        extentionarm.setDirection(DcMotor.Direction.FORWARD);
        liftarm.setDirection(DcMotor.Direction.FORWARD);
        liftarm2.setDirection(DcMotor.Direction.FORWARD);

        frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extentionarm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftarm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftarm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset encoders
        extentionarm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftarm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftarm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        extentionarm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftarm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftarm2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize ODOS
        myOtos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        myOtos.setLinearUnit(DistanceUnit.METER);
        myOtos.setAngularUnit(AngleUnit.DEGREES);
        myOtos.setOffset(new SparkFunOTOS.Pose2D(0, 0, 0));
        myOtos.calibrateImu();
        myOtos.resetTracking();
        myOtos.setLinearScalar(0.998700369);
        myOtos.setAngularScalar(0.994443272);
        myOtos.setPosition(new SparkFunOTOS.Pose2D(0, 0, -90));

        // Initialize voltage sensor
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        telemetry.addData("AUTO v9 CLIP - Ready team!", "Press Play Button");
        telemetry.update();

    }

    private void liftArmToPosition(int targetPosition) {
        // Activate auto mode.

        ((DcMotorEx) liftarm).setTargetPositionTolerance(1);
        ((DcMotorEx) liftarm2).setTargetPositionTolerance(1);

        liftarm.setTargetPosition(-targetPosition);
        liftarm2.setTargetPosition(-targetPosition);

        liftarm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftarm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftarm.setPower(1);
        liftarm2.setPower(1);

    }

    private void extendArmToPosition(int targetPosition) {
        // Activate auto mode.

        ((DcMotorEx) extentionarm).setTargetPositionTolerance(1);

        extentionarm.setTargetPosition(targetPosition);

        extentionarm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        extentionarm.setPower(1);

    }



}
