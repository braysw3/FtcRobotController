package org.firstinspires.ftc.teamcode;
import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.VoltageSensor;



import java.util.Locale;

@Autonomous(name="AutoNew")
public class AutoNew extends LinearOpMode
{
    //#region Variables and Constants
    // Hardware map variables
    private DcMotor frontleft = null;
    private DcMotor frontright = null;
    private DcMotor rearleft = null;
    private DcMotor rearright = null;


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

    private Pose2D currentPose;

    NormalizedColorSensor colorSensorL;
    NormalizedColorSensor colorSensorM;
    NormalizedColorSensor colorSensorR;

    View relativeLayout;


    //#endregion

    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer

    double oldTime = 0;

    private double normalizeRadians(double angle) {
        angle = angle % (2 * Math.PI);
        if (angle < 0) angle += 2 * Math.PI;
        return angle;
    }

    private double angleWrap(double angle) {
        while (angle <= -Math.PI) angle += 2 * Math.PI;
        while (angle > Math.PI) angle -= 2 * Math.PI;
        return angle;
    }


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
        odo.setOffsets(-80, -88, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1

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
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);


        /*
        Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
        The IMU will automatically calibrate when first powered on, but recalibrating before running
        the robot is a good idea to ensure that the calibration is "good".
        resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
        This is recommended before you run your autonomous, as a bad initial calibration can cause
        an incorrect starting value for x, y, and heading.
         */
        odo.resetPosAndIMU();
        odo.recalibrateIMU();


        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", odo.getXOffset(DistanceUnit.MM));
        telemetry.addData("Y offset", odo.getYOffset(DistanceUnit.MM));
        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Heading Scalar", odo.getYawScalar());
        telemetry.update();

        // Get a reference to the RelativeLayout so we can later change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        try {
            runSample(); // actually execute the sample
        } finally {
            // On the way out, *guarantee* that the background is reasonable. It doesn't actually start off
            // as pure white, but it's too much work to dig out what actually was used, and this is good
            // enough to at least make the screen reasonable again.
            // Set the panel back to the default color
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.WHITE);
                }
            });
        }

        initializeHardware();
        waitForStart();

        if (opModeIsActive())
        {
            odo.update();


            //*****************************************************************/
            long startTime = System.currentTimeMillis();

            //test
            driveToTarget(10,0,0,2,false);


            long endTime = System.currentTimeMillis();
            long elapsedTime = endTime - startTime;
            while (opModeIsActive()) {


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
    private void driveToTarget(double targetXcm, double targetYcm, double targetHeadingDeg, double timeoutSeconds, boolean FASTER) {
        runtime.reset();

        // Convert input from cm → mm for Pinpoint
        double targetXMM = targetXcm * 10.0;
        double targetYMM = targetYcm * 10.0;

        // Convert target heading to radians
        double targetHeadingRad = Math.toRadians(targetHeadingDeg);

        // Slowdown parameters
        double slowdownStartCM = 10.0; // start slowing down inside 10cm
        double minSpeedFactor = 0.25;  // never go below 25% of base speed
        double maxTransPower = FASTER ? 0.6 : 0.4; // max translation power
        double maxRotPower   = 0.5;                 // max rotation power

        while (opModeIsActive() && runtime.seconds() < timeoutSeconds) {
            odo.update();

            // Current pose
            Pose2D pose = odo.getPosition();
            double currentX = pose.getX(DistanceUnit.MM);
            double currentY = pose.getY(DistanceUnit.MM);
            double currentH = pose.getHeading(AngleUnit.RADIANS);

            // Compute errors
            double errorX = targetXMM - currentX;
            double errorY = targetYMM - currentY;
            double errorH = angleWrap(targetHeadingRad - currentH);

            // Distance to target
            double distanceMM = Math.hypot(errorX, errorY);

            // Stop if within tolerances
            if (distanceMM < 5.0 && Math.abs(Math.toDegrees(errorH)) < 1.0) {
                break;
            }

            // --- Slow down near target ---
            double slowFactor = 1.0;
            if (distanceMM < slowdownStartCM * 10.0) { // convert cm → mm
                slowFactor = minSpeedFactor + (1.0 - minSpeedFactor) * (distanceMM / (slowdownStartCM * 10.0));
            }

            // Transform global error into robot-relative frame
            double cosH = Math.cos(currentH);
            double sinH = Math.sin(currentH);
            double errorXRobot =  cosH * errorX + sinH * errorY;
            double errorYRobot = -sinH * errorX + cosH * errorY;

            // Proportional control constants
            double kPTrans = 0.02 * slowFactor;  // translation
            double kPRot   = 0.05 * slowFactor;  // rotation

            // Compute robot-relative powers
            double powerX = Range.clip(errorXRobot * kPTrans, -maxTransPower, maxTransPower);
            double powerY = Range.clip(errorYRobot * kPTrans, -maxTransPower, maxTransPower);
            double powerH = Range.clip(errorH * kPRot, -maxRotPower, maxRotPower);

            // --- Mecanum mixing ---
            double fl = powerY + powerX - powerH;
            double fr = powerY - powerX + powerH;
            double rl = powerY - powerX - powerH;
            double rr = powerY + powerX + powerH;

            // Normalize powers
            double maxPower = Math.max(1.0, Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                    Math.max(Math.abs(rl), Math.abs(rr))));
            fl /= maxPower; fr /= maxPower; rl /= maxPower; rr /= maxPower;

            // Send to motors
            frontleft.setPower(fl);
            frontright.setPower(fr);
            rearleft.setPower(rl);
            rearright.setPower(rr);

            // Telemetry for debugging
            telemetry.addData("X error (cm)", errorX / 10.0);
            telemetry.addData("Y error (cm)", errorY / 10.0);
            telemetry.addData("Heading error (deg)", Math.toDegrees(errorH));
            telemetry.addData("FL", fl); telemetry.addData("FR", fr);
            telemetry.addData("RL", rl); telemetry.addData("RR", rr);
            telemetry.update();

            idle();
        }

        // Stop all motors
        frontleft.setPower(0);
        frontright.setPower(0);
        rearleft.setPower(0);
        rearright.setPower(0);
    }

    protected void runSample() {
        // You can give the sensor a gain value, will be multiplied by the sensor's raw value before the
        // normalized color values are calculated. Color sensors (especially the REV Color Sensor V3)
        // can give very low values (depending on the lighting conditions), which only use a small part
        // of the 0-1 range that is available for the red, green, and blue values. In brighter conditions,
        // you should use a smaller gain than in dark conditions. If your gain is too high, all of the
        // colors will report at or near 1, and you won't be able to determine what color you are
        // actually looking at. For this reason, it's better to err on the side of a lower gain
        // (but always greater than  or equal to 1).
        float gain = 2;

        // Once per loop, we will update this hsvValues array. The first element (0) will contain the
        // hue, the second element (1) will contain the saturation, and the third element (2) will
        // contain the value. See http://web.archive.org/web/20190311170843/https://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html
        // for an explanation of HSV color.
        final float[] hsvValuesL = new float[3];
        final float[] hsvValuesM = new float[3];
        final float[] hsvValuesR = new float[3];

        // xButtonPreviouslyPressed and xButtonCurrentlyPressed keep track of the previous and current
        // state of the X button on the gamepad
        boolean xButtonPreviouslyPressed = false;
        boolean xButtonCurrentlyPressed = false;

        // Get a reference to our sensor object. It's recommended to use NormalizedColorSensor over
        // ColorSensor, because NormalizedColorSensor consistently gives values between 0 and 1, while
        // the values you get from ColorSensor are dependent on the specific sensor you're using.

        colorSensorL = hardwareMap.get(NormalizedColorSensor.class, "colorL");
        colorSensorM = hardwareMap.get(NormalizedColorSensor.class, "colorM");
        colorSensorR = hardwareMap.get(NormalizedColorSensor.class, "colorR");



        // If possible, turn the light on in the beginning (it might already be on anyway,
        // we just make sure it is if we can).

//        ((SwitchableLight)colorSensorLeft).enableLight(true);
//        ((SwitchableLight)colorSensorRight).enableLight(true);

        // Wait for the start button to be pressed.
        waitForStart();

        // Loop until we are asked to stop
        while (opModeIsActive()) {
            // Explain basic gain information via telemetry
            telemetry.addLine("Hold the A button on gamepad 1 to increase gain, or B to decrease it.\n");
            telemetry.addLine("Higher gain values mean that the sensor will report larger numbers for Red, Green, and Blue, and Value\n");

            // Update the gain value if either of the A or B gamepad buttons is being held
            if (gamepad1.a) {
                // Only increase the gain by a small amount, since this loop will occur multiple times per second.
                gain += 0.005;
            } else if (gamepad1.b && gain > 1) { // A gain of less than 1 will make the values smaller, which is not helpful.
                gain -= 0.005;
            }

            // Show the gain value via telemetry
            telemetry.addData("Gain", gain);

            // Tell the sensor our desired gain value (normally you would do this during initialization,
            // not during the loop)
            colorSensorL.setGain(gain);
            colorSensorM.setGain(gain);
            colorSensorR.setGain(gain);

            // Check the status of the X button on the gamepad
            xButtonCurrentlyPressed = gamepad1.x;

            // If the button state is different than what it was, then act
            if (xButtonCurrentlyPressed != xButtonPreviouslyPressed) {
                // If the button is (now) down, then toggle the light
                if (xButtonCurrentlyPressed) {
//                    SwitchableLight light = (SwitchableLight)colorSensorLeft;
//                    SwitchableLight light1 = (SwitchableLight)colorSensorRight;
//
//                    light.enableLight(!light.isLightOn());
//                    light1.enableLight(!light1.isLightOn());
                }
            }
            xButtonPreviouslyPressed = xButtonCurrentlyPressed;

            // Get the normalized colors from the sensor
            NormalizedRGBA colorsL = colorSensorL.getNormalizedColors();
            NormalizedRGBA colorsM = colorSensorM.getNormalizedColors();
            NormalizedRGBA colorsR = colorSensorR.getNormalizedColors();

            /* Use telemetry to display feedback on the driver station. We show the red, green, and blue
             * normalized values from the sensor (in the range of 0 to 1), as well as the equivalent
             * HSV (hue, saturation and value) values. See http://web.archive.org/web/20190311170843/https://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html
             * for an explanation of HSV color. */

            // Update the hsvValues array by passing it to Color.colorToHSV()
            Color.colorToHSV(colorsL.toColor(), hsvValuesL);
            Color.colorToHSV(colorsM.toColor(), hsvValuesM);
            Color.colorToHSV(colorsR.toColor(), hsvValuesR);

            telemetry.addData("Hue left", "%.3f", hsvValuesL[0]).addData("Distance left", "%.3f", ((DistanceSensor) colorSensorL).getDistance(DistanceUnit.CM));
            telemetry.addData("Hue left", "%.3f", hsvValuesM[0]).addData("Distance left", "%.3f", ((DistanceSensor) colorSensorM).getDistance(DistanceUnit.CM));
            telemetry.addData("Hue right", "%.3f", hsvValuesR[0]).addData("Distance right", "%.3f", ((DistanceSensor) colorSensorR).getDistance(DistanceUnit.CM));

            /* If this color sensor also has a distance sensor, display the measured distance.
             * Note that the reported distance is only useful at very close range, and is impacted by
             * ambient light and surface reflectivity. */


            String leftcolor = returnColor(hsvValuesL[0]);
            String middlecolor = returnColor(hsvValuesM[0]);
            String rightcolor = returnColor(hsvValuesR[0]);



            telemetry.update();

            // Change the Robot Controller's background color to match the color detected by the color sensor.
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.HSVToColor(hsvValuesL));

                }
            });
        }
    }

    public String returnColor(float HValue){

        if(HValue == 0){
            return "none";
        }
        if(HValue <= 200){
            return "green";
        }

        if(HValue >= 200){
            return "purple";
        }

        return "none";

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

        frontleft.setDirection(DcMotor.Direction.FORWARD);
        frontright.setDirection(DcMotor.Direction.REVERSE);
        rearleft.setDirection(DcMotor.Direction.FORWARD);
        rearright.setDirection(DcMotor.Direction.FORWARD);

        frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset encoders

        // Initialize ODOS
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(0,0,DistanceUnit.METER);
        odo.recalibrateIMU();
        odo.resetPosAndIMU();

        // Initialize voltage sensor
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        telemetry.addData("AUTO v9 CLIP - Ready team!", "Press Play Button");
        telemetry.update();

    }

    public void rotateAroundPoint(double angleDeg,
                                  double xOffsetMM,
                                  double yOffsetMM,
                                  double omegaDegPerSec) {

        double angleRad = Math.toRadians(angleDeg);
        double omegaRadPerSec = Math.toRadians(Math.abs(omegaDegPerSec));

        // Track how much we’ve actually rotated
        double startHeading = odo.getPosition().getHeading(AngleUnit.RADIANS);
        double lastHeading = startHeading;
        double cumulativeRotation = 0.0;

        ElapsedTime timer = new ElapsedTime();
        double maxTime = Math.abs(angleDeg / omegaDegPerSec) * 2.0 + 2.0;

        while (opModeIsActive() && timer.seconds() < maxTime) {
            odo.update();

            double currentHeading = odo.getPosition().getHeading(AngleUnit.RADIANS);

            // Change in heading since last loop
            double delta = angleWrap(currentHeading - lastHeading);
            cumulativeRotation += delta;
            lastHeading = currentHeading;

            double remaining = angleRad - cumulativeRotation;

            // --- Proportional slowdown logic ---
            double maxOmega = omegaRadPerSec;       // max angular speed
            double kP = 2.0;                        // proportional gain (tune this)
            double omega = kP * remaining;          // proportional term

            // Clamp to max speed
            omega = Math.max(-maxOmega, Math.min(omega, maxOmega));

            // Add a minimum speed (so it doesn’t stall near target)
            double minSpeed = 0.25 * maxOmega;      // 25% of max
            if (Math.abs(omega) < minSpeed && Math.abs(remaining) > Math.toRadians(1.0)) {
                omega = Math.signum(omega) * minSpeed;
            }

            // --- Translation for orbit (offset is in mm, convert to meters) ---
            double vx = -omega * (yOffsetMM / 1000.0);
            double vy =  omega * (xOffsetMM / 1000.0);

            // Scaling
            double kTrans = 1.0;
            double kRot   = 0.5;
            double powerX = vx * kTrans;
            double powerY = vy * kTrans;
            double powerR = omega * kRot;

            // --- Mecanum mixing ---
            double fl = powerY + powerX - powerR;
            double fr = powerY - powerX + powerR;
            double rl = powerY - powerX - powerR;
            double rr = powerY + powerX + powerR;

            // Normalize powers
            double max = Math.max(1.0, Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                    Math.max(Math.abs(rl), Math.abs(rr))));
            fl /= max; fr /= max; rl /= max; rr /= max;

            // Send to motors
            frontleft.setPower(fl);
            frontright.setPower(fr);
            rearleft.setPower(rl);
            rearright.setPower(rr);

            telemetry.addData("Cumulative (deg)", Math.toDegrees(cumulativeRotation));
            telemetry.addData("Remaining (deg)", Math.toDegrees(remaining));
            telemetry.addData("Omega (deg/s)", Math.toDegrees(omega));
            telemetry.update();

            // Stop if we’ve rotated enough
            if (Math.abs(remaining) < Math.toRadians(1.0)) {
                break;
            }
        }

        // Stop
        frontleft.setPower(0);
        frontright.setPower(0);
        rearleft.setPower(0);
        rearright.setPower(0);

        telemetry.addLine("Orbit complete");
        telemetry.update();
    }















}
