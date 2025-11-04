package org.firstinspires.ftc.teamcode;
import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import java.util.Locale;

@TeleOp(name="TeleNew")
public class TeleNew extends LinearOpMode {
    // Hardware map variables
    private DcMotor frontleft = null;
    private DcMotor frontright = null;
    private DcMotor rearleft = null;
    private DcMotor rearright = null;
    private DcMotor intake = null;
    private CRServo left = null;
    private CRServo middle = null;
    private CRServo right = null;

    NormalizedColorSensor colorSensorL;
    NormalizedColorSensor colorSensorM;
    NormalizedColorSensor colorSensorR;

    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer

    double oldTime = 0;

    public void runOpMode(){

        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        odo.setOffsets(-80, -88, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.recalibrateIMU();
        odo.resetPosAndIMU();

        frontleft  = hardwareMap.get(DcMotor.class, "frontleft");
        frontright = hardwareMap.get(DcMotor.class, "frontright");
        rearleft   = hardwareMap.get(DcMotor.class, "rearleft");
        rearright  = hardwareMap.get(DcMotor.class, "rearright");
        intake = hardwareMap.get(DcMotor.class, "intake");
        left = hardwareMap.get(CRServo.class, "left");
        middle = hardwareMap.get(CRServo.class, "middle");
        right = hardwareMap.get(CRServo.class, "right");

        frontleft.setDirection(DcMotor.Direction.REVERSE);
        rearleft.setDirection(DcMotor.Direction.REVERSE);

        frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", odo.getXOffset(DistanceUnit.MM));
        telemetry.addData("Y offset", odo.getYOffset(DistanceUnit.MM));
        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Heading Scalar", odo.getYawScalar());
        telemetry.update();


        // Wait for the game to start (driver presses START)
        waitForStart();
        resetRuntime();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

//            runSample();

            //#region Odometry
            odo.update();
            //odo.update(GoBildaPinpointDriver.ReadData.ONLY_UPDATE_HEADING);
            if (gamepad1.a){
                odo.resetPosAndIMU(); //resets the position to 0 and recalibrates the IMU
            }
            if (gamepad1.b){
                odo.recalibrateIMU(); //recalibrates the IMU without resetting position
            }
            double newTime = getRuntime();
            double loopTime = newTime-oldTime;
            double frequency = 1/loopTime;
            oldTime = newTime;
            Pose2D pos = odo.getPosition();

            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);
            String velocity = String.format(Locale.US,"{XVel: %.3f, YVel: %.3f, HVel: %.3f}", odo.getVelX(DistanceUnit.MM), odo.getVelY(DistanceUnit.MM), odo.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES));
            telemetry.addData("Velocity", velocity);
            telemetry.addData("Status", odo.getDeviceStatus());
            telemetry.addData("Pinpoint Frequency", odo.getFrequency()); //prints/gets the current refresh rate of the Pinpoint
            telemetry.addData("REV Hub Frequency: ", frequency); //prints the control system refresh rate
            telemetry.update();
            //#endregion

            //#region Drive

            // Get joystick inputs
            double y = -gamepad1.left_stick_y; // forward/back (invert for natural controls)
            double x = gamepad1.left_stick_x;  // strafe
            double rx = gamepad1.right_stick_x; // rotation

            // Optional slow mode toggle
            double driveSpeed = gamepad1.right_bumper ? 0.4 : 1.0; // hold RB for slow mode

            // Get robot heading (for field-centric)
            double botHeading = odo.getHeading(AngleUnit.RADIANS);

            // Field-centric transform
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);


            // Compute wheel powers
            double frontLeftPower  = rotY + rotX + rx;
            double rearLeftPower   = rotY - rotX + rx;
            double frontRightPower = rotY - rotX - rx;
            double rearRightPower  = rotY + rotX - rx;



            // Normalize
            double max = Math.max(
                    1.0,
                    Math.max(Math.abs(frontLeftPower), Math.max(Math.abs(frontRightPower),
                            Math.max(Math.abs(rearLeftPower), Math.abs(rearRightPower))))
            );
            frontLeftPower  /= max;
            frontRightPower /= max;
            rearLeftPower   /= max;
            rearRightPower  /= max;

            // Apply power
            frontleft.setPower(frontLeftPower * driveSpeed);
            frontright.setPower(frontRightPower * driveSpeed);
            rearleft.setPower(rearLeftPower * driveSpeed);
            rearright.setPower(rearRightPower * driveSpeed);


            //#endregion

            //#region Buttons

            if(gamepad1.x){
                lineUp();
            }

            if(gamepad2.left_bumper){
                intake.setPower(-1);
            } else if (gamepad2.right_bumper){
                intake.setPower(1);
            } else {
                intake.setPower(0);
            }

            if(gamepad2.x){
                left.setPower(1);
            } else {
                left.setPower(0);
            }

            if(gamepad2.y){
                middle.setPower(1);
            } else {
                middle.setPower(0);
            }

            if(gamepad2.b){
                right.setPower(1);
            } else {
                right.setPower(0);
            }
            //#endregion


        }

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

        // Get a reference to our sensor object. It's recommended to use NormalizedColorSensor over
        // ColorSensor, because NormalizedColorSensor consistently gives values between 0 and 1, while
        // the values you get from ColorSensor are dependent on the specific sensor you're using.

        colorSensorL = hardwareMap.get(NormalizedColorSensor.class, "colorL");
        colorSensorM = hardwareMap.get(NormalizedColorSensor.class, "colorM");
        colorSensorR = hardwareMap.get(NormalizedColorSensor.class, "colorR");

        // Wait for the start button to be pressed.
        waitForStart();

        // Loop until we are asked to stop
        while (opModeIsActive()) {



            // Show the gain value via telemetry
            telemetry.addData("Gain", gain);

            // Tell the sensor our desired gain value (normally you would do this during initialization,
            // not during the loop)
            colorSensorL.setGain(gain);
            colorSensorM.setGain(gain);
            colorSensorR.setGain(gain);



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

    public void lineUp() {
        double targetHeading = 0;     // Desired heading (degrees)
        double Kp = 0.01;             // Proportional gain
        double minPower = 0.12;       // Minimum power to move motors
        double maxPower = 0.4;        // Max turn power
        double tolerance = 1.0;       // Stop when within 1 degree

        double currentHeading = odo.getHeading(AngleUnit.DEGREES);

        // Compute shortest path error (normalize to [-180, 180])
        double error = targetHeading - currentHeading;
        error = (error + 540) % 360 - 180;

        // If we’re already close enough, stop motors and return
        if (Math.abs(error) <= tolerance) {
            frontleft.setPower(0);
            frontright.setPower(0);
            rearleft.setPower(0);
            rearright.setPower(0);
            return;
        }

        // Proportional control for turn power
        double turnPower = Kp * error;
        turnPower = Range.clip(turnPower, -maxPower, maxPower);

        // Ensure minimum power to overcome friction
        if (Math.abs(turnPower) < minPower)
            turnPower = Math.copySign(minPower, turnPower);

        // Apply rotational power (tank-style turn)
        frontleft.setPower(turnPower);
        rearleft.setPower(turnPower);
        frontright.setPower(-turnPower);
        rearright.setPower(-turnPower);

        telemetry.addData("Target", targetHeading);
        telemetry.addData("Heading", currentHeading);
        telemetry.addData("Error", error);
        telemetry.addData("Turn Power", turnPower);
        telemetry.update();
    }


}
