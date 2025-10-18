package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import java.util.Locale;

@TeleOp(name="New")
public class TeleNew extends LinearOpMode {
    // Hardware map variables
    private DcMotor frontleft = null;
    private DcMotor frontright = null;
    private DcMotor rearleft = null;
    private DcMotor rearright = null;

    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer

    double oldTime = 0;

    public void runOpMode(){

        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        odo.setOffsets(-80, -228, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.recalibrateIMU();
        odo.resetPosAndIMU();

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

        }

    }
}
