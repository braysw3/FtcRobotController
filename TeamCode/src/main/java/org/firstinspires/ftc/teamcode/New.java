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
public class New extends LinearOpMode
{
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

    // Motor control variables
    private double xF = 0;
    private double yF = 0;
    private double drive = 0;
    private double strafe = 0;
    private double turn = 0;
    private double leftFrontPower = 0;
    private double rightFrontPower = 0;
    private double leftBackPower = 0;
    private double rightBackPower = 0;
    private double max = 0;
    private int frontLeftEncoder = 0;
    private int frontRightEncoder = 0;
    private int rearLeftEncoder = 0;
    private int rearRightEncoder = 0;
    private double gearReduction = 2; // Gear reduction for low gear
    private double STICK_DEADZONE = 0.00;
    private double clawPivotPosition = 0.35;
    private double armPivotPosition = 0.7;

    // Flag to indicate that auto mode is active.
    private boolean autoModeActive = false;

    public static final double TICKS_PER_REV = 8192;
    public static final double WHEEL_RADIUS = 1; // inches
    public static final double GEAR_RATIO = 1;

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer

    double oldTime = 0;

    @Override public void runOpMode()
    {
        //#region Intitialize Hardware
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
        myOtos.setPosition(new SparkFunOTOS.Pose2D(0, 0, 0));
        //#endregion

        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        odo.setOffsets(-84.0, -168.0, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.recalibrateIMU();
        odo.resetPosAndIMU();
        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", odo.getXOffset(DistanceUnit.MM));
        telemetry.addData("Y offset", odo.getYOffset(DistanceUnit.MM));
        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Heading Scalar", odo.getYawScalar());

        telemetry.addData("TELEOP - Ready team!", "Press Play Button");
        telemetry.update();
        waitForStart();



        while (opModeIsActive())
        {
            odo.update();


            SparkFunOTOS.Pose2D currentPose = myOtos.getPosition();
            // log some telem
            telemetry.addData("leftFrontPower", leftFrontPower);
            telemetry.addData("rightFrontPower", rightFrontPower);
            telemetry.addData("leftBackPower", leftBackPower);
            telemetry.addData("rightBackPower", rightBackPower);
            telemetry.addData("ExtArm", extentionarm.getCurrentPosition());
            telemetry.addData("LiftArm", liftarm.getCurrentPosition());
            telemetry.addData("X", currentPose.x);
            telemetry.addData("Y", currentPose.y);
            telemetry.addData("H", currentPose.h);
            telemetry.addData("gamepad2.right_stick_y", gamepad2.right_stick_y);
            telemetry.addData("LiftArm Power", liftarm.getPower());
            telemetry.addData("LiftArm2 Power", liftarm2.getPower());
            telemetry.addData("LiftArm Busy", liftarm.isBusy());
            telemetry.addData("LiftArm2 Busy", liftarm2.isBusy());
            telemetry.update();



            //*************************************************************************
            //  CONTROLS
            //*************************************************************************

            // Read manual input.
            double manualInput = gamepad2.right_stick_y;

            // GamePad 1 - Drive
            if (Math.abs(gamepad1.left_stick_x) + Math.abs(gamepad1.left_stick_y) + Math.abs(gamepad1.right_stick_x) > 0)
            {
                // xF, yF from left stick (field-centric motion)
                xF = gamepad1.left_stick_x;   // Field X command
                yF = -gamepad1.left_stick_y;  // Field Y command (invert to make up positive)

                // Apply deadzone to the joysticks
                if (Math.abs(gamepad1.left_stick_x) < STICK_DEADZONE) xF = 0;
                if (Math.abs(gamepad1.left_stick_y) < STICK_DEADZONE) yF = 0;

                // Precompute sine/cosine of heading for convenience.
                double headingRadians = Math.toRadians(currentPose.h);
                double cosH = Math.cos(headingRadians);
                double sinH = Math.sin(headingRadians);

                // Rotate the field-centric inputs into the robot's local coordinate frame.
                drive = xF * -sinH + yF * cosH;
                strafe = xF * cosH + yF * sinH;

                // driver control turning
                turn = gamepad1.right_stick_x / 1;
                if (Math.abs(turn) < STICK_DEADZONE) turn = 0;



                //--------------------------------------------------------------------------
                // Mecanum drive formulas in robot coordinates.
                //--------------------------------------------------------------------------
                double leftFrontPower  = drive + strafe + turn;
                double rightFrontPower = drive - strafe - turn;
                double leftBackPower   = drive - strafe + turn;
                double rightBackPower  = drive + strafe - turn;

                // Normalize wheel powers
                double maxMagnitude = Math.max(
                        Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower)),
                        Math.max(Math.abs(leftBackPower),  Math.abs(rightBackPower)));
                if (maxMagnitude > 1.0)
                {
                    leftFrontPower  /= maxMagnitude;
                    rightFrontPower /= maxMagnitude;
                    leftBackPower   /= maxMagnitude;
                    rightBackPower  /= maxMagnitude;
                }

                // Send powers to the motors
                if (gamepad1.a)
                {
                    // Low gear
                    frontleft.setPower(leftFrontPower / gearReduction);
                    frontright.setPower(rightFrontPower / gearReduction);
                    rearleft.setPower(leftBackPower / gearReduction);
                    rearright.setPower(rightBackPower / gearReduction);
                }
                else
                {
                    // High gear
                    frontleft.setPower(leftFrontPower);
                    frontright.setPower(rightFrontPower);
                    rearleft.setPower(leftBackPower);
                    rearright.setPower(rightBackPower);
                }
            }
            else
            {
                // Stop the robot if no input and heading lock is off
                frontleft.setPower(0);
                frontright.setPower(0);
                rearleft.setPower(0);
                rearright.setPower(0);
            }





            if(gamepad1.dpad_down){
                myOtos.setPosition(new SparkFunOTOS.Pose2D(0, 0, 0));
            }

            // roller release
            if (Math.abs(gamepad2.right_trigger) > 0) {
                gripl.setPower(gamepad2.right_trigger);
                gripr.setPower(-gamepad2.right_trigger);

                // roller catch
            } else if (Math.abs(gamepad2.left_trigger) > 0) {
                gripl.setPower(-gamepad2.left_trigger);
                gripr.setPower(gamepad2.left_trigger);

            } else {
                gripl.setPower(0);
                gripr.setPower(0);
            }

            if (gamepad1.y){
                odo.resetPosAndIMU(); //resets the position to 0 and recalibrates the IMU
            }

            if (gamepad1.b){
                odo.recalibrateIMU(); //recalibrates the IMU without resetting position
            }

            // Pick from floor
            // if (gamepad2.dpad_down) {
            //	 armpivot.setPosition(0.5);
            //	 upperpivot.setPosition(0.55);
            //	 lowerpivot.setPosition(0.45);
            //	 clawpivot.setPosition(0.5);

            // }
            if (gamepad2.dpad_down) {
                armpivot.setPosition(0.5);
                upperpivot.setPosition(0.55);
                lowerpivot.setPosition(0.45);
                clawpivot.setPosition(0.5);


            }


            // Clip
            if (gamepad2.dpad_up){
                lowerpivot.setPosition(0.64);
                upperpivot.setPosition(0.6);
                armpivot.setPosition(0.23);
                clawpivot.setPosition(0.25);
            }

            // Pick from wall
            if (gamepad2.dpad_left){
                armpivot.setPosition(0.5);
                upperpivot.setPosition(0.42);
                lowerpivot.setPosition(0.25);
                clawpivot.setPosition(0.8);
                liftArmToPosition(160);
            }
            // Rotate 90
            if (gamepad2.a){
                clawpivot.setPosition(0.8);
            }
            // Clip on bar setup
            // if (gamepad2.x) {
            //  liftArmToPosition(500);
            //  upperpivot.setPosition(0.5);
            //  //sleep(200);
            //  lowerpivot.setPosition(0.6);
            //  //sleep(200);
            //  clawpivot.setPosition(0.65);
            //  //sleep(200);
            //  armpivot.setPosition(0.5);
            //  liftArmToPosition(1650);
            //  }

            // Basket
            if (gamepad2.y) {
                // armpivot.setPosition(0.5);
                // lowerpivot.setPosition(0.9);
                // sleep(500);
                // upperpivot.setPosition(0.9);
                // sleep(500);
                // lowerpivot.setPosition(0.6);
                // clawpivot.setPosition(0.5);
                lowerpivot.setPosition(0.1);
                armpivot.setPosition(0.65);
                upperpivot.setPosition(0.9);
                sleep(900);
                lowerpivot.setPosition(0.55);
                clawpivot.setPosition(0.5);
            }

            // Pick from floor
            // if (gamepad2.dpad_down) {
            //	 armpivot.setPosition(0.8);
            //	 upperpivot.setPosition(0.55);
            //	 lowerpivot.setPosition(0.25);
            //	 clawpivot.setPosition(0.35);
            // }

            // Claw pivot left
            // if (gamepad2.dpad_left) {
            //	 clawPivotPosition = clawPivotPosition + 0.007;
            //	 clawpivot.setPosition(clawPivotPosition);

            //	 armPivotPosition = armPivotPosition - 0.005;
            //	 armpivot.setPosition(armPivotPosition);
            // }

            // Claw pivot right
            // if (gamepad2.dpad_right) {
            //	 clawPivotPosition = clawPivotPosition - 0.007;
            //	 clawpivot.setPosition(clawPivotPosition);

            //	 armPivotPosition = armPivotPosition + 0.005;
            //	 armpivot.setPosition(armPivotPosition);
            // }

            // if (gamepad2.a) {
            //	 clawpivot.setPosition(0.6);
            // }

            if (gamepad2.share) {
                // Reset encoders
                // liftarm.setPower(1);
                // liftarm2.setPower(1);
                // extentionarm.setPower(-1);
                // sleep(300);
                // liftarm.setPower(0);
                // liftarm2.setPower(0);
                // extentionarm.setPower(0);
                extentionarm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                liftarm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                liftarm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                extentionarm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                liftarm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                liftarm2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                myOtos.resetTracking();
            }


            // Manual extension arm
            if (Math.abs(gamepad2.left_stick_x) > 0) {
                extentionarm.setPower(gamepad2.left_stick_x);
            } else {
                extentionarm.setPower(0);
            }


            // If the stick is moved beyond the deadzone, override any active auto command.
            if (Math.abs(manualInput) > STICK_DEADZONE) {
                if (autoModeActive) {
                    // Cancel auto mode.
                    autoModeActive = false;
                    // Switch back to manual (open-loop) control.
                    liftarm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    liftarm2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }
                // Manual control (note: one motor’s power is inverted for opposing directions).
                liftarm.setPower(manualInput);
                liftarm2.setPower(manualInput);
            }
            // If no manual input is detected and auto mode is active, do nothing.
            // The motors will continue to run toward/hold their target.
            else if (autoModeActive) {
                // Optionally, you can monitor the motors’ progress:
                if (!liftarm.isBusy() && !liftarm2.isBusy()) {
                    // Auto target reached; you might choose to disable auto mode here
                    // or simply let the RUN_TO_POSITION behavior hold the position.
                }
            }
            // Neither manual input nor auto command active: stop the motors.
            else {
                liftarm.setPower(0);
                liftarm2.setPower(0);
            }


            // if (gamepad1.share) {
            //	 armpivot = null;
            //	 upperpivot = null;
            //	 lowerpivot = null;
            //	 clawpivot = null;
            //	 gripl = null;
            //	 gripr = null;
            //	 armpivot = hardwareMap.get(Servo.class, "armpivot");
            //	 upperpivot = hardwareMap.get(Servo.class, "upperpivot");
            //	 lowerpivot = hardwareMap.get(Servo.class, "lowerpivot");
            //	 clawpivot = hardwareMap.get(Servo.class, "clawpivot");
            //	 gripl = hardwareMap.get(CRServo.class, "gripl");
            //	 gripr = hardwareMap.get(CRServo.class, "gripr");
            // }

            if (gamepad1.dpad_up) {
                liftarm.setPower(1);
                liftarm2.setPower(1);
                extentionarm.setPower(-1);
                sleep(10000);
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

            // Reset the calculated values so the robot stops if no new values are calculated
            xF = 0;
            yF = 0;
            drive = 0;
            strafe = 0;
            turn = 0;

            // It is best practice to always call idle() at the bottom of your loops.
            // Yielding CPU Time: It gives time back to the system to process other tasks, such as communications and hardware updates.
            // Handling Stop Requests: It checks if a stop has been requested for your OpMode and facilitates a graceful shutdown if needed.
            // FTC SDK Compliance: It's a recommended practice in FTC programming to ensure your OpMode cooperates well with the overall control system.
            idle();
        }
    }


    private void liftArmToPosition(int targetPosition) {
        // Activate auto mode.
        autoModeActive = true;

        ((DcMotorEx) liftarm).setTargetPositionTolerance(1);
        ((DcMotorEx) liftarm2).setTargetPositionTolerance(1);

        liftarm.setTargetPosition(-targetPosition);
        liftarm2.setTargetPosition(targetPosition);

        liftarm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftarm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftarm.setPower(1);
        liftarm2.setPower(1);

        // ((DcMotorEx) liftarm).setVelocity(10000);
        // ((DcMotorEx) liftarm2).setVelocity(10000);

    }

}
