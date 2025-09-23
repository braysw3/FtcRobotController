package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name = "Deadwheels")
public class Deadwheels extends LinearOpMode {

    public DcMotor leftEncoder, rightEncoder, frontEncoder;

    public void init(HardwareMap hwMap) {
        leftEncoder = hwMap.get(DcMotor.class, "leftEncoder");
        rightEncoder = hwMap.get(DcMotor.class, "rightEncoder");
        frontEncoder = hwMap.get(DcMotor.class, "frontEncoder");

        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    }

    public static final double TICKS_PER_REV = 8192;
    public static final double WHEEL_RADIUS = 1; // inches
    public static final double GEAR_RATIO = 1;

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }


    @Override
    public void runOpMode() {


        waitForStart();


        while (opModeIsActive()) {

            telemetry.addData("Left Encoder", leftEncoder.getCurrentPosition());
            telemetry.addData("Right Encoder", rightEncoder.getCurrentPosition());
            telemetry.addData("Front Encoder", frontEncoder.getCurrentPosition());
            telemetry.update();


        }
    }
}
