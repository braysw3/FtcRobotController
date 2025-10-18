package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="What_up")
public class What_up extends LinearOpMode {

    private static final double STICK_DEADZONE = 0;
    private DcMotor frontleft = null;
    private DcMotor frontright = null;
    private DcMotor rearleft = null;
    private DcMotor rearright = null;
    private double frontleftpower = 0;
    private double frontrightpower = 0;
    private double rearrightpower = 0;
    private double rearleftpower = 0;
    @Override
    public void runOpMode() {
        frontleft = hardwareMap.get(DcMotor.class, "frontleft");  // Control Hub - Motors - 2 - REV Robotics UltraPlanetary HD Hex Motor
        frontright = hardwareMap.get(DcMotor.class, "frontright"); // Control Hub - Motors - 1 - REV Robotics UltraPlanetary HD Hex Motor
        frontleft = hardwareMap.get(DcMotor.class, "rearleft");
        frontleft = hardwareMap.get(DcMotor.class, "rearright");
        waitForStart();

        while (opModeIsActive()) {

            if (Math.abs(gamepad1.left_stick_x) + Math.abs(gamepad1.left_stick_y) + Math.abs(gamepad1.right_stick_x) > 0)
            {

                // xF, yF from left stick (field-centric motion)
                double xF = gamepad1.left_stick_x;   // Field X command
                double yF = -gamepad1.left_stick_y;  // Field Y command (invert to make up positive)

                // Apply deadzone to the joysticks
                if (Math.abs(gamepad1.left_stick_x) < STICK_DEADZONE) xF = 0;
                if (Math.abs(gamepad1.left_stick_y) < STICK_DEADZONE) yF = 0;


                // Normalize wheel powers
                double maxMagnitude = Math.max(
                        Math.max(Math.abs(frontleftpower), Math.abs(frontrightpower)),
                        Math.max(Math.abs(rearleftpower), Math.abs(rearrightpower)));

                if (maxMagnitude > 1.0)
                {
                    frontleftpower  /= maxMagnitude;
                    frontrightpower /= maxMagnitude;

                }



        }


    }
}}

