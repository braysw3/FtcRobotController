package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.teamcode.Datalogger;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.logging.Logger;
import java.util.logging.FileHandler;
import java.util.logging.SimpleFormatter;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;



@TeleOp(name="Logtest")
public class Logtest extends LinearOpMode {

    private Datalogger datalog;
    private Datalogger.GenericField df1;

    public void runOpMode(){

        df1 = new Datalogger.GenericField("targetX");

        datalog = new Datalogger.Builder()
                .setFilename("AndyFile.txt")
                .setFields(df1)
                .setAutoTimestamp(Datalogger.AutoTimestamp.DECIMAL_SECONDS)
                .build();

        waitForStart();

        while(opModeIsActive()){

            df1.set(1);
            datalog.writeLine();

        }
    }




}
