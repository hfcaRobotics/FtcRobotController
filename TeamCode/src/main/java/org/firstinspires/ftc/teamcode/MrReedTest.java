package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class MrReedTest extends OpMode {
    @Override
    //Every OpMode needs and init and a loop method
    public void init() {
        telemetry.addData("Mr Reed", "Is HILARIOUS"); //How to make a comment

        // variables
        int teamNumber; //integer=a rounded number (8)
        double motorSpeed; //double=a floating number, not rounded (8.1233423)
        boolean clawClosed; //boolean=a true or false value (open)
        String name = "Roaring Gears";

        teamNumber = 16956;
        motorSpeed = 0.75;
        clawClosed = true;

        telemetry.addData("Team Number", teamNumber);
        telemetry.addData("Motor Speed", motorSpeed);
        telemetry.addData("Claw Position",clawClosed);
        telemetry.addData("Name", name);

    }

    @Override
    public void loop() {

    }
//single line comment
    /*
    multi-line comment
     */

}
