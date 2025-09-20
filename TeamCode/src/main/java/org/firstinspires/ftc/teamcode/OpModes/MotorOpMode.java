package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanism.MotorHardware;

@TeleOp
public class MotorOpMode extends OpMode {
    MotorHardware board = new MotorHardware();
    @Override
    public void init() {
        board.init(hardwareMap);
    }
    @Override
    public void loop(){
        double motorSpeed = -gamepad1.left_stick_y;
        board.setMotorSpeed(motorSpeed);
        //board.setMotorSpeed(-gamepad1.left_trigger);
        //board.setMotorSpeed(gamepad1.right_trigger);
    }
}
