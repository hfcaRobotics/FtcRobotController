package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanism.MotorTestBoard;

@TeleOp
public class MotorGamepadOpMode extends OpMode {
    MotorTestBoard board = new MotorTestBoard();
    @Override
    public void init() {
        board.init(hardwareMap);
    }

    @Override
    public void loop(){
        if(gamepad1.a) {
            board.setMotorSpeed(0.5);
        }
        else{
            board.setMotorSpeed(0.0);
        }
        telemetry.addData("Motor rotations", board.getMotorRotations());
    }
}
