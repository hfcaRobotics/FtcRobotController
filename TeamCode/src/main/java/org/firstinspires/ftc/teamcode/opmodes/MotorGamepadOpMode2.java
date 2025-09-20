package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanism.MotorTestBoard;

@TeleOp
public class MotorGamepadOpMode2 extends OpMode {
    MotorTestBoard board = new MotorTestBoard();
    @Override
    public void init() {
        board.init(hardwareMap);
    }

    @Override
    public void loop(){
        double motorSpeed = gamepad1.left_stick_y;

        board.setMotorSpeed(motorSpeed);

        telemetry.addData("Motor rotations", board.getMotorRotations());
        telemetry.addData("Motor speed", motorSpeed);
    }
}
