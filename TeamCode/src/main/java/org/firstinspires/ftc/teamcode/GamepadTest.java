package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class GamepadTest extends OpMode {

    @Override
    public void init() {

    }

    @Override
    public void loop() {
        telemetry.addData("x", gamepad1.left_stick_x);
        telemetry.addData("y", gamepad1.left_stick_y);
        telemetry.addData("a", gamepad1.a);
        telemetry.addData("b", gamepad1.b);
        telemetry.addData("right x", gamepad1.right_stick_x);
        telemetry.addData("right y", gamepad1.right_stick_y);
        telemetry.addData("right T", gamepad1.right_trigger);
        telemetry.addData("LEft T", gamepad1.left_trigger);

    }
}
