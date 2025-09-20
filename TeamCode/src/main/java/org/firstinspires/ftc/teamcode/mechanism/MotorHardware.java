package org.firstinspires.ftc.teamcode.mechanism;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MotorHardware {
    private DcMotor motor;

    public void init(HardwareMap hwMap){
        motor = hwMap.get(DcMotor.class,"motor");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void setMotorSpeed(double speed) {
        motor.setPower(speed);
    }
}
