package org.firstinspires.ftc.teamcode.mechanism;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MotorTestBoard {
    private DcMotor motor;
    private double ticksPerRotation;

    public void init(HardwareMap hwMap) {
        motor = hwMap.get(DcMotor.class, "motor");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ticksPerRotation = motor.getMotorType().getTicksPerRev();
    }

    public void setMotorSpeed(double speed){

        motor.setPower(speed);
    }

    public double getMotorRotations(){

        return motor.getCurrentPosition()/ticksPerRotation;
    }
}
