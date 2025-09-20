package org.firstinspires.ftc.teamcode.mechanism;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MechanumDriveMechs {
    private DcMotor frontLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backLeftDrive;
    private DcMotor backRightDrive;

    IMU imu;

    public void init(HardwareMap hwMap) {

        frontLeftDrive = hwMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hwMap.get(DcMotor.class, "front_right_drive");
        backLeftDrive = hwMap.get(DcMotor.class, "back_left_drive");
        backRightDrive = hwMap.get(DcMotor.class, "back_right_drive");

        // This assumes we will have encoder cables hooked up
        // need to remove or comment out if we do not hook up the encoders

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Will have to mess with these setting to reverse the motors where needed
        // in order to make the Mechanum drive function properly

        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);


        imu = hwMap.get(IMU.class, "imu");
        // This needs to be changed to match the orientation on your robot
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    public void reset_yaw(){
            imu.resetYaw();
    }

    public double getimuYaw(){
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }
    public void FrontLeftSpeed(double speed){

        frontLeftDrive.setPower(speed);
    }

    public void FrontRightSpeed(double speed){

        frontRightDrive.setPower(speed);
    }
    public void BackLeftSpeed(double speed){

        backLeftDrive.setPower(speed);
    }

    public void BackRightSpeed(double speed){

        backRightDrive.setPower(speed);
    }

}
