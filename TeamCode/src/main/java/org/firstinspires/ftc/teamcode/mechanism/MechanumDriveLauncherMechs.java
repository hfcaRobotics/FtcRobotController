package org.firstinspires.ftc.teamcode.mechanism;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MechanumDriveLauncherMechs {
    public DcMotor frontLeftDrive;
    public DcMotor frontRightDrive;
    public DcMotor backLeftDrive;
    public DcMotor backRightDrive;
    public DcMotorEx launcher;
    public CRServo leftFeeder;
    public CRServo rightFeeder;


    IMU imu;

    public void init(HardwareMap hwMap) {

        frontLeftDrive = hwMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hwMap.get(DcMotor.class, "front_right_drive");
        backLeftDrive = hwMap.get(DcMotor.class, "back_left_drive");
        backRightDrive = hwMap.get(DcMotor.class, "back_right_drive");
        launcher = hwMap.get(DcMotorEx.class, "launcher");
        leftFeeder = hwMap.get(CRServo.class, "left_feeder");
        rightFeeder = hwMap.get(CRServo.class, "right_feeder");

        // This assumes we will have encoder cables hooked up
        // need to remove or comment out if we do not hook up the encoders

       //frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Will have to mess with these setting to reverse the motors where needed
        // in order to make the Mechanum drive function properly

        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);


        imu = hwMap.get(IMU.class, "imu");
        // This needs to be changed to match the orientation on your robot
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.UP;

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
    public void drive(double forward, double right, double rotate) {
        // This calculates the power needed for each wheel based on the amount of forward,
        // strafe right, and rotate
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        double maxPower = 1.0;
        double maxSpeed = 1.0;  // make this slower for outreaches

        // This is needed to make sure we don't pass > 1.0 to any wheel
        // It allows us to keep all of the motors in proportion to what they should
        // be and not get clipped
        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));

        // We multiply by maxSpeed so that it can be set lower for outreaches
        // When a young child is driving the robot, we may not want to allow full
        // speed.
        FrontLeftSpeed(maxSpeed * (frontLeftPower / maxPower));
        FrontRightSpeed(maxSpeed * (frontRightPower / maxPower));
        BackLeftSpeed(maxSpeed * (backLeftPower / maxPower));
        BackRightSpeed(maxSpeed * (backRightPower / maxPower));
    }
    public void driveFieldRelative(double forward, double right, double rotate) {
        // First, convert direction being asked to drive to polar coordinates
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        // Second, rotate angle by the angle the robot is pointing
        theta = AngleUnit.normalizeRadians(theta - getimuYaw());

        // Third, convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        // Finally, call the drive method with robot relative forward and right amounts
        drive(newForward, newRight, rotate);
    }

}
