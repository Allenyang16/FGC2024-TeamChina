package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.kinematics.DifferentialOdometry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


public class TankDrive extends SubsystemBase {
    private final Motor leftDriveFront;
    private final Motor rightDriveFront;
    private final IMU imu;
    private final DifferentialOdometry odometry;


    public TankDrive (final HardwareMap hardwareMap) {
        leftDriveFront  = new Motor(hardwareMap, "left_drive");
        rightDriveFront = new Motor(hardwareMap, "right_drive");

        leftDriveFront.setInverted(true);
        rightDriveFront.setInverted(false);
        leftDriveFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightDriveFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        leftDriveFront.setRunMode(Motor.RunMode.RawPower);
        rightDriveFront.setRunMode(Motor.RunMode.RawPower);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        odometry = new DifferentialOdometry(new Pose2d(
                0, 0, Rotation2d.fromDegrees(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES))
        ), 16.9291);


        //rightDriveBack.setDirection(DcMotorEx.Direction.FORWARD);
        //strafeDrive.setDirection(DcMotorEx.Direction.FORWARD);
    }

    public void moveRobot(double x, double yaw) {
        // Calculate left and right wheel powers.
        double leftPower    = x - yaw;
        double rightPower   = x + yaw;

        // Normalize wheel powers to be less than 1.0
        leftPower = Range.clip(leftPower, -1, 1);
        rightPower = Range.clip(rightPower, -1, 1);

        // Send powers to the wheels.
        leftDriveFront.set(leftPower);
        //leftDriveBack.setPower(leftPower);
        rightDriveFront.set(rightPower);
        //rightDriveBack.setPower(rightPower);
    }

    public Pose2d getPose() {
        return odometry.getPose();
    }

    @Override
    public void periodic() {
        odometry.updatePosition(leftDriveFront.getDistance(), rightDriveFront.getDistance());
    }
}
