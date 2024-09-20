package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@Config
@TeleOp(name = "Claw Test")
public class PIDClawTest extends LinearOpMode {
    private Telemetry mTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    private Motor mLeftClawMotor;
    private Motor.Encoder mLeftEncoder;

    private Motor mRightClawMotor;
    private Motor.Encoder mRightEncoder;

    private PIDFController leftController;
    private  PIDFController rightController;
    public static String motorName = "intakeLeft";
    public static boolean isReverse = false;
    public static double kP = 0.022;
    public static double kI = 0;
    public static double kD = 0;
    public static double setpoint = 0;
    public static boolean readOnly = true;

    @Override
    public void runOpMode() throws InterruptedException {
        mLeftClawMotor = new Motor(hardwareMap, motorName);
        mLeftClawMotor.stopAndResetEncoder();
        mLeftClawMotor.setInverted(false);
        mLeftClawMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        mLeftClawMotor.setRunMode(Motor.RunMode.RawPower);

        mLeftEncoder = mLeftClawMotor.encoder;

        mRightClawMotor = new Motor(hardwareMap, "intakeRight");
        mRightClawMotor.stopAndResetEncoder();
        mRightClawMotor.setInverted(true);
        mRightClawMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        mRightClawMotor.setRunMode(Motor.RunMode.RawPower);

        mRightEncoder = mRightClawMotor.encoder;

        leftController = new PIDController(kP, kI, kD);
        leftController.reset();
        leftController.setTolerance(5);

        rightController = new PIDController(kP, kI, kD);
        rightController.reset();
        rightController.setTolerance(5);

        waitForStart();

        while (opModeIsActive()) {
            leftController.setSetPoint(setpoint);
            rightController.setSetPoint(setpoint);

            if(!leftController.atSetPoint() && !readOnly) {
                mLeftClawMotor.set(leftController.calculate(mLeftEncoder.getDistance()));
            }
            else {
                mLeftClawMotor.set(0);
            }

            if(!rightController.atSetPoint() && !readOnly) {
                mRightClawMotor.set(rightController.calculate(mRightEncoder.getDistance()));
            }
            else{
                mRightClawMotor.set(0);
            }

            mTelemetry.addData("Setpoint", setpoint);
            mTelemetry.addData("Left Current Position", mLeftEncoder.getDistance());
            mTelemetry.addData("Right Current Position", mRightEncoder.getDistance());
            mTelemetry.addData("Is at Sepoint", leftController.atSetPoint());
            mTelemetry.update();
        }


    }
}
