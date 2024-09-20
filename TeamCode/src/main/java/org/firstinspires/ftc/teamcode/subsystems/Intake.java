package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Intake extends SubsystemBase {
    private final Motor mIntakeRight;
    private final Motor mIntakeLeft;
    private final Servo mRollerLeft; //Continuous
    private final Servo mRollerRight;
    private final Servo mArmLeft; //Continuous
    private final Servo mArmRight; //Continuous
    private final Servo mDoorLeft; //Position
    private final Servo mDoorRight; //Position

    private final PIDFController leftController = new PIDController(0.02, 0, 0);
    private final PIDFController rightController = new PIDController(0.01, 0, 0);

    private final ColorSensor mLeftColorSensor;
    private final ColorSensor mRightColorSensor;

    private IntakeState mIntakeState = IntakeState.STOW;

    private final ElapsedTime timer = new ElapsedTime();

    private ArmState armState = ArmState.RISING;
    private final TouchSensor upperMag;
    private final TouchSensor lowerMag;

    private boolean isBallCaught = false;
    private boolean hasStarted = false;
    private boolean isRised = false;

    private DoorState mLeftDoorState = DoorState.CLOSE;
    private DoorState mRightDoorState = DoorState.CLOSE;

    private TelemetryPacket packet = new TelemetryPacket();

    public Intake(final HardwareMap hardwareMap) {

        mIntakeLeft = new Motor(hardwareMap, "intakeLeft");
        mIntakeRight = new Motor(hardwareMap, "intakeRight");

        mRollerLeft = hardwareMap.get(Servo.class,"rollerLeft"); // 0 1 reverse 0 outtake 1 intake
        mRollerRight = hardwareMap.get(Servo.class, "rollerRight"); //0 1 0 outtake 1 intake

        mArmLeft = hardwareMap.get(Servo.class, "armLeft"); //0 1 0 down 1 up
        mArmRight = hardwareMap.get(Servo.class, "armRight");//0 1 reverse 0 down 1 up

        mDoorLeft = hardwareMap.get(Servo.class, "doorLeft"); //0 0.3 1
        mDoorRight = hardwareMap.get(Servo.class, "doorRight"); //0 0.25 1 reverse

        mLeftColorSensor = hardwareMap.get(ColorSensor.class, "colorSensorLeft");
        mRightColorSensor = hardwareMap.get(ColorSensor.class, "colorSensorRight");

        upperMag = hardwareMap.get(TouchSensor.class, "upperMagnetic");
        lowerMag = hardwareMap.get(TouchSensor.class, "lowerMagnetic");

        mIntakeLeft.setInverted(true);
        mIntakeRight.setInverted(false);

        mIntakeLeft.stopAndResetEncoder();
        mIntakeRight.stopAndResetEncoder();

        mIntakeLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        mIntakeRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        mIntakeLeft.setRunMode(Motor.RunMode.RawPower);
        mIntakeRight.setRunMode(Motor.RunMode.RawPower);

        mRollerLeft.setDirection(Servo.Direction.REVERSE);
        mRollerRight.setDirection(Servo.Direction.FORWARD);

        mArmLeft.setDirection(Servo.Direction.FORWARD);
        mArmRight.setDirection(Servo.Direction.REVERSE);

        mArmLeft.setPosition(0.5);
        mArmRight.setPosition(0.5);

        mDoorLeft.setDirection(Servo.Direction.REVERSE);
        mDoorRight.setDirection(Servo.Direction.REVERSE);

        mLeftColorSensor.enableLed(true);
        mRightColorSensor.enableLed(true);

        leftController.setTolerance(2);
        rightController.setTolerance(2);

        resetController();
        setIntakePosition(IntakeState.STOW);
    }

    public void setIntakePower(double power) {
        mIntakeLeft.set(power);
        mIntakeRight.set(power);
    }


    public void resetController() {
        leftController.reset();
        rightController.reset();
    }

    public void setIntakePosition(IntakeState intakeState) {
        this.mIntakeState = intakeState;
        leftController.setSetPoint(intakeState.leftPosition);
        rightController.setSetPoint(intakeState.rightPosition);
    }

    public void switchIntakeState() {
        switch (mIntakeState) {
            case STOW:
                setIntakePosition(IntakeState.PUSH);
                timer.reset();
                break;

            case PUSH:
                setIntakePosition(IntakeState.STOW);
                break;
        }
    }

    public void moveIntakeToPosition() {
        boolean leftColorBallDetected = mLeftColorSensor.red() >= 200
                || mLeftColorSensor.green() >= 200
                || mLeftColorSensor.blue() >= 200;

        boolean rightColorBallDetected = mRightColorSensor.red() >= 200
                || mRightColorSensor.green() >= 200
                || mRightColorSensor.blue() >= 200;

        if((leftColorBallDetected || rightColorBallDetected)
                && mIntakeState == IntakeState.PUSH
                && (timer.seconds() > 0.5)
                && !isBallCaught
        ) {
            setIntakePosition(IntakeState.STOW);
            isBallCaught = true;
        }

        if(!(leftColorBallDetected || rightColorBallDetected)) isBallCaught = false;

        mIntakeLeft.set(leftController.calculate(mIntakeLeft.encoder.getDistance()));
        mIntakeRight.set(rightController.calculate(mIntakeRight.encoder.getDistance()));
    }

    public void setRollerPower(double power) {
        mRollerLeft.setPosition(power);
        mRollerRight.setPosition(power);
    }

    public void setArmPower(double power) {
        mArmLeft.setPosition(power);
        mArmRight.setPosition(power);
    }

    public void setArmState(ArmState armState) {
        this.armState = armState;
    }

    public void moveArmPosition() {
        switch (armState) {
            case IDLE:
                mArmLeft.setPosition(0.5);
                mArmRight.setPosition(0.5);
                break;
            case RISING:
                riseArm();
                break;
            case FALLING:
                stowArm();
                break;
        }
    }

    public void riseArm() {
        //armState = ArmState.RISED;
        if(!upperMag.isPressed()) {
            mArmLeft.setPosition(1);
            mArmRight.setPosition(1);
        }
        else {
            mArmLeft.setPosition(0.5);
            mArmRight.setPosition(0.5);
            isRised = true;
            armState = ArmState.IDLE;
        }
    }

    public void stowArm() {
        if(!lowerMag.isPressed()) {
            mArmLeft.setPosition(-0.6);
            mArmRight.setPosition(-0.6);
        }
        else {
            mArmLeft.setPosition(0.5);
            mArmRight.setPosition(0.5);
            armState = ArmState.IDLE;
        }
    }


    public boolean getUpperMagPressed() {
        return upperMag.isPressed();
    }

    public void switchLeftDoorState() {
        switch (mLeftDoorState){
            case OPEN:
                mLeftDoorState = DoorState.CLOSE;
                mDoorLeft.setPosition(0.7);
                break;
            case CLOSE:
                mLeftDoorState = DoorState.OPEN;
                mDoorLeft.setPosition(0.3);
        }
    }

    public void switchRightDoorState() {
        switch (mRightDoorState){
            case OPEN:
                mRightDoorState = DoorState.CLOSE;
                mDoorRight.setPosition(0.4);
                break;
            case CLOSE:
                mRightDoorState = DoorState.OPEN;
                mDoorRight.setPosition(0.8);
        }
    }

    public void initializeIntakeSystem() {
        if(!hasStarted) {
            mDoorLeft.setPosition(0.7);
            mDoorRight.setPosition(0.4);
            hasStarted = true;
        }
    }

    public enum ArmState {
        RISING, FALLING, IDLE;
    }


    public enum DoorState {
        IDLE(0.22, 0.95),
        OPEN(0.3, 0.8),
        CLOSE(0.70, 0.4);

        private final double leftPosition;
        private final double rightPosition;
        DoorState(double leftPosition, double rightPosition) {
            this.leftPosition = leftPosition;
            this.rightPosition = rightPosition;
        }
    }

    public enum IntakeState{
        STOW(1, 1),
        PUSH(145, 150),
        GRAB(45, 45);
        private final double leftPosition;
        private final double rightPosition;
        IntakeState(double leftPosition, double rightPosition) {
            this.rightPosition = rightPosition;
            this.leftPosition = leftPosition;
        }

        @Override
        public String toString() {
            switch (this) {
                case GRAB:
                    return "GRAB";
                case PUSH:
                    return "PUSH";
                case STOW:
                    return "STOW";
                default:
                    return "";
            }
        }
    }




    @Override
    public void periodic() {
        initializeIntakeSystem();
        moveArmPosition();
        moveIntakeToPosition();

        packet.put("Is Upper Trigger", upperMag.isPressed());
        packet.put("Intake State", mIntakeState.toString());
        packet.put("Has ball", isBallCaught);
        packet.put("Left SetPoint", leftController.getSetPoint());
        packet.put("Right SetPoint", rightController.getSetPoint());
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}
