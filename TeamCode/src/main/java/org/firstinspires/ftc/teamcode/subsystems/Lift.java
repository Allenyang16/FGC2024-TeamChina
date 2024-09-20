package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

public class Lift extends SubsystemBase {
    private final Motor mFrontLeftSlide;
    private final Motor mBackLeftSlide;
    private final Motor mFrontRightSlide;
    private final Motor mBackRightSlide;

    private final TouchSensor mFrontTouchSensor;
    private final TouchSensor mBackTouchSensor;

    private LiftState mFrontLiftState = LiftState.IDLE;
    private LiftState mBackLiftState = LiftState.IDLE;

    private ReleaseDirection releaseDirection = ReleaseDirection.IDLE;
    private ReleaseLevel releaseLevel = ReleaseLevel.ORIGIN;

    private TelemetryPacket packet = new TelemetryPacket();

    private final double slideKp = 0.1;
    private final double positionTolerance = 10;
    private final boolean shouldLimitHeight = true;

    //private final ElevatorFeedforward ff = new ElevatorFeedforward(0, 0, 0, 0);

    public Lift(final HardwareMap hardwareMap) {
    mFrontLeftSlide = new Motor(hardwareMap, "frontLeftLift", 28, 6000);
        mBackLeftSlide = new Motor(hardwareMap, "backLeftLift", 28, 6000);
        mFrontRightSlide = new Motor(hardwareMap, "frontRightLift", 28, 6000);
        mBackRightSlide = new Motor(hardwareMap, "backRightLift", 28, 6000);

        mFrontTouchSensor = hardwareMap.get(TouchSensor.class, "frontSensor");
        mBackTouchSensor = hardwareMap.get(TouchSensor.class, "backSensor");

        mFrontLeftSlide.setInverted(false);
        mBackLeftSlide.setInverted(false);
        mFrontRightSlide.setInverted(false);
        mBackRightSlide.setInverted(true);

        mFrontLeftSlide.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        mBackLeftSlide.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        mFrontRightSlide.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        mBackRightSlide.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        mFrontLeftSlide.setPositionCoefficient(slideKp);
        mBackLeftSlide.setPositionCoefficient(slideKp);
        mFrontRightSlide.setPositionCoefficient(slideKp);
        mBackRightSlide.setPositionCoefficient(slideKp);

        mFrontLeftSlide.setPositionTolerance(positionTolerance);
        mBackLeftSlide.setPositionCoefficient(positionTolerance);
        mFrontRightSlide.setPositionCoefficient(positionTolerance);
        mBackRightSlide.setPositionCoefficient(positionTolerance);

        stopAndResetEncoder();
        mFrontLeftSlide.setTargetDistance(0);
        mBackLeftSlide.setTargetDistance(0);
        mFrontRightSlide.setTargetDistance(0);
        mBackRightSlide.setTargetDistance(0);
    }

    public void setReleaseDirection(ReleaseDirection direction) {
        releaseDirection = direction;
        //updateCurrentState();
    }

    public void setReleaseLevel(ReleaseLevel level) {
        releaseLevel = level;
        //updateCurrentState();
    }

    public void liftUp() {
        switch (releaseLevel) {
            case ORIGIN:
                releaseLevel = ReleaseLevel.LOW;
                packet.put("yes", "You trigger the ORIGIN");
                return;
            case LOW:
                releaseLevel = ReleaseLevel.MID;
                packet.put("yes", "You trigger the LOW");
                return;
            case MID:
                releaseLevel = ReleaseLevel.HIGH;
                packet.put("yes", "You trigger the MID");
                return;
            case HIGH:
                releaseLevel = ReleaseLevel.HIGH;
                packet.put("yes", "You trigger the HIGH");
        }
        //updateCurrentState();
    }

    public void fallDown() {
        switch (releaseLevel) {
            case ORIGIN:
                releaseLevel = ReleaseLevel.ORIGIN;
                packet.put("yes", "You trigger the ORIGIN");
                return;
            case LOW:
                releaseLevel = ReleaseLevel.ORIGIN;
                packet.put("yes", "You trigger the LOW");
                return;
            case MID:
                releaseLevel = ReleaseLevel.LOW;
                packet.put("yes", "You trigger the MID");
                return;
            case HIGH:
                releaseLevel = ReleaseLevel.MID;
                packet.put("yes", "You trigger the HIGH");
        }
        //updateCurrentState();
    }

    public void updateCurrentState() {
        switch (releaseLevel) {
            case ORIGIN:
                setOriginal();
                return;
            case LOW:
                setLowRelease(releaseDirection);
                return;
            case MID:
                setMidRelease(releaseDirection);
                return;
            case HIGH:
                setHighRelease(releaseDirection);
        }
        //TODO Need to consider whether to add this
        setPositionMode();
    }

    public void setLowRelease(ReleaseDirection direction) {
        switch (direction) {
            case FRONT:
                mFrontLiftState = LiftState.LOW_GOAL_LOW;
                mBackLiftState = LiftState.LOW_GOAL_HIGH;
                return;
            case BACK:
                mFrontLiftState = LiftState.LOW_GOAL_HIGH;
                mBackLiftState = LiftState.LOW_GOAL_LOW;
                return;
            case IDLE:
                mFrontLiftState = LiftState.LOW_GOAL_LOW;
                mBackLiftState = LiftState.LOW_GOAL_LOW;
        }
    }

    public void setMidRelease(ReleaseDirection direction) {
        switch (direction) {
            case FRONT:
                mFrontLiftState = LiftState.MID_GOAL_LOW;
                mBackLiftState = LiftState.MID_GOAL_HIGH;
                return;
            case BACK:
                mFrontLiftState = LiftState.MID_GOAL_HIGH;
                mBackLiftState = LiftState.MID_GOAL_LOW;
                return;
            case IDLE:
                mFrontLiftState = LiftState.MID_GOAL_LOW;
                mBackLiftState = LiftState.MID_GOAL_LOW;
        }
    }

    public void setHighRelease(ReleaseDirection direction) {
        switch (direction) {
            case FRONT:
                mFrontLiftState = LiftState.HIGH_GOAL_LOW;
                mBackLiftState = LiftState.HIGH_GOAL_HIGH;
                return;
            case BACK:
                mFrontLiftState = LiftState.HIGH_GOAL_HIGH;
                mBackLiftState = LiftState.HIGH_GOAL_LOW;
                return;
            case IDLE:
                mFrontLiftState = LiftState.HIGH_GOAL_LOW;
                mBackLiftState = LiftState.HIGH_GOAL_LOW;
        }
    }

    public void setOriginal() {
        mFrontLiftState = LiftState.ORIGINAL;
        mBackLiftState = LiftState.ORIGINAL;
    }

    public void stopMotor() {
        mFrontLiftState = LiftState.IDLE;
        mBackLiftState = LiftState.IDLE;
    }

    public void setPositionMode() {
        mFrontLeftSlide.setRunMode(Motor.RunMode.PositionControl);
        mBackLeftSlide.setRunMode(Motor.RunMode.PositionControl);
        mFrontRightSlide.setRunMode(Motor.RunMode.PositionControl);
        mBackRightSlide.setRunMode(Motor.RunMode.PositionControl);
    }

    public void stopAndResetEncoder() {
        mFrontLeftSlide.stopAndResetEncoder();
        mBackLeftSlide.stopAndResetEncoder();
        mFrontRightSlide.stopAndResetEncoder();
        mBackRightSlide.stopAndResetEncoder();
    }

    public void setPower(double frontPower, double backPower) {
        mFrontLeftSlide.setRunMode(Motor.RunMode.RawPower);
        mBackLeftSlide.setRunMode(Motor.RunMode.RawPower);
        mFrontRightSlide.setRunMode(Motor.RunMode.RawPower);
        mBackRightSlide.setRunMode(Motor.RunMode.RawPower);

        frontPower = Range.clip(frontPower, -1, 1);
        backPower = Range.clip(backPower, -1, 1);

        boolean limitFrontHigh = mFrontLeftSlide.getDistance() >= 4000 || mFrontRightSlide.getDistance() >= 4000;
        boolean limitBackHigh = mBackLeftSlide.getDistance() >= 3500 || mBackRightSlide.getDistance() >= 3500;
        boolean limitFrontLow = mFrontTouchSensor.isPressed();
        boolean limitBackLow = mBackTouchSensor.isPressed();

        packet.put("Limit Front High", limitFrontHigh);
        packet.put("Limit Back High", limitBackHigh);

        if(frontPower > 0 && limitFrontHigh)
            setFrontSlidesPower(0);
        else if(backPower > 0 && limitBackHigh) {
            setBackSlidesPower(0);
        }
        else if(frontPower < 0 && limitFrontLow){
            setFrontSlidesPower(0);
        }
        else if(backPower < 0 && limitBackLow){
            setBackSlidesPower(0);
        }
        else {
            setFrontSlidesPower(frontPower);
            setBackSlidesPower(backPower);
        }

//        if(Math.abs(mFrontLeftSlide.getDistance() - mBackLeftSlide.getDistance()) > 1000) {
//            return;
//        }
//
//        if(frontPower <= 0) {
//            mFrontLeftSlide.set(frontPower);
//            mFrontRightSlide.set(frontPower);
//        }
//        else {
//            if(mFrontLeftSlide.getDistance() < 3870 && mFrontRightSlide.getDistance() < 3870
//                    && mFrontLeftSlide.getDistance() > -200 && mFrontRightSlide.getDistance() > -200
//                    && shouldLimitHeight){
//                mFrontLeftSlide.set(frontPower);
//                mFrontRightSlide.set(frontPower);
//            }
//        }
//
//        if(backPower <= 0) {
//            mBackLeftSlide.set(backPower);
//            mBackRightSlide.set(backPower);
//        }
//        else {
//            if(mBackLeftSlide.getDistance() < 4120 && mBackRightSlide.getDistance() < 4120
//                    && mBackLeftSlide.getDistance() > -200 && mBackRightSlide.getDistance() > -200
//                    && shouldLimitHeight) {
//                mBackLeftSlide.set(backPower);
//                mBackRightSlide.set(backPower);
//            }
//        }
    }

    public void resetEncoders() {
        resetFrontEncoders();
        resetBackEncoders();
    }

    public void resetFrontEncoders() {
        mFrontLeftSlide.resetEncoder();
        mFrontRightSlide.resetEncoder();
    }

    public void resetBackEncoders() {
        mBackLeftSlide.resetEncoder();
        mBackRightSlide.resetEncoder();
    }


    public void setFrontSlidesPower(double frontPower) {
        mFrontLeftSlide.set(frontPower);
        mFrontRightSlide.set(frontPower);
    }

    public void setBackSlidesPower(double backPower) {
        mBackLeftSlide.set(backPower);
        mBackRightSlide.set(backPower);
    }

    public void setFrontLiftsPosPower(double power) {
        if(!mFrontLeftSlide.atTargetPosition()) {
            mFrontLeftSlide.set(power);
            mFrontRightSlide.set(power);
        }
        else {
            mFrontLeftSlide.set(0);
            mFrontRightSlide.set(0);
        }
    }

    public void setBackLiftsPosPower(double power) {
        if(!mBackLeftSlide.atTargetPosition()) {
            mBackLeftSlide.set(power);
            mBackRightSlide.set(power);
        }
        else {
            mBackLeftSlide.set(0);
            mBackRightSlide.set(0);
        }
    }

    private void setFrontLiftsDistance(double distance) {
        mFrontLeftSlide.setTargetDistance(distance);
        mFrontRightSlide.setTargetDistance(distance);
    }

    private void setBackLiftsDistance(double distance) {
        mBackLeftSlide.setTargetDistance(distance);
        mBackRightSlide.setTargetDistance(distance);
    }


    public enum LiftState{
        IDLE(0, 0),
        ORIGINAL(0, 0.3),
        LOW_GOAL_LOW(469, 0.3),
        LOW_GOAL_HIGH(747, 0.3),
        MID_GOAL_LOW(659, 0.3),
        MID_GOAL_HIGH(931, 0.3),
        HIGH_GOAL_LOW(800, 0.3),
        HIGH_GOAL_HIGH(1291, 0.3);

        private final double height;
        private final double power;
        private LiftState(double height, double power) {
            this.height = height;
            this.power = power;
        }
    }

    public enum ReleaseDirection {
        FRONT, BACK, IDLE;

        @Override
        public String toString() {
            switch (this) {
                case FRONT:
                    return "FRONT";
                case BACK:
                    return "BACK";
                case IDLE:
                    return "IDLE";
                default:
                    return "WRONG STATE";
            }
        }
    }

    public enum ReleaseLevel {
        ORIGIN, LOW, MID, HIGH;

        @Override
        public String toString() {
            switch (this) {
                case ORIGIN:
                    return "ORIGIN";
                case LOW:
                    return "LOW";
                case MID:
                    return "MID";
                case HIGH:
                    return "HIGH";
                default:
                    return "WRONG STATE";
            }
        }
    }


    @Override
    public void periodic(){
//        updateCurrentState();
//
//        setFrontLiftsDistance(mFrontLiftState.height);
//        setBackLiftsDistance(mBackLiftState.height);
//
//        setFrontLiftsPosPower(mFrontLiftState.power);
//        setBackLiftsPosPower(mBackLiftState.power);

        if(mFrontTouchSensor.isPressed()) {
            resetFrontEncoders();
        }

        if(mBackTouchSensor.isPressed()) {
            resetBackEncoders();
        }

//        packet.put("Direction", releaseDirection);
//        packet.put("Level", releaseLevel);
//        packet.put("Front Height", mFrontLiftState.height);
//        packet.put("Back Height", mBackLiftState.height);

        packet.put("Front Pressed", mFrontTouchSensor.isPressed());
        packet.put("Back Pressed", mBackTouchSensor.isPressed());

        packet.put("FrontLeftPos", mFrontLeftSlide.getCurrentPosition());
        packet.put("FrontRightPos", mFrontRightSlide.getCurrentPosition());
        packet.put("BackLeftPos", mBackLeftSlide.getCurrentPosition());
        packet.put("BackRightPos", mBackRightSlide.getCurrentPosition());

        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}
