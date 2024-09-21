package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.TankDriveCommand;
import org.firstinspires.ftc.teamcode.commands.WjLiftOpenCommand;
import org.firstinspires.ftc.teamcode.common.util.SlewRateLimiter;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.TankDrive;

@TeleOp(name = "FGC 2024 CommandOP")
public class FGC2024TeleOp extends CommandOpMode {
    private TriggerReader triggerReader;
    private SlewRateLimiter driverLimiter;
    private SlewRateLimiter turnLimiter;

    //private List<LynxModule> allHubs;

    private TankDrive tankDrive;
    private Lift lift;
    private Intake intake;
    private GamepadEx gamepadEx1, gamepadEx2;



    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

//        for(LynxModule hub : allHubs) {
//            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//        }

        //Subsystems Initialization
        tankDrive = new TankDrive(hardwareMap);
        lift = new Lift(hardwareMap);
        intake = new Intake(hardwareMap);

        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        driverLimiter = new SlewRateLimiter(4);
        turnLimiter = new SlewRateLimiter(3);

    tankDrive.setDefaultCommand(
        new TankDriveCommand(
            tankDrive,
            () -> -driverLimiter.calculate(gamepadEx1.getLeftY()) ,
            () -> gamepadEx1.getRightX(),
            lift::shouldSlowDrive));

    //        lift.setDefaultCommand(new LiftOpenLoopCommand(
    //                lift, () -> gamepadEx2.getLeftY(), () -> -gamepadEx2.getRightY(),
    //                () -> intake.getUpperMagPressed()
    //        ));

    lift.setDefaultCommand(new WjLiftOpenCommand(
            lift, () -> gamepadEx2.getLeftY(),
            () -> -gamepadEx2.getRightY()
    ));

    gamepadEx2
        .getGamepadButton(GamepadKeys.Button.B)
        .whenPressed(new InstantCommand(() -> intake.setArmState(Intake.ArmState.RISING)))
        .whenReleased(new InstantCommand(() -> intake.setArmState(Intake.ArmState.IDLE)));

    gamepadEx2
        .getGamepadButton(GamepadKeys.Button.A)
        .whenPressed(new InstantCommand(() -> intake.setArmState(Intake.ArmState.FALLING)))
        .whenReleased(new InstantCommand(() -> intake.setArmState(Intake.ArmState.IDLE)));

    gamepadEx2
        .getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
        .whenPressed(
                new InstantCommand(
                        () -> {
                            intake.setRollerPower(1);
                        }
                ))
        .whenReleased(
                new InstantCommand(
                        () -> {
                            intake.setRollerPower(0.5);
                        }
        ));

    gamepadEx2
            .getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
            .whenPressed(
                    new InstantCommand(() -> {
                        intake.setRollerPower(0);
                    }
                    ))
            .whenReleased(new InstantCommand(
                    () -> {
                        intake.setRollerPower(0.5);
                    }
            ));

    gamepadEx2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(new InstantCommand(
            () -> intake.switchLeftDoorState()
    ));

        gamepadEx2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new InstantCommand(
                () -> intake.switchRightDoorState()
        ));

        gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new InstantCommand(() -> intake.switchIntakeState())
        );

//        gamepadEx1.getGamepadButton(GamepadKeys.Button.B).whenPressed(
//                new InstantCommand(() -> intake.setIntakePosition(Intake.IntakeState.GRAB))
//        );
//
//        gamepadEx1.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
//                new InstantCommand(() -> intake.setIntakePosition(Intake.IntakeState.PUSH))
//        );

        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new InstantCommand(() -> lift.resetEncoders())
        );



        //Buttons Binding


    }

    @Override
    public void run() {
//        for(LynxModule hub : allHubs) {
//            hub.clearBulkCache();
//        }
        CommandScheduler.getInstance().run();
    }



    public int getDPADAngle(GamepadEx gamepad) {
        if(gamepad == null) return -1;
        boolean[] buttonArray = {
                gamepad.getButton(GamepadKeys.Button.DPAD_LEFT),
                gamepad.getButton(GamepadKeys.Button.DPAD_RIGHT),
                gamepad.getButton(GamepadKeys.Button.DPAD_UP),
                gamepad.getButton(GamepadKeys.Button.DPAD_DOWN)
        };
        if(buttonArray[0] && !buttonArray[1] && !buttonArray[2] && !buttonArray[3]) {
            return 180;
        }
        if(!buttonArray[0] && buttonArray[1] && buttonArray[2] && buttonArray[3]) {
            return 0;
        }
        if(!buttonArray[0] && !buttonArray[1] && buttonArray[2] && !buttonArray[3]) {
            return 90;
        }
        if(!buttonArray[0] && !buttonArray[1] && !buttonArray[2] && buttonArray[3]) {
            return 270;
        }
        if(buttonArray[0] && !buttonArray[1] && buttonArray[2] && !buttonArray[3]) {
            return 135;
        }
        if(buttonArray[0] && !buttonArray[1] && !buttonArray[2] && buttonArray[3]) {
            return 225;
        }
        if(!buttonArray[0] && buttonArray[1] && buttonArray[2] && !buttonArray[3]) {
            return 45;
        }
        if(buttonArray[0] && buttonArray[1] && !buttonArray[2] && buttonArray[3]) {
            return 315;
        }

        return -1;
    }



}
