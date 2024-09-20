package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.util.MathUtil;
import org.firstinspires.ftc.teamcode.subsystems.TankDrive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TankDriveCommand extends CommandBase {
    private final TankDrive tankDrive;
    private final DoubleSupplier driveSupplier;
    private final DoubleSupplier turnSupplier;
    private final BooleanSupplier isSlowMode;

    public TankDriveCommand(TankDrive tank, DoubleSupplier drive,
                            DoubleSupplier turn, BooleanSupplier slow) {
        tankDrive = tank;
        driveSupplier = drive;
        turnSupplier = turn;
        isSlowMode = slow;

        addRequirements(tankDrive);
    }

    @Override
    public void execute() {
        double drivePower = driveSupplier.getAsDouble();
        double turnPower = turnSupplier.getAsDouble() * 0.8;

        if(MathUtil.isNear(0, drivePower, 0.001)) {
            turnPower = turnPower * 0.8;
        }

        if(isSlowMode.getAsBoolean()) {
            drivePower = drivePower * 0.4;
            turnPower = turnPower * 0.8;
        }

        tankDrive.moveRobot(
                drivePower,
                turnPower
        );
    }
}
