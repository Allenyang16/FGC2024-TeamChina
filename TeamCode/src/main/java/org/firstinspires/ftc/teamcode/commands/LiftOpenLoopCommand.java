package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Lift;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class LiftOpenLoopCommand extends CommandBase {
    private final Lift lift;
    private final DoubleSupplier frontSupplier;
    private final DoubleSupplier backSupplier;

    public LiftOpenLoopCommand(Lift lift, DoubleSupplier front, DoubleSupplier back, BooleanSupplier upperMagPressed) {
        this.lift = lift;
        this.frontSupplier = front;
        this.backSupplier = back;

        addRequirements(lift);
    }

    @Override
    public void execute() {
        lift.setPower(frontSupplier.getAsDouble(), backSupplier.getAsDouble());
    }
}
