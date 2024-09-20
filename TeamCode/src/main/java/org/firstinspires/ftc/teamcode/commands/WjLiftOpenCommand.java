package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Lift;

import java.util.function.DoubleSupplier;

public class WjLiftOpenCommand extends CommandBase {
    private final Lift lift;
    private final DoubleSupplier frontSupplier;
    private final DoubleSupplier bothSupplier;

    public WjLiftOpenCommand(Lift lift, DoubleSupplier front, DoubleSupplier both) {
        this.lift = lift;
        this.frontSupplier = front;
        this.bothSupplier = both;

        addRequirements(lift);
    }

    @Override
    public void execute() {
        double front = frontSupplier.getAsDouble();
        double both = bothSupplier.getAsDouble();

        lift.setPower(front + both, both);
    }
}
