package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Lift;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class WjLiftOpenCommand extends CommandBase {
    private final Lift lift;
    private final DoubleSupplier frontSupplier;
    private final DoubleSupplier bothSupplier;
    private final BooleanSupplier backSupplier;

    public WjLiftOpenCommand(Lift lift, DoubleSupplier front, DoubleSupplier both, BooleanSupplier back) {
        this.lift = lift;
        this.frontSupplier = front;
        this.bothSupplier = both;
        this.backSupplier = back;

        addRequirements(lift);
    }

    @Override
    public void execute() {
        double front = frontSupplier.getAsDouble();
        double both = bothSupplier.getAsDouble();
        if (backSupplier.getAsBoolean()){
            lift.setBackLiftsPosPower(-0.5);
        }else{
            lift.setPower(front + both, both);
        }


    }
}
