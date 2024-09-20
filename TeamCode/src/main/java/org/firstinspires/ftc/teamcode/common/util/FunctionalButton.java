package org.firstinspires.ftc.teamcode.common.util;

import com.arcrobotics.ftclib.command.button.Button;

import java.util.function.BooleanSupplier;

public class FunctionalButton extends Button {
    private BooleanSupplier booleanSupplier;

    public FunctionalButton(BooleanSupplier supplier) {
        booleanSupplier = supplier;
    }

    @Override
    public boolean get() {
        return booleanSupplier.getAsBoolean();
    }
}
