package frc.robot.subsystems.swerve.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import frc.lib.tuneables.TuneableCommand;
import frc.robot.subsystems.swerve.Swerve;

public class SwerveCommands {
    private final Swerve swerve;

    public SwerveCommands(Swerve swerve) {
        this.swerve = swerve;
    }

    public TuneableCommand controller(DoubleSupplier xDoubleSupplier, DoubleSupplier yDoubleSupplier,
            DoubleSupplier rotationDoubleSupplier, BooleanSupplier isFieldRelative) {
        return new Controller(swerve, xDoubleSupplier, yDoubleSupplier, rotationDoubleSupplier, isFieldRelative);
    }
}
