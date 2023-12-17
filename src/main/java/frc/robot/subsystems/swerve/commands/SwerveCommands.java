package frc.robot.subsystems.swerve.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.lib.tuneables.TuneableCommand;
import frc.robot.subsystems.swerve.Swerve;

public class SwerveCommands {
    private final Swerve swerve;

    public SwerveCommands(Swerve swerve) {
        this.swerve = swerve;
    }

    public TuneableCommand controller(DoubleSupplier xDoubleSupplier, DoubleSupplier yDoubleSupplier,
            DoubleSupplier rotationDoubleSupplier, BooleanSupplier isFieldRelative) {
        return new SwerveController(swerve, xDoubleSupplier, yDoubleSupplier, rotationDoubleSupplier, isFieldRelative);
    }

    public TuneableCommand controlModules(DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
        return TuneableCommand.wrap((table) -> {
            return new RunCommand(() -> {
                SwerveModuleState[] moduleStates = new SwerveModuleState[4];
                for (int i = 0; i < moduleStates.length; i++) {
                    moduleStates[i] = new SwerveModuleState(
                        0.0,
                        new Rotation2d(Math.atan2(ySupplier.getAsDouble(), xSupplier.getAsDouble()) + Math.toRadians(90)));
                }
                swerve.setModulesState(moduleStates, false);
            }, swerve);
        });
    }
}
