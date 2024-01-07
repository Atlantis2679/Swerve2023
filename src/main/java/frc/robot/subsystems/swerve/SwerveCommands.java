package frc.robot.subsystems.swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.lib.tuneables.TuneableCommand;
import frc.lib.valueholders.BooleanHolder;
import frc.robot.subsystems.swerve.commands.SwerveController;

public class SwerveCommands {
    private final Swerve swerve;

    public SwerveCommands(Swerve swerve) {
        this.swerve = swerve;
    }

    public TuneableCommand controller(DoubleSupplier forwardSupplier, DoubleSupplier sidewaysSupplier,
            DoubleSupplier rotationSupplier, BooleanSupplier isFieldRelativeSupplier) {
        return new SwerveController(swerve, forwardSupplier, sidewaysSupplier, rotationSupplier,
                isFieldRelativeSupplier);
    }

    public TuneableCommand controlModules(DoubleSupplier steerXSupplier, DoubleSupplier steerYSupplier,
            DoubleSupplier speedSupplier) {
        return TuneableCommand.wrap(tuneableBuilder -> {
            BooleanHolder optimizeState = tuneableBuilder.addBoolean("optimize state", true);
            return new RunCommand(() -> {
                double steerY = steerYSupplier.getAsDouble();
                double steerX = steerXSupplier.getAsDouble();
                double speed = speedSupplier.getAsDouble();

                SwerveModuleState[] moduleStates = new SwerveModuleState[4];
                for (int i = 0; i < moduleStates.length; i++) {
                    moduleStates[i] = new SwerveModuleState(
                            speed * SwerveContants.MAX_SPEED_MPS,
                            new Rotation2d(
                                    steerX != 0 || steerY != 0
                                            ? Math.atan2(steerY, steerX) + Math.toRadians(90)
                                            : 0));
                }
                swerve.setModulesState(moduleStates, false, optimizeState.get());
            }, swerve);
        });
    }
}
