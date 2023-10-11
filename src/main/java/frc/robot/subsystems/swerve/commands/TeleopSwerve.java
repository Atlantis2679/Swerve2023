package frc.robot.subsystems.swerve.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveContants;

public class TeleopSwerve extends CommandBase {
    private final Swerve swerve;

    private DoubleSupplier xValuesSupplier;
    private DoubleSupplier yValuesSupplier;
    private DoubleSupplier rotationValuesSupplier;

    public TeleopSwerve(Swerve swerve, DoubleSupplier xValuesSupplier, DoubleSupplier yValuesSupplier, DoubleSupplier rotationValuesSupplier) {
        this.swerve = swerve;
        addRequirements(swerve);

        this.xValuesSupplier = xValuesSupplier;
        this.yValuesSupplier = yValuesSupplier;
        this.rotationValuesSupplier = rotationValuesSupplier;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        swerve.drive(
                new Translation2d(
                    xValuesSupplier.getAsDouble() * SwerveContants.FALCON_MAX_SPEED, 
                    yValuesSupplier.getAsDouble() * SwerveContants.FALCON_MAX_SPEED),
                rotationValuesSupplier.getAsDouble() * SwerveContants.FALCOM_MAX_ANGULAR_VELOCITY);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
