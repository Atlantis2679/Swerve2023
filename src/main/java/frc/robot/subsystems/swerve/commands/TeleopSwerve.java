package frc.robot.subsystems.swerve.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.Swerve;
import static frc.robot.subsystems.swerve.SwerveContants.*;

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
        /* the x and y are swapped in the translation2d because we're using Field Coordinate system
        * and when using this system the x from the view of the drivers is the depth of a field 
        * while the y is horizonal to the field.

        for further reading: https://docs.wpilib.org/he/stable/docs/software/advanced-controls/geometry/coordinate-systems.html
        */

        swerve.drive(
                new Translation2d(
                        -1 * yValuesSupplier.getAsDouble(),
                        xValuesSupplier.getAsDouble()).times(FALCON_MAX_SPEED_MPS),
                rotationValuesSupplier.getAsDouble() * FALCOM_MAX_ANGULAR_VELOCITY);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
