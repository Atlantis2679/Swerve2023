package frc.robot.subsystems.swerve.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.tuneables.TuneableBuilder;
import frc.lib.tuneables.TuneableCommand;
import frc.robot.subsystems.swerve.Swerve;
import static frc.robot.subsystems.swerve.SwerveContants.*;

public class Controller extends TuneableCommand {
    private final Swerve swerve;

    private DoubleSupplier xValuesSupplier;
    private DoubleSupplier yValuesSupplier;
    private DoubleSupplier rotationValuesSupplier;
    private BooleanSupplier isFieldRelative;
    private double maxSpeedMPS = FALCOM_MAX_ANGULAR_VELOCITY;
    private double maxSpeedAngular = FALCON_MAX_SPEED_MPS;

    public Controller(Swerve swerve, DoubleSupplier xValuesSupplier, DoubleSupplier yValuesSupplier,
     DoubleSupplier rotationValuesSupplier, BooleanSupplier isFieldRelative) {
        this.swerve = swerve;
        addRequirements(swerve);

        this.xValuesSupplier = xValuesSupplier;
        this.yValuesSupplier = yValuesSupplier;
        this.rotationValuesSupplier = rotationValuesSupplier;
        this.isFieldRelative = isFieldRelative;
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

        if(!isFieldRelative.getAsBoolean()) {
            swerve.drive(
                new Translation2d(
                        yValuesSupplier.getAsDouble(),
                        -1 * xValuesSupplier.getAsDouble()).times(FALCON_MAX_SPEED_MPS),
                -1 * rotationValuesSupplier.getAsDouble() * FALCOM_MAX_ANGULAR_VELOCITY,
                isFieldRelative);
        }
        else {
            swerve.drive(
                new Translation2d(
                    yValuesSupplier.getAsDouble(),
                    -1 * xValuesSupplier.getAsDouble()).times(FALCON_MAX_SPEED_MPS),
                -1 * rotationValuesSupplier.getAsDouble() * FALCOM_MAX_ANGULAR_VELOCITY,
                isFieldRelative);
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void initTuneable(TuneableBuilder builder) {
        builder.addDoubleProperty("Max rotation", () -> maxSpeedAngular, (maxSpeedAngularUpdate) -> maxSpeedAngular = maxSpeedAngularUpdate);
        builder.addDoubleProperty("Max speed MPS", () -> maxSpeedMPS, (maxSpeedMPSUpdate) -> maxSpeedMPS = maxSpeedMPSUpdate);
    }
}
