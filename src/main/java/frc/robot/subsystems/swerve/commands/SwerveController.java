package frc.robot.subsystems.swerve.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.tuneables.SendableType;
import frc.lib.tuneables.TuneableBuilder;
import frc.lib.tuneables.TuneableCommand;
import frc.lib.tuneables.TuneablesTable;
import frc.lib.valueholders.DoubleHolder;
import frc.robot.subsystems.swerve.Swerve;
import static frc.robot.subsystems.swerve.SwerveContants.*;

public class SwerveController extends TuneableCommand {
    private final Swerve swerve;
    private TuneablesTable tuneablesTable = new TuneablesTable(SendableType.LIST);

    private DoubleSupplier xValuesSupplier;
    private DoubleSupplier yValuesSupplier;
    private DoubleSupplier rotationValuesSupplier;
    private BooleanSupplier isFieldRelative;

    private DoubleHolder maxSpeedAngular = tuneablesTable.addNumber("Max Angular Speed", FALCON_MAX_SPEED_MPS);

    public SwerveController(Swerve swerve, DoubleSupplier xValuesSupplier, DoubleSupplier yValuesSupplier,
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
        /*
         * the x and y are swapped in the translation2d because we're using Field
         * Coordinate system and when using this system the x from the view of the
         * drivers is the depth of a field while the y is horizonal to the field.
         * 
         * for further reading:
         * https://docs.wpilib.org/he/stable/docs/software/advanced-controls/geometry/
         * coordinate-systems.html
         */
        swerve.drive(
                new Translation2d(
                        yValuesSupplier.getAsDouble(),
                        -1 * xValuesSupplier.getAsDouble()).times(FALCON_MAX_SPEED_MPS),
                -1 * rotationValuesSupplier.getAsDouble() * maxSpeedAngular.get(),
                isFieldRelative.getAsBoolean());
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
        tuneablesTable.initTuneable(builder);
    }
}
