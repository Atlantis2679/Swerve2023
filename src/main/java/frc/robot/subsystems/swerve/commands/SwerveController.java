package frc.robot.subsystems.swerve.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

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

    private DoubleSupplier sidewaysSupplier;
    private DoubleSupplier forwardSupplier;
    private DoubleSupplier rotationsSupplier;
    private BooleanSupplier isFieldRelative;

    private DoubleHolder maxSpeedAngular = tuneablesTable.addNumber("Max Angular Speed", FALCON_MAX_SPEED_MPS);

    public SwerveController(Swerve swerve, DoubleSupplier forwardSupplier, DoubleSupplier sidewaysSupplier, 
            DoubleSupplier rotationsSupplier, BooleanSupplier isFieldRelative) {
        this.swerve = swerve;
        addRequirements(swerve);

        this.sidewaysSupplier = sidewaysSupplier;
        this.forwardSupplier = forwardSupplier;
        this.rotationsSupplier = rotationsSupplier;
        this.isFieldRelative = isFieldRelative;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        swerve.drive(
                forwardSupplier.getAsDouble() * FALCON_MAX_SPEED_MPS,
                sidewaysSupplier.getAsDouble() * FALCON_MAX_SPEED_MPS,
                rotationsSupplier.getAsDouble() * maxSpeedAngular.get(),
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
