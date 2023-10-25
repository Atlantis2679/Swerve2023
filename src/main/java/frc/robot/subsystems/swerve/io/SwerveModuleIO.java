package frc.robot.subsystems.swerve.io;

import java.util.function.Supplier;

import frc.lib.logfields.LogFieldsTable;
import frc.lib.logfields.IOBase;

public abstract class SwerveModuleIO extends IOBase{
    public final Supplier<Double> absoluteAngle = fields.addDouble("absoluteAngle", this::getAbsoluteAngleDegrees);
    public final Supplier<Double> integratedEncoderAngle = fields.addDouble("absoluteAngle", this::getIntegratedEncoderDegrees);
    public final Supplier<Double> driveSpeedMPS = fields.addDouble("driveSpeedMPS", this::getDriveSpeedMPS);
    public final Supplier<Double> driveDistanceMeters = fields.addDouble("driveDistanceMeters", this::getDriveDistanceMeters);
    
    public SwerveModuleIO(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    // inputs

    protected abstract double getAbsoluteAngleDegrees();

    protected abstract double getDriveSpeedMPS();

    protected abstract double getIntegratedEncoderDegrees();

    protected abstract double getDriveDistanceMeters();

    // Outputs

    public abstract void setDriveSpeed(double demand);

    public abstract void setAngleMotor(double angle);

    public abstract void setAngleMotorEncoder(double degrees);


}
