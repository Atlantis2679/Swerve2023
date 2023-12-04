package frc.robot.subsystems.swerve.io;

import java.util.function.DoubleSupplier;

import frc.lib.logfields.LogFieldsTable;
import frc.lib.logfields.IOBase;

public abstract class SwerveModuleIO extends IOBase{
    public final DoubleSupplier absoluteAngle = fields.addDouble("absoluteAngle", this::getAbsoluteAngle);
    public final DoubleSupplier driveSpeed = fields.addDouble("driveSpeed", this::getDriveSpeed);
    
    public SwerveModuleIO(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    // inputs

    protected abstract double getAbsoluteAngle();

    protected abstract double getDriveSpeed();

    // Outputs

    public abstract void setDriveSpeed(double demand);

    public abstract void setAngleMotor(double angle);

    public abstract void setAngleMotorEncoder(double degrees);


}
