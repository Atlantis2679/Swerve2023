package frc.robot.subsystems.swerve.io;

import java.util.function.Supplier;

import frc.robot.utils.fields.FieldsTable;
import frc.robot.utils.fields.IOBase;

public abstract class SwerveModuleIO extends IOBase{
    public final Supplier<Double> absoluteAngle = fields.addDouble("absoluteAngle", this::getAbsoluteAngle);
    public final Supplier<Double> driveSpeed = fields.addDouble("driveSpeed", this::getDriveSpeed);
    
    public SwerveModuleIO(FieldsTable fieldsTable) {
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
