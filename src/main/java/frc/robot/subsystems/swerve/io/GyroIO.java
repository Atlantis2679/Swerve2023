package frc.robot.subsystems.swerve.io;

import java.util.function.Supplier;

import frc.robot.utils.fields.FieldsTable;
import frc.robot.utils.fields.IOBase;

public abstract class GyroIO extends IOBase{
    public final Supplier<Double> yaw = fields.addDouble("yaw", this::getYaw);
    
    public GyroIO(FieldsTable fieldsTable) {
        super(fieldsTable);
    }

    // inputes

    protected abstract double getYaw();
}
