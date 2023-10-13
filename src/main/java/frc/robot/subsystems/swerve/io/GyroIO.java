package frc.robot.subsystems.swerve.io;

import java.util.function.Supplier;

import frc.lib.logfields.LogFieldsTable;
import frc.lib.logfields.IOBase;

public abstract class GyroIO extends IOBase{
    public final Supplier<Double> yaw = fields.addDouble("yaw", this::getYaw);
    
    public GyroIO(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    // inputes

    protected abstract double getYaw();
}
