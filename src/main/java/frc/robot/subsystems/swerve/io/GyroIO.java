package frc.robot.subsystems.swerve.io;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import frc.lib.logfields.LogFieldsTable;
import frc.lib.logfields.IOBase;

public abstract class GyroIO extends IOBase{
    public final DoubleSupplier yaw = fields.addDouble("yaw", this::getYaw);
    public final BooleanSupplier isConnected = fields.addBoolean("isConnected", this::isConnected);
    
    public GyroIO(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    // inputes

    protected abstract double getYaw();

    protected abstract boolean isConnected();
}
