package frc.robot.subsystems.swerve.io;

import frc.lib.logfields.LogFieldsTable;

public class GyroIOSim extends GyroIO{
    public GyroIOSim(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    @Override 
    protected double getYawDegreesCW() {
        return 0;
    }

    @Override
    protected boolean isConnected() {
        return false;
    }
}