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

    @Override
    public void resetGyro() {
        return;
    }

    @Override
    public void setYaw(double degree) {
        return;
    }
}