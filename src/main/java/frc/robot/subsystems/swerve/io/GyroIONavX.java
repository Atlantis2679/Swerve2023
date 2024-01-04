package frc.robot.subsystems.swerve.io;

import com.kauailabs.navx.frc.AHRS;
import frc.lib.logfields.LogFieldsTable;

import static frc.robot.RobotMap.*;


public class GyroIONavX extends GyroIO{
    private final AHRS navX = new AHRS(NAVX_PORT);
    
    public GyroIONavX(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    @Override 
    protected double getYaw() {
        return navX.getYaw();
    }

    @Override
    protected boolean isConnected() {
        return navX.isConnected();
    }
}
