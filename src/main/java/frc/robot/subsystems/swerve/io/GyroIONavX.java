package frc.robot.subsystems.swerve.io;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SerialPort;
import frc.robot.utils.fields.FieldsTable;

public class GyroIONavX extends GyroIO{
    private final AHRS navX;
    
    public GyroIONavX(FieldsTable fieldsTable, SerialPort.Port navXID) {
        super(fieldsTable);
        navX = new AHRS(navXID);
    }

    @Override 
    protected double getYaw() {
        return navX.getYaw();
    }

}
