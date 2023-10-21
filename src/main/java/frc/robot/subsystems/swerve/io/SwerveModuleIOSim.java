package frc.robot.subsystems.swerve.io;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.subsystems.swerve.Converstions;
import frc.robot.utils.fields.FieldsTable;

public class SwerveModuleIOSim extends SwerveModuleIO {
    private final FlywheelSim driveMotor;
    private final FlywheelSim angleMotor;
    private double encoderIntegratedDegreesSim = 0;
    private double encoderAbsolueDegreesSim = 0;

    public SwerveModuleIOSim(FieldsTable fieldsTable, int driveMotorID, int angleMotorID, int encoderID) {
        super(fieldsTable);

        driveMotor = new FlywheelSim(DCMotor.getFalcon500(1), 6.75, 0.025);
        angleMotor = new FlywheelSim(DCMotor.getFalcon500(1), 12.8, 0.004);
    }

    @Override
    public void periodicBeforeFields() {
        double angleDiffRad = angleMotor.getAngularVelocityRadPerSec() * 0.02;
        encoderAbsolueDegreesSim += Math.toDegrees(angleDiffRad);
        encoderIntegratedDegreesSim = (encoderIntegratedDegreesSim + Math.toDegrees(angleDiffRad)) % 360;

        driveMotor.update(0.02);
        angleMotor.update(0.02);
    }

    @Override 
    protected double getAbsoluteAngle() {
        return encoderAbsolueDegreesSim;
    }

    @Override
    protected double getDriveSpeed() {
        return Converstions.RPMToFalcon(driveMotor.getAngularVelocityRPM(), 6.75);
    }

    @Override
    protected double getIntegratedEncoderAngle() {
        return encoderIntegratedDegreesSim;
    }

    @Override
    public void setDriveSpeed(double demandPrcentOutput) {
        driveMotor.setInputVoltage(demandPrcentOutput * 12);
    }

    @Override
    public void setAngleMotor(double angleTics) {
        angleMotor.setInputVoltage((angleTics * 12) / 2048);
    }

    @Override
    public void setAngleMotorEncoder(double angleDegrees) {
        encoderIntegratedDegreesSim = angleDegrees;
    }
}

