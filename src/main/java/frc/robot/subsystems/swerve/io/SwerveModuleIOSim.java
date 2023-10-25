package frc.robot.subsystems.swerve.io;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.subsystems.swerve.Converstions;
import frc.robot.subsystems.swerve.SwerveContants;
import frc.robot.utils.fields.FieldsTable;

public class SwerveModuleIOSim extends SwerveModuleIO {
    private final FlywheelSim driveMotor;
    private final FlywheelSim angleMotor;
    private double encoderIntegratedDegreesSim = 0;
    private double encoderAbsolueDegreesSim = 0;
    private double distanceMeters = 0;
    private final PIDController pidControllerAngle = new PIDController(1, 0, 0);

    public SwerveModuleIOSim(FieldsTable fieldsTable, int driveMotorID, int angleMotorID, int encoderID) {
        super(fieldsTable);

        driveMotor = new FlywheelSim(DCMotor.getFalcon500(1), SwerveContants.GEAR_RATIO_DRIVE, 0.05);
        angleMotor = new FlywheelSim(DCMotor.getFalcon500(1), SwerveContants.GEAR_RATIO_ANGLE, 0.004);
    }

    @Override
    public void periodicBeforeFields() {
        fields.recordOutput("angular velucity rad per sec", angleMotor.getAngularVelocityRadPerSec());
        fields.recordOutput("MPS drive", driveMotor.getAngularVelocityRPM() * SwerveContants.WHEEL_CIRCUMFERENCE);

        double angleDiffRad = angleMotor.getAngularVelocityRadPerSec() * 0.02;
        encoderIntegratedDegreesSim += Math.toDegrees(angleDiffRad);
        encoderAbsolueDegreesSim = encoderIntegratedDegreesSim % 360;

        driveMotor.update(0.02);
        angleMotor.update(0.02);

        double angleDiffDistance = driveMotor.getAngularVelocityRadPerSec() * 0.02;
        distanceMeters += angleDiffDistance * SwerveContants.WHEEL_RADIUS;

        double calculate = (pidControllerAngle.calculate(getIntegratedEncoderDegrees()));
        angleMotor.setInputVoltage((calculate * 12) / 360);
    }

    @Override 
    protected double getAbsoluteAngleDegrees() {
        return encoderAbsolueDegreesSim;
    }

    @Override
    protected double getDriveSpeedMPS() {
        return Converstions.falconToMPS(Converstions.RPMToFalcon(driveMotor.getAngularVelocityRPM(), SwerveContants.GEAR_RATIO_DRIVE),
                     SwerveContants.WHEEL_CIRCUMFERENCE, SwerveContants.GEAR_RATIO_DRIVE);
    }

    @Override
    protected double getIntegratedEncoderDegrees() {
        return encoderIntegratedDegreesSim;
    }

    @Override
    protected double getDriveDistanceMeters() {
        return distanceMeters;
    }

    @Override
    public void setDriveSpeed(double demandPrcentOutput) {
        demandPrcentOutput = MathUtil.clamp(demandPrcentOutput, -1, 1);
        driveMotor.setInputVoltage(demandPrcentOutput * 12);
    }

    @Override
    public void setAngleMotor(double degrees) {
        pidControllerAngle.setSetpoint(degrees);
    }

    @Override
    public void setAngleMotorEncoder(double angleDegrees) {
        encoderIntegratedDegreesSim = angleDegrees;
    }
}
