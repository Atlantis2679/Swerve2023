package frc.robot.subsystems.swerve.io;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.lib.logfields.LogFieldsTable;
import static frc.robot.subsystems.swerve.SwerveContants.*;

public class SwerveModuleIOSim extends SwerveModuleIO {
    private final FlywheelSim driveMotorSim;
    private final FlywheelSim angleMotorSim;
    private double simEncoderIntegratedRotations = 0;
    private double simEncoderAbsolueRotations = 0;
    private double simDriveRotations = 0;
    private final PIDController pidControllerAngle = new PIDController(10, 0, 0);

    public SwerveModuleIOSim(LogFieldsTable fieldsTable, int driveMotorID, int angleMotorID, int encoderID) {
        super(fieldsTable);

        driveMotorSim = new FlywheelSim(DCMotor.getFalcon500(1), GEAR_RATIO_DRIVE, 0.05);
        angleMotorSim = new FlywheelSim(DCMotor.getFalcon500(1), GEAR_RATIO_ANGLE, 0.004);
    }

    @Override
    public void periodicBeforeFields() {
        double angleRPS = angleMotorSim.getAngularVelocityRPM() / 60;
        double angleRotationsDiff = angleRPS * 0.02;
        simEncoderIntegratedRotations += angleRotationsDiff;
        simEncoderAbsolueRotations = simEncoderIntegratedRotations % 1;

        driveMotorSim.update(0.02);
        angleMotorSim.update(0.02);

        double driveRPS = driveMotorSim.getAngularVelocityRPM() / 60;
        double driveRotationsDiff = driveRPS * 0.02;
        simDriveRotations += driveRotationsDiff;

        double AnglePIDResult = (pidControllerAngle.calculate(getIntegratedAngleEncoderRotations()));
        angleMotorSim.setInputVoltage(AnglePIDResult);
    }

    @Override 
    protected double getAbsoluteAngleRotations() {
        return simEncoderAbsolueRotations;
    }

    @Override
    protected double getDriveSpeedRPS() {
        return driveMotorSim.getAngularVelocityRPM() / 60;
    }

    @Override
    protected double getIntegratedAngleEncoderRotations() {
        return simEncoderIntegratedRotations;
    }

    @Override
    protected double getDriveMotorRotations() {
        return simDriveRotations;
    }

    @Override
    protected double getP() {
        return pidControllerAngle.getP();
    }

    @Override
    protected double getI() {
        return pidControllerAngle.getI();
    }

    @Override
    protected double getD() {
        return pidControllerAngle.getD();
    }

    @Override
    public void setDriveSpeedPrecentage(double demandPrcentOutput) {
        demandPrcentOutput = MathUtil.clamp(demandPrcentOutput, -1, 1);
        driveMotorSim.setInputVoltage(demandPrcentOutput * 12);
    }

    @Override
    public void setAngleMotorPositionRotations(double rotations) {
        pidControllerAngle.setSetpoint(rotations);
    }

    @Override
    public void setIntegratedAngleEncoderRotations(double angleRotations) {
        simEncoderIntegratedRotations = angleRotations;
    }

    @Override
    public void setP(double p) {
        pidControllerAngle.setP(p);
    }

    @Override
    public void setI(double i) {
        pidControllerAngle.setI(i);
    }

    @Override
    public void setD(double d) {
        pidControllerAngle.setD(d);
    }
}

