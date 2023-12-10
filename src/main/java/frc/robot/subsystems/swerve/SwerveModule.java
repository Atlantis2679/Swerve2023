package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.logfields.LogFieldsTable;
import frc.lib.tuneables.Tuneable;
import frc.lib.tuneables.TuneableBuilder;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.io.SwerveModuleIO;
import frc.robot.subsystems.swerve.io.SwerveModuleIOFalcon;
import frc.robot.subsystems.swerve.io.SwerveModuleIOSim;

import static frc.robot.subsystems.swerve.SwerveContants.*;

public class SwerveModule implements Tuneable {
    private final int moduleNumber;

    private final LogFieldsTable fields;
    private final SwerveModuleIO io;

    private final int driveMotorID;
    private final int angleMotorID;
    private final int encoderID;
    private double angleOffSetDegrees;

    private double lastDriveDistanceMeters;
    private double currDriveDistanceMeters;

    private final double WHEEL_CIRCUMFERENCE_METERS = 2 * Math.PI * WHEEL_RADIUS_METERS;

    public SwerveModule(int moduleNumber, int driveMotorID, int angleMotorID, int encoderID,
            double angleOffSetDegrees, LogFieldsTable fieldsTable) {
        this.moduleNumber = moduleNumber;
        this.driveMotorID = driveMotorID;
        this.angleMotorID = angleMotorID;
        this.encoderID = encoderID;
        this.angleOffSetDegrees = angleOffSetDegrees;

        fields = fieldsTable.getSubTable("Module " + moduleNumber);
        fields.update();

        io = Robot.isSimulation()
                ? new SwerveModuleIOSim(fields, this.driveMotorID, this.angleMotorID, this.encoderID)
                : new SwerveModuleIOFalcon(fields, this.driveMotorID, this.angleMotorID, this.encoderID);

        lastDriveDistanceMeters = getDriveDistanceMeters();
        currDriveDistanceMeters = getDriveDistanceMeters();
    }

    public void periodic() {
        lastDriveDistanceMeters = currDriveDistanceMeters;
        currDriveDistanceMeters = getDriveDistanceMeters();
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        desiredState = optimize(desiredState, getIntegratedEncoderAngleDegrees());

        double demandPrcentOutput = desiredState.speedMetersPerSecond / FALCON_MAX_SPEED_MPS;
        io.setDriveSpeedPrecentage(demandPrcentOutput);

        // only rotate when speed is greater then 1%, to avoid damaging wheels.
        if (Math.abs(desiredState.speedMetersPerSecond) > (FALCON_MAX_SPEED_MPS * 0.01)) {
            io.setAngleMotorRotations(desiredState.angle.getRotations());
        }
    }

    public void resetToAbsolute() {
        io.setIntegratedAngleEncoderRotations(io.absoluteAngleRotations.getAsDouble());
    }

    public double getAbsoluteAngleDegrees() {
        return (io.absoluteAngleRotations.getAsDouble() * 360) - angleOffSetDegrees;
    }

    public int getModuleNumber() {
        return this.moduleNumber;
    }

    public double getDriveDistanceMeters() {
        return io.driveMotorRotations.getAsDouble() * WHEEL_CIRCUMFERENCE_METERS;
    }

    public double getIntegratedEncoderAngleDegrees() {
        return io.integratedEncoderAngleRotations.getAsDouble() * 360;
    }

    public double placeInAppropriateScope(double currentAngleDegrees, double targetAngleDegrees) {
        int scope = (int) currentAngleDegrees / 360;

        double lowerBound = currentAngleDegrees >= 0 ? scope * 360 : (scope - 1) * 360;
        double upperBound = currentAngleDegrees >= 0 ? (scope + 1) * 360 : scope * 360;

        while (targetAngleDegrees < lowerBound) {
            targetAngleDegrees += 360;
        }
        while (targetAngleDegrees > upperBound) {
            targetAngleDegrees -= 360;
        }

        if (targetAngleDegrees - currentAngleDegrees > 180) {
            targetAngleDegrees -= 360;
        } else if (targetAngleDegrees - currentAngleDegrees < -180) {
            targetAngleDegrees += 360;
        }

        return targetAngleDegrees;
    }

    public SwerveModuleState optimize(SwerveModuleState desiredState, double currentAngleDegrees) {
        double targetAngle = placeInAppropriateScope(currentAngleDegrees, desiredState.angle.getDegrees());
        double targetSpeed = desiredState.speedMetersPerSecond;

        double delta = targetAngle - currentAngleDegrees;

        if (Math.abs(delta) > 90) {
            targetSpeed = -targetSpeed;
            targetAngle = delta > 0 ? (targetAngle - 180) : (targetAngle + 180);
        }

        return new SwerveModuleState(targetSpeed, new Rotation2d(Math.toRadians(targetAngle)));
    }

    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(getModuleMPS(), getRotation2d());
    }

    public double getModuleMPS() {
        return io.driveSpeedRPS.getAsDouble() * WHEEL_CIRCUMFERENCE_METERS;
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getAbsoluteAngleDegrees());
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(currDriveDistanceMeters, getRotation2d());
    }

    public SwerveModulePosition getModulePositionDelta() {
        return new SwerveModulePosition(
                currDriveDistanceMeters - lastDriveDistanceMeters,
                getRotation2d());
    }

    public void setAbsoluteEncoderAngle(double degrees) {
        angleOffSetDegrees = degrees;
    }

    public double getP() {
        return io.kP.getAsDouble();
    }

    public double getI() {
        return io.kI.getAsDouble();
    }

    public double getD() {
        return io.kD.getAsDouble();
    }

    public void setP(double p) {
        io.setP(p);
    }

    public void setI(double i) {
        io.setI(i);
    }

    public void setD(double d) {
        io.setD(d);
    }

    @Override
    public void initTuneable(TuneableBuilder builder) {
        // builder.addChild("PID module " + getModuleNumber(), (Tuneable) (PIDBuiler) ->
        // {
        // PIDBuiler.setSendableType(SendableType.PID);
        // PIDBuiler.addDoubleProperty("p", () -> io.kP.getAsDouble(), (kPUpdate) ->
        // io.setP(kPUpdate));
        // PIDBuiler.addDoubleProperty("i", () -> io.kI.getAsDouble(), (kIUpdate) ->
        // io.setP(kIUpdate));
        // PIDBuiler.addDoubleProperty("d", () -> io.kD.getAsDouble(), (kDUpdate) ->
        // io.setP(kDUpdate));
        // });
        builder.addDoubleProperty("Absolute Angle Degrees", this::getAbsoluteAngleDegrees,
                this::setAbsoluteEncoderAngle);
    }
}
