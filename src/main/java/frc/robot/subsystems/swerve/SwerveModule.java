package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.subsystems.swerve.io.SwerveModuleIO;
import frc.robot.subsystems.swerve.io.SwerveModuleIOFalcon;
import frc.robot.subsystems.swerve.io.SwerveModuleIOSim;
import frc.robot.utils.fields.FieldsTable;

public class SwerveModule {

    private final int moduleNumber;

    private final FieldsTable fields;
    private final SwerveModuleIO io;

    private final int driveMotorID;
    private final int angleMotorID;
    private final int encoderID;
    private final Rotation2d angleOffSet;

    public SwerveModule(int moduleNumber, int driveMotorID, int angleMotorID, int encoderID,
            double angleOffSetDegrees, boolean isSimulation) {

        this.moduleNumber = moduleNumber;
        this.driveMotorID = driveMotorID;
        this.angleMotorID = angleMotorID;
        this.encoderID = encoderID;
        this.angleOffSet = new Rotation2d(Math.toRadians(angleOffSetDegrees));

        fields = new FieldsTable("Swerve Module " + this.moduleNumber);

        if (isSimulation)
            io = new SwerveModuleIOSim(fields, this.driveMotorID, this.angleMotorID, this.encoderID);
        else
            io = new SwerveModuleIOFalcon(fields, this.driveMotorID, this.angleMotorID, this.encoderID);
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        desiredState = optimize(desiredState, new Rotation2d(Math.toRadians(getIntegratedEncoderAngle())));

        double demandPrcentOutput = desiredState.speedMetersPerSecond / SwerveContants.FALCON_MAX_SPEED;
        io.setDriveSpeed(demandPrcentOutput);

        if (Math.abs(desiredState.speedMetersPerSecond) > (SwerveContants.FALCON_MAX_SPEED * 0.01)) {
            double angleTics = Converstions.degreesToFalcon(desiredState.angle.getDegrees(), SwerveContants.GEAR_RATIO);
            io.setAngleMotor(angleTics);
        }
    }

    public void resetToAbsolute() {
        double absoluteAngle = getAbsoluteAngle();

        double absoluteAngleInFalcon = Converstions.degreesToFalcon(absoluteAngle, SwerveContants.GEAR_RATIO);

        io.setAngleMotorEncoder(absoluteAngleInFalcon);
    }

    public double getAbsoluteAngle() {
        return io.absoluteAngle.get() - angleOffSet.getDegrees();
    }

    public int getModuleNumber() {
        return this.moduleNumber;
    }

    public double getDistanceMeters() {
        return Converstions.falconToMeters(io.driveSpeed.get(), SwerveContants.WHEEL_CIRCUMFERENCE,
                SwerveContants.GEAR_RATIO);
    }

    public double getIntegratedEncoderAngle() {
        return Converstions.falconToDegrees(io.integratedEncoderAngle.get(), SwerveContants.GEAR_RATIO);
    }

    public double placeInAppropriateScope(double currentAngle, double targetAngle) {
        double lowerOffset = currentAngle % 360;

        double lowerBound = lowerOffset >= 0 ? currentAngle - lowerOffset : currentAngle - (360 + lowerOffset);
        double upperBound = lowerOffset >= 0 ? currentAngle + (360 - lowerOffset) : currentAngle - lowerOffset;

        while (targetAngle < lowerBound) {
            targetAngle += 360;
        }
        while (targetAngle > upperBound) {
            targetAngle -= 360;
        }

        if (targetAngle - currentAngle > 180) {
            targetAngle -= 360;
        } else if (targetAngle - currentAngle < -180) {
            targetAngle += 360;
        }

        return targetAngle;
    }

    public SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
        double targetAngle = placeInAppropriateScope(currentAngle.getDegrees(), desiredState.angle.getDegrees());
        double targetSpeed = desiredState.speedMetersPerSecond;

        double delta = targetAngle - currentAngle.getDegrees();

        if (Math.abs(delta) > 90) {
            targetSpeed = -targetSpeed;
            targetAngle = delta > 0 ? (targetAngle - 180) : (targetAngle + 180);
        }

        return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
    }

}
