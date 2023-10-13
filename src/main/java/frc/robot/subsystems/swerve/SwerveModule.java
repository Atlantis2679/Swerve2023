package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.subsystems.swerve.io.SwerveModuleIO;
import frc.robot.subsystems.swerve.io.SwerveModuleIOFalcon;
import frc.lib.fields.FieldsTable;

public class SwerveModule {
    private final int moduleNumber;

    private final FieldsTable fields;
    private final SwerveModuleIO io;

    private final int driveMotorID;
    private final int angleMotorID;
    private final int encoderID;
    private final Rotation2d angleOffSet;

    public SwerveModule(int moduleNumber, int driveMotorID, int angleMotorID, int encoderID,
            double angleOffSetDegrees) {

        this.moduleNumber = moduleNumber;
        this.driveMotorID = driveMotorID;
        this.angleMotorID = angleMotorID;
        this.encoderID = encoderID;
        this.angleOffSet = new Rotation2d(Math.toRadians(angleOffSetDegrees));

        fields = new FieldsTable("Swerve Module " + this.moduleNumber);
        io = new SwerveModuleIOFalcon(fields, this.driveMotorID, this.angleMotorID, this.encoderID);
    }

    public void SetDesiredState(SwerveModuleState desiredState) {
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
        return Converstions.falconToMeters(io.driveSpeed.get(), SwerveContants.WHEEL_CIRCUMFERENCE, SwerveContants.GEAR_RATIO);
    }

}
