package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.RobotMap.Module0;
import frc.robot.RobotMap.Module1;
import frc.robot.RobotMap.Module2;
import frc.robot.RobotMap.Module3;
import frc.robot.subsystems.swerve.io.GyroIO;
import frc.robot.subsystems.swerve.io.GyroIONavX;
import frc.lib.logfields.LogFieldsTable;

public class Swerve extends SubsystemBase {

    private final LogFieldsTable fields = new LogFieldsTable(getName());
    private final GyroIO io = new GyroIONavX(fields, RobotMap.NAVX_PORT);
    private final SwerveDriveOdometry odometry;

    private final SwerveModule[] modules = {
            new SwerveModule(0, Module0.DRIVE_MOTOR_ID, Module0.ANGLE_MOTOR_ID, Module0.ENCODER_ID,
                    SwerveContants.MODULE_0_ANGLE_OFFSET_DEGREES),
            new SwerveModule(1, Module1.DRIVE_MOTOR_ID, Module1.ANGLE_MOTOR_ID, Module1.ENCODER_ID,
                    SwerveContants.MODULE_1_ANGLE_OFFSET_DEGREES),
            new SwerveModule(2, Module2.DRIVE_MOTOR_ID, Module2.ANGLE_MOTOR_ID, Module2.ENCODER_ID,
                    SwerveContants.MODULE_2_ANGLE_OFFSET_DEGREES),
            new SwerveModule(3, Module3.DRIVE_MOTOR_ID, Module3.ANGLE_MOTOR_ID, Module3.ENCODER_ID,
                    SwerveContants.MODULE_3_ANGLE_OFFSET_DEGREES) };

    private final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            SwerveContants.FRONT_RIGHT_LOCATION,
            SwerveContants.FRONT_LEFT_LOCATION,
            SwerveContants.BACK_RIGHT_LOCATION,
            SwerveContants.BACK_LEFT_LOCATION);

    public Swerve() {
        resetModulesToAbsolute();

        odometry = new SwerveDriveOdometry(swerveKinematics, getRotation2d(), getModulesPositions());
    }

    @Override
    public void periodic() {
        odometry.update(getRotation2d(), getModulesPositions());

        fields.recordOutput("Odometry", odometry.getPoseMeters());
    }

    public void drive(Translation2d translation, double angularVelocity) {
        ChassisSpeeds desiredChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(),
                translation.getY(),
                angularVelocity,
                getRotation2d());

        SwerveModuleState[] swerveModuleStates = swerveKinematics.toSwerveModuleStates(desiredChassisSpeeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveContants.FALCON_MAX_SPEED);

        for (SwerveModule module : modules) {
            module.SetDesiredState(swerveModuleStates[module.getModuleNumber()]);
        }
    }

    public double getYaw() {
        return io.yaw.get();
    }

    public void resetModulesToAbsolute() {
        for (SwerveModule module : modules) {
            module.resetToAbsolute();
        }
    }

    public Rotation2d getRotation2d() {
        return new Rotation2d(Math.toRadians(io.yaw.get()));
    }

    public SwerveModulePosition[] getModulesPositions() {
        SwerveModulePosition[] modulePosition = new SwerveModulePosition[4];

        for (SwerveModule module : modules) {
            modulePosition[module.getModuleNumber()] = new SwerveModulePosition(
                    module.getDistanceMeters(),
                    getRotation2d());
        }

        return modulePosition;
    }
}
