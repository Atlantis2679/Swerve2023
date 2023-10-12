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
import frc.robot.subsystems.swerve.SwerveContants.Module1;
import frc.robot.subsystems.swerve.SwerveContants.Module2;
import frc.robot.subsystems.swerve.SwerveContants.Module3;
import frc.robot.subsystems.swerve.SwerveContants.Module4;
import frc.robot.subsystems.swerve.io.GyroIO;
import frc.robot.subsystems.swerve.io.GyroIONavX;
import frc.robot.utils.fields.FieldsTable;

public class Swerve extends SubsystemBase {

    private final FieldsTable fields = new FieldsTable(getName());
    private final GyroIO io = new GyroIONavX(fields, RobotMap.NAVX_PORT);
    private final SwerveDriveOdometry odometry;

    private final SwerveModule[] swerveModules = {
            new SwerveModule(1, Module1.DRIVE_MOTOR_ID, Module1.ANGLE_MOTOR_ID, Module1.ENCODER_ID,
                    Module1.ANGLE_OFFSET_DEGREES),
            new SwerveModule(2, Module2.DRIVE_MOTOR_ID, Module2.ANGLE_MOTOR_ID, Module2.ENCODER_ID,
                    Module2.ANGLE_OFFSET_DEGREES),
            new SwerveModule(3, Module3.DRIVE_MOTOR_ID, Module3.ANGLE_MOTOR_ID, Module3.ENCODER_ID,
                    Module3.ANGLE_OFFSET_DEGREES),
            new SwerveModule(4, Module4.DRIVE_MOTOR_ID, Module4.ANGLE_MOTOR_ID, Module4.ENCODER_ID,
                    Module4.ANGLE_OFFSET_DEGREES) };

    private final SwerveDriveKinematics swerveKInematics = new SwerveDriveKinematics(
            SwerveContants.FRONT_RIGHT_LOCATION,
            SwerveContants.FRONT_LEFT_LOCATION,
            SwerveContants.BACK_RIGHT_LOCATION,
            SwerveContants.BACK_LEFT_LOCATION);

    public Swerve() {
        resetToAbsolute();

        odometry = new SwerveDriveOdometry(swerveKInematics, getRotation2d(), getModulesPositions());
    }

    public void drive(Translation2d translation, double rotation) {
        SwerveModuleState[] swerveModuleStates = swerveKInematics.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation,
                        getRotation2d()));

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, RobotMap.FALCON_MAX_SPEED);

        for (SwerveModule module : swerveModules) {
            module.SetDesiredState(swerveModuleStates[module.getModuleNumber() - 1]);
        }
    }

    public double getYaw() {
        return io.yaw.get();
    }

    public void resetToAbsolute() {
        for (SwerveModule module : swerveModules) {
            module.resetToAbsolute();
        }
    }

    public Rotation2d getRotation2d() {
        return new Rotation2d(Math.toRadians(io.yaw.get()));
    }

    public SwerveModulePosition[] getModulesPositions() {
        SwerveModulePosition[] modulePosition = new SwerveModulePosition[4];

        for (SwerveModule module : swerveModules) {
            modulePosition[module.getModuleNumber() - 1] = new SwerveModulePosition(
                    module.getMeters(),
                    getRotation2d());
        }

        return modulePosition;
    }

    @Override
    public void periodic() {
        odometry.update(getRotation2d(), getModulesPositions());

        fields.recordOutput("Odometry", odometry.getPoseMeters());
    }
}
