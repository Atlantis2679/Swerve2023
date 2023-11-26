package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.io.GyroIO;
import frc.robot.subsystems.swerve.io.GyroIONavX;
import frc.lib.logfields.LogFieldsTable;
import frc.robot.RobotMap.Module0;
import frc.robot.RobotMap.Module1;
import frc.robot.RobotMap.Module2;
import frc.robot.RobotMap.Module3;

import static frc.robot.subsystems.swerve.SwerveContants.*;
import static frc.robot.RobotMap.*;

public class Swerve extends SubsystemBase {
        private final LogFieldsTable fields = new LogFieldsTable(getName());
        private final GyroIO io = new GyroIONavX(fields, NAVX_PORT);

        private final SwerveDriveOdometry odometry;

        private final SwerveModule[] modules = {
                        new SwerveModule(0, Module0.DRIVE_MOTOR_ID, Module0.ANGLE_MOTOR_ID, Module0.ENCODER_ID,
                                        MODULE_0_ANGLE_OFFSET_DEGREES, fields),
                        new SwerveModule(1, Module1.DRIVE_MOTOR_ID, Module1.ANGLE_MOTOR_ID, Module1.ENCODER_ID,
                                        MODULE_1_ANGLE_OFFSET_DEGREES, fields),
                        new SwerveModule(2, Module2.DRIVE_MOTOR_ID, Module2.ANGLE_MOTOR_ID, Module2.ENCODER_ID,
                                        MODULE_2_ANGLE_OFFSET_DEGREES, fields),
                        new SwerveModule(3, Module3.DRIVE_MOTOR_ID, Module3.ANGLE_MOTOR_ID, Module3.ENCODER_ID,
                                        MODULE_3_ANGLE_OFFSET_DEGREES, fields) };

        public final Translation2d FRONT_RIGHT_LOCATION = new Translation2d(SwerveContants.TRACK_WIDTH_M / 2,
                        SwerveContants.TRACK_LENGTH_M / 2);
        public final Translation2d FRONT_LEFT_LOCATION = new Translation2d(SwerveContants.TRACK_WIDTH_M / 2,
                        -SwerveContants.TRACK_LENGTH_M / 2);
        public final Translation2d BACK_RIGHT_LOCATION = new Translation2d(-SwerveContants.TRACK_WIDTH_M / 2,
                        SwerveContants.TRACK_LENGTH_M / 2);
        public final Translation2d BACK_LEFT_LOCATION = new Translation2d(-SwerveContants.TRACK_WIDTH_M / 2,
                        -SwerveContants.TRACK_LENGTH_M / 2);

        private final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                        FRONT_RIGHT_LOCATION,
                        FRONT_LEFT_LOCATION,
                        BACK_RIGHT_LOCATION,
                        BACK_LEFT_LOCATION);

        public Swerve() {
                fields.update();
                resetModulesToAbsolute();

                odometry = new SwerveDriveOdometry(swerveKinematics, getRotation2d(), getModulesPositions());
        }

        @Override
        public void periodic() {
                odometry.update(getRotation2d(), getModulesPositions());

                fields.recordOutput("Odometry", odometry.getPoseMeters());
                fields.recordOutput("Module States", modules[0].getModuleState(),
                                modules[1].getModuleState(), modules[2].getModuleState(),
                                modules[3].getModuleState());
                fields.recordOutput("MPS drive module 0", modules[0].getModuleMPS());
                fields.recordOutput("MPS drive module 1", modules[1].getModuleMPS());
                fields.recordOutput("MPS drive module 2", modules[2].getModuleMPS());
                fields.recordOutput("MPS drive module 3", modules[3].getModuleMPS());
        }

        public void drive(Translation2d translation, double angularVelocity) {
                ChassisSpeeds desiredChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                                translation.getX(),
                                translation.getY(),
                                angularVelocity,
                                getRotation2d());

                SwerveModuleState[] swerveModuleStates = swerveKinematics.toSwerveModuleStates(desiredChassisSpeeds);

                SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveContants.FALCON_MAX_SPEED_MPS);

                fields.recordOutput("Module Desired States", swerveModuleStates[0],
                                swerveModuleStates[1], swerveModuleStates[2],
                                swerveModuleStates[3]);

                for (SwerveModule module : modules) {
                        module.setDesiredState(swerveModuleStates[module.getModuleNumber()]);
                }
        }

        public double getYaw() {
                return io.yaw.getAsDouble();
        }

        public void resetModulesToAbsolute() {
                for (SwerveModule module : modules) {
                        module.resetToAbsolute();
                }
        }

        public Rotation2d getRotation2d() {
                return new Rotation2d(Math.toRadians(io.yaw.getAsDouble()));
        }

        public SwerveModulePosition[] getModulesPositions() {
                SwerveModulePosition[] modulePosition = new SwerveModulePosition[4];

                for (SwerveModule module : modules) {
                        modulePosition[module.getModuleNumber()] = new SwerveModulePosition(
                                        module.getDistanceMeters(),
                                        new Rotation2d(Math.toRadians(module.getAbsoluteAngle())));
                }

                return modulePosition;
        }
}
