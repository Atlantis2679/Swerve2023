package frc.robot.subsystems.swerve;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.util.Units;

public class SwerveContants {
    public final static double GEAR_RATIO_DRIVE = 6.75;
    public final static double GEAR_RATIO_ANGLE = 12.8;
    public final static double WHEEL_RADIUS_METERS = Units.inchesToMeters(2);

    public final static double MAX_SPEED_MPS = 5;
    public final static double MAX_ANGULAR_VELOCITY = 3;

    public final static double TRACK_WIDTH_M = 0.551942;
    public final static double TRACK_LENGTH_M = 0.551942;

    public final static double MODULE_0_ABSOLUTE_ANGLE_OFFSET_DEGREES = -113.73046875;
    public final static double MODULE_1_ABSOLUTE_ANGLE_OFFSET_DEGREES = 168.75;
    public final static double MODULE_2_ABSOLUTE_ANGLE_OFFSET_DEGREES = -54.052734375;
    public final static double MODULE_3_ABSOLUTE_ANGLE_OFFSET_DEGREES = 39.462890625;

    public final static double KP = 1.8;
    public final static double KI = 0.0;
    public final static double KD = 0;

    public final static HolonomicPathFollowerConfig pathFollowerConfigs = new HolonomicPathFollowerConfig(
            new PIDConstants(0.75, 0, 0.018),
            new PIDConstants(0.9, 0.0, 0.008),
            1,
            0.39028,
            new ReplanningConfig());
}
