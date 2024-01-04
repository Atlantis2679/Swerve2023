package frc.robot.subsystems.swerve;

import edu.wpi.first.math.util.Units;

public class SwerveContants {
    public final static double GEAR_RATIO_DRIVE = 6.75;
    public final static double GEAR_RATIO_ANGLE = 12.8;
    public final static double WHEEL_RADIUS_METERS = Units.inchesToMeters(2);

    public final static double FALCON_MAX_SPEED_MPS = 5;
    public final static double FALCOM_MAX_ANGULAR_VELOCITY = 3;

    public final static double TRACK_WIDTH_M = 0.551942;
    public final static double TRACK_LENGTH_M = 0.551942;

    public final static double MODULE_0_ANGLE_OFFSET_DEGREES = 65.478515625;
    public final static double MODULE_1_ANGLE_OFFSET_DEGREES = -12.216796875;
    public final static double MODULE_2_ANGLE_OFFSET_DEGREES = 126.826171875;
    public final static double MODULE_3_ANGLE_OFFSET_DEGREES = -142.2;

    public final static double KP = 2.8;
    public final static double KI = 0.0;
    public final static double KD = 0.2;
}
