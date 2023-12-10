package frc.robot.subsystems.swerve;

public class Converstions {
    private final static double COUNTS_PER_REV_CANCODER = 4096;

    public static double radiusToCircumference(double radius) {
        return radius * 2 * Math.PI;
    }

    public static double CANcoderToDegrees(double positionCounts, double gearRatio) {
        return positionCounts * (360.0 / (gearRatio * COUNTS_PER_REV_CANCODER));
    }

    public static double degreesToCANcoder(double degrees, double gearRatio) {
        return degrees / (360.0 / (gearRatio * COUNTS_PER_REV_CANCODER));
    }

    public static double rotationsToDegrees(double positionCounts, double gearRatio) {
        return positionCounts * (360.0 / gearRatio);
    }

    public static double degreesToRotations(double degrees, double gearRatio) {
        return (degrees / 360.0) / gearRatio;
    }

    public static double rotationsToRPM(double velocityCounts, double gearRatio) {
        double motorRPM = velocityCounts * (600.0 / COUNTS_PER_REV_CANCODER);
        double mechRPM = motorRPM / gearRatio;
        return mechRPM;
    }

    public static double RPMToRotations(double RPM, double gearRatio) {
        double motorRPM = RPM * gearRatio;
        double sensorCounts = motorRPM * 600.0;
        return sensorCounts;
    }

    public static double rotationsToMPS(double velocityRPS, double wheelRadiusMeters, double gearRatio) {
        double wheelRPM = rotationsToRPM(velocityRPS, gearRatio);
        double wheelMPS = (wheelRPM * radiusToCircumference(wheelRadiusMeters)) / 60;
        return wheelMPS;
    }

    public static double MPSToRotations(double velocity, double wheelRadiusMeters, double gearRatio) {
        double wheelRPM = ((velocity * 60) / radiusToCircumference(wheelRadiusMeters));
        double wheelVelocity = RPMToRotations(wheelRPM, gearRatio);
        return wheelVelocity;
    }

    public static double rotationsToMeters(double positionCounts, double wheelRadiusMeters, double gearRatio) {
        return positionCounts * (radiusToCircumference(wheelRadiusMeters) / gearRatio);
    }

    public static double MetersToRotations(double meters, double wheelRadiusMeters, double gearRatio) {
        return meters / (radiusToCircumference(wheelRadiusMeters) / gearRatio);
    }

    public static double RPMToMPS(double RPM, double wheelRadiusMeters) {
        return wheelRadiusMeters * 2 * Math.PI * (RPM / 60);
    }
}
