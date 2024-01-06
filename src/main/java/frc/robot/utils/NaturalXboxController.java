package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * Class that extends the regular CommandXboxController to provide:
 * - automatic deadband.
 * - Y axis that is up-positive.
 * - Squared axis values getters, for finer control.
 */
public class NaturalXboxController extends CommandXboxController {
    private double deadband;

    public NaturalXboxController(int port, double deadband) {
        super(port);
        this.deadband = deadband;
    }

    public NaturalXboxController(int port) {
        this(port, 0.05);
    }

    public void setDeadband(double deadband) {
        this.deadband = deadband;
    }

    @Override
    public double getRightX() {
        return applyDeadband(super.getRightX());
    }

    @Override
    public double getRightY() {
        return -1 * applyDeadband(super.getRightY());
    }

    @Override
    public double getLeftX() {
        return applyDeadband(super.getLeftX());
    }

    @Override
    public double getLeftY() {
        return -1 * applyDeadband(super.getLeftY());
    }

    @Override
    public double getLeftTriggerAxis() {
        return applyDeadband(super.getLeftTriggerAxis());
    }

    @Override
    public double getRightTriggerAxis() {
        return applyDeadband(super.getRightTriggerAxis());
    }

    public double getSquaredRightX() {
        return square(getRightX());
    }

    public double getSquaredRightY() {
        return square(getRightY());
    }

    public double getSquaredLeftX() {
        return square(getLeftX());
    }

    public double getSquaredLeftY() {
        return square(getLeftY());
    }

    public double getSquaredLeftTriggerAxis() {
        return square(getLeftTriggerAxis());
    }

    public double getSquaredRightTriggerAxis() {
        return square(getRightTriggerAxis());
    }

    public double applyDeadband(double value) {
        return Math.abs(value) < deadband ? 0 : value;
    }

    public double square(double value) {
        return Math.pow(value, 2) * (value >= 0 ? 1 : -1);
    }
}
