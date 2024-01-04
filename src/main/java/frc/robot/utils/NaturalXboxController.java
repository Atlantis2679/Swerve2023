package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class NaturalXboxController extends CommandXboxController {
    public NaturalXboxController(int port) {
        super(port);
    }

    @Override
    public double getRightX() {
        return super.getRightX();
    }

    @Override
    public double getRightY() {
        return -1 * naturalize(super.getRightY());
    }

    @Override
    public double getLeftX() {
        return super.getLeftX();
    }

    @Override
    public double getLeftY() {
        return -1 * naturalize(super.getLeftY());
    }

    public double naturalize(double value) {
        if(Math.abs(value) > 0.05) {
            return 0;
        }

        return Math.pow(value, 2) * Math.signum(value);
    }
}
