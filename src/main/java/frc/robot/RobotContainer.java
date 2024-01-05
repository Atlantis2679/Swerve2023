package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.tuneables.TuneablesManager;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveCommands;
import frc.robot.utils.NaturalXboxController;

public class RobotContainer {
    private final Swerve swerve = new Swerve();
    private final NaturalXboxController driverController = new NaturalXboxController(RobotMap.Controllers.DRIVER_PORT);
    private final SwerveCommands swerveCommands = new SwerveCommands(swerve);

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        swerve.setDefaultCommand(swerveCommands.controller(
                driverController::getSquaredLeftY,
                driverController::getSquaredLeftX,
                driverController::getRightX,
                driverController.leftBumper()::getAsBoolean));

        driverController.a().onTrue(new InstantCommand(swerve::resetYaw));

        TuneablesManager.add("Swerve/switch to modules control",
                swerveCommands.controlModules(
                        driverController::getLeftX,
                        driverController::getLeftY,
                        driverController::getRightX));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
