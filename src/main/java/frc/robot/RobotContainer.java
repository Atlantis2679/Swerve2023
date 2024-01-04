// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.tuneables.TuneablesManager;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveCommands;
import frc.robot.utils.NaturalXboxController;

public class RobotContainer {
    private final Swerve swerve = new Swerve();
    private final CommandXboxController driverController = new CommandXboxController(RobotMap.Controllers.DRIVER_PORT);
    private final SwerveCommands swerveCommands = new SwerveCommands(swerve);

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        swerve.setDefaultCommand(swerveCommands.controller(
                () -> naturalize(driverController.getLeftX() * 0.5),
                () -> naturalize(driverController.getLeftY() * -0.5),
                () -> naturalize(driverController.getRightX() * 0.8),
                driverController.leftBumper()::getAsBoolean));

        driverController.a().onTrue(new InstantCommand(swerve::resetYaw));

        TuneablesManager.add("Swerve/switch to modules control",
                swerveCommands.controlModules(
                        () -> MathUtil.applyDeadband(driverController.getLeftX(), 0.1),
                        () -> MathUtil.applyDeadband(-driverController.getLeftY(), 0.1),
                        () -> MathUtil.applyDeadband(-driverController.getRightY(), 0.1)).fullTuneable());

            
    }

    public double naturalize(double value) {
        if(Math.abs(value) < 0.05) {
            return 0;
        }

        return Math.pow(value, 2) * Math.signum(value);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
