// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.commands.SwerveCommands;

public class RobotContainer {
    private final static Swerve swerve = new Swerve();
    public final CommandXboxController driverController = new CommandXboxController(RobotMap.Controllers.DRIVER_PORT);

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        swerve.setDefaultCommand(new SwerveCommands(swerve).controller(
                () -> driverController.getLeftX(),
                () -> -driverController.getLeftY(),
                () -> driverController.getRightX(),
                () -> driverController.leftBumper().getAsBoolean()));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
