package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.subsystems.swerve.Swerve;

public class Autos {
    public static Command getOutsideOfBegginingLine(Swerve swerve) {
        return new PathPlannerAuto("getOutsideOfStartingLin");
    }

    public static Command print(String string) {
        return new PrintCommand(string);
    }

}


