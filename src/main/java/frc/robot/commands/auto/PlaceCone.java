package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Auto2.Arm.ArmTopAuto;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.Claw;
import frc.robot.subsystems.Arm.Wrist;
import frc.robot.subsystems.Swerve.SwerveDrive;

/**
 * PlaceCone
 */
public class PlaceCone extends SequentialCommandGroup {
    public PlaceCone(Arm arm, Wrist wrist, SwerveDrive swerveDrive, Claw claw){
        addCommands(new ArmTopAuto(arm, wrist), new DriveToWall(swerveDrive), new OpenClaw(claw));
    }
} 