package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Balance.balance_alt;
import frc.robot.commands.Drivetrain.DriveDistance;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.Claw;
import frc.robot.subsystems.Arm.Wrist;
import frc.robot.subsystems.Swerve.SwerveDrive;
import frc.robot.commands.auto.OneCubeAuto;

public class EverythingAuto extends CommandBase{
    SwerveDrive swerve;
    Arm arm;
    Wrist wrist;
    Claw claw;
    
    public EverythingAuto(SwerveDrive swerve, Arm arm, Wrist wrist, Claw claw){
        this.swerve = swerve;
        this.arm = arm;
        this.wrist = wrist;
        this.claw = claw;
        addRequirements(swerve, claw, arm);
    }
    @Override
    public void initialize() {
       new SequentialCommandGroup(
   
            new OneCubeAuto(swerve, claw, arm, wrist, SwerveDrive.headless),
            new DriveDistance(swerve, -1, 0, -.3)
            //Need to add balance together when it is found to be functional
            //new balance_alt(swerve)

       ).schedule();
    }
    @Override
    public boolean isFinished() {
        return true;
    }
}