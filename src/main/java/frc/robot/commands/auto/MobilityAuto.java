package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drivetrain.DriveDistance;
import frc.robot.subsystems.Swerve.SwerveDrive;

public class MobilityAuto extends CommandBase{
    SwerveDrive swerve;
    public MobilityAuto(SwerveDrive swerve){
        this.swerve = swerve;

        


    }
    @Override
    public void initialize() {
        new SequentialCommandGroup(
            new DriveDistance(swerve, 1, .3, 0) 

        );
    }
     @Override
     public boolean isFinished() {
         return true;
     }

    
}
