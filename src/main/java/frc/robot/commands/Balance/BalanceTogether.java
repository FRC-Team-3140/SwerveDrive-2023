package frc.robot.commands.Balance;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve.SwerveDrive;

public class BalanceTogether extends ParallelRaceGroup {

    SwerveDrive swerve;

    
    public BalanceTogether(SwerveDrive swerve,int direction) {
        this.swerve = swerve;

        SequentialCommandGroup balance = new SequentialCommandGroup(
            // Modify Speeds in the Commands below so that the robot doen't move too fast!
            new MoveToRamp(swerve,direction).withInterruptBehavior(InterruptionBehavior.kCancelSelf),
            new ClimbRamp(swerve).withInterruptBehavior(InterruptionBehavior.kCancelSelf),
            new BalanceAndEngage(swerve, direction).withInterruptBehavior(InterruptionBehavior.kCancelSelf)
        );
        addCommands(
            balance
        );    
    }
}
