package frc.robot.commands.Balance;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve.SwerveDrive;

public class BalanceTogether extends ParallelRaceGroup {

    SwerveDrive swerve;

    
    public BalanceTogether(SwerveDrive swerve) {
        this.swerve = swerve;

        SequentialCommandGroup balance = new SequentialCommandGroup(
            // Modify Speeds in the Commands below so that the robot doen't move too fast!
            //new MoveToRamp(swerve).withInterruptBehavior(InterruptionBehavior.kCancelSelf),
            //new EncoderDriveDistance(swerve, 1, 0.5, 0).withInterruptBehavior(InterruptionBehavior.kCancelSelf),
            new BalanceAndEngage(swerve).withInterruptBehavior(InterruptionBehavior.kCancelSelf)
        );
        addCommands(
            balance
        );    
    }
}
