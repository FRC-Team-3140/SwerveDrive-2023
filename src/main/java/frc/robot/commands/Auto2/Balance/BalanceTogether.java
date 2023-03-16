package frc.robot.commands.Auto2.Balance;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve.SwerveDrive;

public class BalanceTogether extends SequentialCommandGroup {

    SwerveDrive swerve;

    public BalanceTogether(SwerveDrive swerve) {
        addCommands(
            //Modify Speeds in the Commands below so that the robot doen't move too fast!
            new MoveToRamp(swerve).withInterruptBehavior(InterruptionBehavior.kCancelSelf),
            new ClimbRamp(swerve).withInterruptBehavior(InterruptionBehavior.kCancelSelf),
            new BalanceAndEngage(swerve).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    }
}
