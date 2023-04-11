// package frc.robot.commands.Auto2.Balance;

// public class ClimbRamp {

// }

package frc.robot.commands.Balance;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Comms3140;
import frc.robot.subsystems.Swerve.SwerveDrive;

public class ClimbRamp extends CommandBase {
    SwerveDrive m_drive;
    NetworkTable navxTable;
    double stopAngle; // Needs to be less than the MoveToRamp stop angle!
    double climbSpeed; // Needs to be slow and controlled!
    double angle = stopAngle + 1; // Greater than stopAngle so that it doesn't stop instantly - JIC

    Comms3140 comms = Comms3140.getInstance();

    public ClimbRamp(SwerveDrive swerveDrive) {
        addRequirements(swerveDrive);

        m_drive = swerveDrive;
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        navxTable = inst.getTable("SmartDashboard").getSubTable("DataNAVX");

        comms.registerDoubleSetting("BalanceClimb", "speed", () -> climbSpeed, (value) -> {
            climbSpeed = value;
        }, 0.25);
        comms.registerDoubleSetting("BalanceClimb", "stopAngle", () -> stopAngle, (value) -> {
            stopAngle = value;
        }, 8.0);
        comms.sendStringTelemetry("BalanceClimb", "State", "Off");
    }

    public void initialize() {
        comms.sendStringTelemetry("BalanceClimb", "State", "Initialize");
    }

    @Override
    public void execute() {
        comms.sendStringTelemetry("BalanceClimb", "State", "Execute");

        angle = navxTable.getEntry("navx_filtered_pitch").getDouble(0.0);

        if (angle < 0) {
            m_drive.arcadeDrive(climbSpeed, 0);

        } else {
            m_drive.arcadeDrive(-climbSpeed, 0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        comms.sendStringTelemetry("BalanceClimb", "State", "Off");

        m_drive.setChassisSpeeds(0, 0, 0);
    }

    @Override
    public boolean isFinished() {
        boolean is_finished = Math.abs(angle) < stopAngle;

        comms.sendDoubleTelemetry("BalanceClimb", "Angle Stop", stopAngle);
        comms.sendDoubleTelemetry("BalanceClimb", "Angle Filtered", angle);
        comms.sendBooleanTelemetry("BalanceClimb", "Is Finished", is_finished);

        return is_finished;
    }

}
