package frc.robot.commands.Balance;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Comms3140;
import frc.robot.subsystems.Swerve.SwerveDrive;

public class MoveToRamp extends CommandBase {
    public static final int kForward = 0;
    public static final int kBackward = 1;

    SwerveDrive m_drive;
    NetworkTable navxTable;
    double stopAngle;
    double rampApproachVelocity; // percentage multiplier for the NEO
    int m_direction = 0;
    Comms3140 comms = Comms3140.getInstance();

    public MoveToRamp(SwerveDrive swerveDrive, int direction) {
        addRequirements(swerveDrive);
        this.m_direction = direction;
        m_drive = swerveDrive;

        comms.registerDoubleSetting("BalanceMove", "Speed", () -> rampApproachVelocity, (value) -> {
            rampApproachVelocity = value;
        }, 0.5);
        comms.registerDoubleSetting("BalanceMove", "Stop Angle", () -> stopAngle, (value) -> {
            stopAngle = value;
        }, 0.5);

        navxTable = NetworkTableInstance.getDefault().getTable("SmartDashboard").getSubTable("DataNAVX");
    }

    @Override
    public void initialize() {
        SwerveDrive.headless = false;
    }

    @Override
    public void execute() {
        if (m_direction == kForward) {
            comms.sendStringTelemetry("BalanceMove", "Direction", "Forward");
            m_drive.setChassisSpeeds(rampApproachVelocity, 0, 0);
        } else {
            comms.sendStringTelemetry("BalanceMove", "Direction", "Forward");
            m_drive.setChassisSpeeds(-rampApproachVelocity, 0, 0);
        }
    }

    @Override
    public boolean isFinished() {
        double angle = navxTable.getEntry("navx_filtered_pitch").getDouble(0.0);
        boolean is_finished = Math.abs(angle) > stopAngle;

        comms.sendDoubleTelemetry("BalanceMove", "Angle", angle);
        comms.sendDoubleTelemetry("BalanceMove", "Stop Angle", stopAngle);
        comms.sendBooleanTelemetry("BalanceMove", "Is Finished", is_finished);

        return is_finished;
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.setChassisSpeeds(0, 0, 0);
    }
}
