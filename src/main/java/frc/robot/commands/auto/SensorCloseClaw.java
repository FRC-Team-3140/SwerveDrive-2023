package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Arm.Claw;

public class SensorCloseClaw extends CommandBase{
    Claw claw;
    public SensorCloseClaw(Claw claw){
        this.claw = claw;
        addRequirements(claw);
    }
    long startTime;
    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();
    }
    @Override
    public boolean isFinished() {
        return !RobotContainer.getPhotoElectricSensorValue() ||System.currentTimeMillis() - startTime > 4000;
    }
    @Override
    public void end(boolean interrupted) {
        claw.clawClosed();
    }
}
