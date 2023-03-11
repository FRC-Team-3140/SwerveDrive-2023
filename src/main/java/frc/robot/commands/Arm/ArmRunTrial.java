package frc.robot.commands.Arm;

import com.revrobotics.CANSparkMax;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmRunTrial extends CommandBase{
    CANSparkMax motor;
    Supplier<Double> y_axis;

    public ArmRunTrial(CANSparkMax trialMotor, Supplier<Double> y_axis ){
        motor = trialMotor;
        
    }
    @Override
    public void initialize() {
        super.initialize();
    }
    public void execute(){
        super.execute();
        double y = y_axis.get();
        motor.setVoltage(5 * y);
    }
    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);
    }
    
    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return false;
    }


}

