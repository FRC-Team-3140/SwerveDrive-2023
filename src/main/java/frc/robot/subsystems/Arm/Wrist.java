package frc.robot.subsystems.Arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class Wrist extends SubsystemBase{
    private final double minAngleWrist = 0;
    private final double maxAngleWrist = 209;

    // Wrist Absolute Encoder
    public final DutyCycleEncoder wristEncoder = new DutyCycleEncoder(1);
    public final CANSparkMax wristSparkMax;
    double wristLength = Units.feetToMeters(23.75 / 12);
    DigitalInput limitSwitchWrist = RobotContainer.getlimitSwitchWrist();

    public Wrist(int WristID){
        wristSparkMax = new CANSparkMax(WristID, MotorType.kBrushless);
        
        wristSparkMax.restoreFactoryDefaults();
        wristSparkMax.setSmartCurrentLimit(20);
        wristSparkMax.setIdleMode(IdleMode.kBrake);
        wristSparkMax.burnFlash();

        wristSparkMax.getEncoder().setPosition(0.0);

    }
    




    public void setWristAngle(double angle) {
        Math.min(Math.max(angle, minAngleWrist), maxAngleWrist);
    }
    public void setWristVoltage(double wristVoltage) {
        System.out.println(getWristAngle());
        if(!limitSwitchWrist.get()){
          wristSparkMax.setVoltage(Math.min(wristVoltage,0));
        }else if(getWristAngle() < 2){
          wristSparkMax.setVoltage(Math.max(wristVoltage, 0));
        }else{
          wristSparkMax.setVoltage(wristVoltage);
        }
      }
    
    public DutyCycleEncoder getWristEncoder() {
        return wristEncoder;
      }
    public double getWristAngle() {
        double a = ((getWristEncoder().getAbsolutePosition() - .892239) % 1) * 360;
        return a < 0 ? a + 360 : a;
      }
      @Override
      public void periodic() {
        NetworkTableInstance.getDefault().getTable("Arm").getEntry("WristAngle").setDouble(getWristAngle());
        NetworkTableInstance.getDefault().getTable("Arm").getEntry("Wrist Voltage").setDouble(wristSparkMax.getBusVoltage());

      }
    
}
