package frc.robot.subsystems.Arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class Wrist extends SubsystemBase {
  private final double minAngleWrist = 0;
  private final double maxAngleWrist = 209;

  // Wrist Absolute Encoder
  public final DutyCycleEncoder wristEncoder = new DutyCycleEncoder(1);
  public final CANSparkMax wristSparkMax;
  private double wristP;
  private double wristD;
  private PIDController wristPidController = new PIDController(wristP, 0, wristD);
  private double WristAngle;
  private double WristAngleSetPt;
  // out of 12V
  private double maxVoltage = 4;
  private double motorVoltage = 0;

  double wristLength = Units.feetToMeters(23.75 / 12);
  DigitalInput limitSwitchWrist = RobotContainer.getlimitSwitchWrist();

  public Wrist(int WristID) {
    wristSparkMax = new CANSparkMax(WristID, MotorType.kBrushless);

    wristSparkMax.restoreFactoryDefaults();
    wristSparkMax.setSmartCurrentLimit(20);
    wristSparkMax.setIdleMode(IdleMode.kCoast);
    wristSparkMax.burnFlash();

    wristSparkMax.getEncoder().setPosition(0.0);

    // Initial P and D values for the PID Controller
    NetworkTableInstance.getDefault().getTable("Arm").getEntry("Wrist P").setDouble(0.4);
    NetworkTableInstance.getDefault().getTable("Arm").getEntry("Wrist D").setDouble(0.05);

  }

  // public void setWristVoltage(double wristVoltage) {
  // System.out.println(getWristAngle());
  // if (!limitSwitchWrist.get()) {
  // wristSparkMax.setVoltage(Math.min(wristVoltage, 0));
  // } else if (getWristAngle() < 2) {
  // wristSparkMax.setVoltage(Math.max(wristVoltage, 0));
  // } else {
  // wristSparkMax.setVoltage(wristVoltage);
  // }
  // }

  @Override
  public void periodic() {
    NetworkTableInstance.getDefault().getTable("Arm").getEntry("WristAngle").setDouble(getWristAngle());
    NetworkTableInstance.getDefault().getTable("Arm").getEntry("WristAngleSetPt").setDouble(WristAngleSetPt);
    NetworkTableInstance.getDefault().getTable("Arm").getEntry("Wrist PID Output").setDouble(wristPidController.calculate(WristAngle));
    NetworkTableInstance.getDefault().getTable("Arm").getEntry("Wrist Voltage").setDouble(motorVoltage);
    wristP = NetworkTableInstance.getDefault().getTable("Arm").getEntry("Wrist P").getDouble(0.0);
    wristD = NetworkTableInstance.getDefault().getTable("Arm").getEntry("Wrist D").getDouble(0.0);

    if (wristP != wristPidController.getP() || wristD != wristPidController.getD()) {
      wristPidController.setP(wristP);
      wristPidController.setD(wristD);
    }

    WristAngle = getWristAngle();

    wristPidController.setSetpoint(WristAngleSetPt);
    motorVoltage = wristPidController.calculate(WristAngle);
    if(Math.abs(motorVoltage) > maxVoltage){
      motorVoltage = Math.signum(motorVoltage)*maxVoltage;
    }

    wristSparkMax.setVoltage(motorVoltage);
  }

  // Getter methods
  public DutyCycleEncoder getWristEncoder() {
    return wristEncoder;
  }

  public double getWristAngle() {
    // the .892239 is correction factor that aligns the encoder to have 0 at the
    // minimum angle
    double a = ((getWristEncoder().getAbsolutePosition() - .892239)) * 360;
    return a < 0 ? a + 360 : a;
  }

  // Setter methods
  public void setWristAngle(double angle) {
    WristAngleSetPt = Math.min(Math.max(angle, minAngleWrist), maxAngleWrist);
  }
}
