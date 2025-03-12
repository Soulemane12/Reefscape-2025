package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  // Instantiate the motor on PWM/Can ID 10 (adjust if needed) as brushless.
  private final TalonFX m_shooterMotor =new TalonFX(50, "ChooChooTrain");
  private final DutyCycleOut m_shooterDutyCycleOut = new DutyCycleOut(0.2);
  private final DutyCycleOut m_shooterOutakeCycle = new DutyCycleOut(-0.2);
  private final DutyCycleOut zeroShooter = new DutyCycleOut(0);


  /** Sets the shooter motor speed.
   * @param speed The speed to set. Positive values shoot forward.
   */
  public void shoot(double speed) {
    // Reverse the sign if needed to match your motor wiring.
    m_shooterMotor.set(-speed);
  }

   public Command shooterIntakeControl(){
    return this.startEnd(
      () -> {
        m_shooterMotor.setControl(m_shooterDutyCycleOut);
      }, 
      () -> {
        m_shooterMotor.setControl(zeroShooter);
  });
  }

  public Command shooterOutakeControl(){
    return this.startEnd(
      () -> {
        m_shooterMotor.setControl(m_shooterOutakeCycle);
      }, 
      () -> {
        m_shooterMotor.setControl(zeroShooter);
  });
  }

  
  @Override
  public void periodic() {
    // Optional: add any periodic shooter updates here.
  }
}
