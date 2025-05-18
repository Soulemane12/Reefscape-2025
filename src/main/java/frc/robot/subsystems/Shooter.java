package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  // Instantiate the motor on PWM/Can ID 10 (adjust if needed) as brushless.
  private final TalonFX m_shooterMotor = new TalonFX(ShooterConstants.kShooterMotorID, "ChooChooTrain");
  private final DutyCycleOut m_shooterDutyCycleOut = new DutyCycleOut(1);
  private final DutyCycleOut m_shooterOutakeCycle = new DutyCycleOut(-1);
  private final DutyCycleOut zeroShooter = new DutyCycleOut(0);

  // Stall detection constants
  private static final double STALL_VELOCITY_THRESHOLD = 0.1; // Minimum velocity to consider not stalled
  private static final double STALL_CURRENT_THRESHOLD = 40.0; // Current threshold in amps
  private static final int STALL_DURATION_MS = 500; // Duration to consider a stall
  private long stallStartTime = 0;
  //private boolean isStalled = false;

  public Shooter() {
    // Configure current limits
    CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs()
      .withStatorCurrentLimit(45.0) // Maximum current in amps
      .withStatorCurrentLimitEnable(true)
      .withSupplyCurrentLimit(45.0)
      .withSupplyCurrentLimitEnable(true);

    m_shooterMotor.getConfigurator().apply(currentLimits);
  }

  /**
   * Sets the shooter motor speed.
   * 
   * @param speed The speed to set. Positive values shoot forward.
   */
  public void shoot(double speed) {
    // Reverse the sign if needed to match your motor wiring.
    m_shooterMotor.set(-speed);
  }

  /**
   * Checks if the motor is stalled based on velocity and current
   * @return true if the motor is stalled
   */
  private boolean isMotorStalled() {
    double velocity = m_shooterMotor.getVelocity().getValueAsDouble();
    double current = m_shooterMotor.getStatorCurrent().getValueAsDouble();
    
    boolean stallCondition = Math.abs(velocity) < STALL_VELOCITY_THRESHOLD && 
                            Math.abs(current) > STALL_CURRENT_THRESHOLD;
    
    if (stallCondition) {
      if (stallStartTime == 0) {
        stallStartTime = System.currentTimeMillis();
      } else if (System.currentTimeMillis() - stallStartTime > STALL_DURATION_MS) {
        return true;
      }
    } else {
      stallStartTime = 0;
    }
    
    return false;
  }

  public Command shooterIntakeControl() {
    return this.startEnd(
        () -> {
          m_shooterMotor.setControl(m_shooterDutyCycleOut);
        },
        () -> {
          m_shooterMotor.setControl(zeroShooter);
        }).until(this::isMotorStalled);
  }

  public Command shooterOutakeControl() {
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
    // Update SmartDashboard with motor status
    SmartDashboard.putNumber("Shooter Velocity", m_shooterMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Shooter Current", m_shooterMotor.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putBoolean("Shooter Stalled", isMotorStalled());
  }
}
