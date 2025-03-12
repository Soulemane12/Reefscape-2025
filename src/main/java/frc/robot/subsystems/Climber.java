// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

//import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private final TalonFX m_climberMotor;
  private final DutyCycleOut m_climberRequest;
  private final DutyCycleOut zeroRequest;

  public Climber() {
    m_climberMotor = new TalonFX(56, "ChooChooTrain");
    m_climberRequest = new DutyCycleOut(1.0);
    zeroRequest = new DutyCycleOut(0);
  }

  public void runClimber(){
    m_climberMotor.setControl(m_climberRequest);
  }

  public Command climberControl(){
    return this.startEnd(
      () -> {
        m_climberMotor.setControl(m_climberRequest);
      }, 
      () -> {
        m_climberMotor.setControl(zeroRequest);
  });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
