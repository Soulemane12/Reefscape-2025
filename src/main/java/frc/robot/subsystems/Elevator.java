package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class Elevator extends SubsystemBase {
  // Two motors for the elevator drive.
  private final TalonFX m_motor1 = new TalonFX(ElevatorConstants.kElevatorMotorID1, "ChooChooTrain");
  private final TalonFX m_motor2 = new TalonFX(ElevatorConstants.kElevatorMotorID2, "ChooChooTrain");
  
  // Add Motion Magic control request
  private int m_printCount = 0;
  private double INITIAL_OFFSET = 0;
  private boolean hasInitialized = false;

  private final ShuffleboardTab m_elevatorTab;

  public Elevator() {
    // Configure both motors
    TalonFXConfiguration config = new TalonFXConfiguration();
    
    // Configure feedback and gear ratio if needed
    config.Feedback.SensorToMechanismRatio = 25.0; // Adjust based on your gear ratio
    
    // Configure Motion Magic m_printCount 
    config.MotionMagic.withMotionMagicCruiseVelocity(10.0)  // Reduced from 20.0
                      .withMotionMagicAcceleration(20.0)    // Reduced from 40.0
                      .withMotionMagicJerk(2000.0);         // Reduced from 4000.0
    
    // Configure PID values
    config.Slot0.kP = 21.6;  // Reduced from 60.0
    config.Slot0.kI = 0.0;   
    config.Slot0.kD = 0.18;   // Reduced from 0.5
    config.Slot0.kS = 0.25;  
    config.Slot0.kV = 1.1;   
    config.Slot0.kA = 0.05;  
    config.Slot0.kG = 0.1;

    // Apply configuration to both motors
    m_motor1.getConfigurator().apply(config);
    m_motor2.getConfigurator().apply(config);
    
    // Set motor2 to follow motor1 in opposite direction
    m_motor2.setControl(new Follower(m_motor1.getDeviceID(), true));

    // Create Shuffleboard tab for elevator
    m_elevatorTab = Shuffleboard.getTab("Elevator");
    m_elevatorTab.addDouble("Current Position", this::getPosition);
    m_elevatorTab.addDouble("Target Position", () -> m_motor1.getClosedLoopReference().getValueAsDouble());
    m_elevatorTab.addDouble("Position Error", () -> m_motor1.getClosedLoopError().getValueAsDouble());
  }

  /**
   * Moves the elevator to a specific position using Motion Magic
   * @param targetPosition The target position in rotations
   */
  // public void setPosition(double targetPosition) {
  //   m_motor1.setNeutralMode(NeutralModeValue.Brake);
  //   m_motor2.setNeutralMode(NeutralModeValue.Brake);
  //   m_motor1.setControl(m_motionMagic.withPosition(targetPosition).withSlot(0));
  // }

  // Overloaded method for DynamicMotionMagicVoltage input
  public void setPositionWithRequest(MotionMagicVoltage request) {
   // m_motor1.setNeutralMode(NeutralModeValue.Brake);
   // m_motor2.setNeutralMode(NeutralModeValue.Brake);
    m_motor2.setControl(new Follower(m_motor1.getDeviceID(), true));
    m_motor1.setControl(request.withPosition(-request.Position));
  }

  @Override
  public void periodic() {
    // Initialize offset once we get a non-zero reading
    if (!hasInitialized) {
      double currentPosition = m_motor1.getPosition().getValueAsDouble();
      if (currentPosition != 0) {
        INITIAL_OFFSET = currentPosition;
        hasInitialized = true;
        // System.out.println("Initialized elevator offset to: " + INITIAL_OFFSET);
      }
    }

   
        double position = (m_motor1.getPosition().getValueAsDouble() - INITIAL_OFFSET) * -1;
        double velocity = m_motor1.getVelocity().getValueAsDouble();

        System.out.println("Elevator Position: " + position);
    
    // Update Shuffleboard values are automatically handled by the addDouble() methods above
  }

  /**
   * MovessetPosition the elevator at the given speed.
   * Positive values move the elevator up and negative values move it down.
   *
   * @param speed A value between -1.0 and 1.0 representing motor output.
   */
  public void moveElevator(double speed) {
    // Ensure motor2 is following motor1
    m_motor2.setControl(new Follower(m_motor1.getDeviceID(), true));
    // Only control motor1, motor2 will follow
    m_motor1.setControl(new DutyCycleOut(-speed*0.5));
  }

  /** Stops the elevator. */
  public void stop() {
    moveElevator(0);
  }

  /**
   * Gets the current position of the elevator, adjusted for initial offset
   * @return The current position in rotations
   */
  public double getPosition() {
    return (m_motor1.getPosition().getValueAsDouble() - INITIAL_OFFSET) * -1;
}
}