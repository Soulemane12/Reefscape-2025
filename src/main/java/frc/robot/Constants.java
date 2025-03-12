// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class ReefAlignmentConstants {
    // PID values - Start with these and tune based on robot behavior
    public static final double X_REEF_ALIGNMENT_P = 2.5;  // Forward/back movement
    public static final double Y_REEF_ALIGNMENT_P = 4.5;  // Left/right movement
    public static final double ROT_REEF_ALIGNMENT_P = 0.058;  // Rotation

    // Setpoints - These are the target positions relative to the AprilTag
    // You'll need to adjust these based on where you want the robot to stop
    public static final double X_SETPOINT_REEF_ALIGNMENT = -0.5;  // Distance in front of tag (meters)
    public static final double Y_SETPOINT_REEF_ALIGNMENT = 0.19;  // Side offset from tag (meters)
    public static final double ROT_SETPOINT_REEF_ALIGNMENT = 0;   // Angle relative to tag (degrees)

    // Tolerances - How close you need to be to consider it "aligned"
    public static final double X_TOLERANCE_REEF_ALIGNMENT = 0.005;  // Meters
    public static final double Y_TOLERANCE_REEF_ALIGNMENT = 0.005;  // Meters
    public static final double ROT_TOLERANCE_REEF_ALIGNMENT = 0.5;  // Degrees

    // Timing constants
    public static final double DONT_SEE_TAG_WAIT_TIME = 1;    // How long to wait before stopping if tag lost
    public static final double POSE_VALIDATION_TIME = 0.3;    // How long to stay aligned before finishing
  }

  public static class ElevatorConstants{
    public static final int kElevatorMotorID1 = 20;
    public static final int kElevatorMotorID2 = 21;

    // Elevator positions
    public static final double kElevatorL0Position = 0.25;  // Point 0 position
    public static final double kElevatorL2Position = 0.8;   // L2 position
    public static final double kElevatorL3Position = 1.95;   // L3 position
    public static final double kElevatorL4Position = 3.9;   // L4 position
  }
  public static class ShooterConstants{
    public static final int kShooterMotorID = 30;
  }
  public static class PivotConstants {
    public static final int kPivotMotorID = 30;  // Same as ShooterMotorID

    // Pivot positions in radians
    public static final double kPivotL0Position = 0.75;     // L0 position
    public static final double kPivotL2Position = -0.7;    // L2 position
    public static final double kPivotL3Position = -0.7;     // L3 position
    public static final double kPivotL4Position = -0.525;     // L4 position
    public static final double kPivotInPosition = -1;    // In position
    public static final double kPivotParallelPosition = -2.1;  // Parallel to ground
  }
}