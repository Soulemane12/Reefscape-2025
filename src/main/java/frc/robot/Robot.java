// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.Timer;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  private static final boolean kUseLimelight = true;

  public Robot() {
    m_robotContainer = new RobotContainer();
  }


  
  @Override
  public void robotInit() {

    PathfindingCommand.warmupCommand().schedule();
  }

  
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {

    // Seed field-centric orientation at the start of teleop
    m_robotContainer.drivetrain.runOnce(() -> m_robotContainer.drivetrain.seedFieldCentric());

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {

    if (kUseLimelight) {
      var driveState = m_robotContainer.drivetrain.getState();
      double headingDeg = driveState.Pose.getRotation().getDegrees();
      // Add 180 degrees to account for the camera being mounted at the front of the robot
      double adjustedHeadingDeg = headingDeg + 180.0;
      double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);

      LimelightHelpers.SetRobotOrientation("limelight", adjustedHeadingDeg, 0, -45, 0, 0, 0);
      var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
      if (llMeasurement != null && llMeasurement.tagCount > 0 && Math.abs(omegaRps) < 2.0) {
        m_robotContainer.drivetrain.addVisionMeasurement(llMeasurement.pose, llMeasurement.timestampSeconds);
      }
    }
  
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    // Seed field-centric orientation at the start of teleop
    m_robotContainer.drivetrain.runOnce(() -> m_robotContainer.drivetrain.seedFieldCentric());
  }

  @Override
  public void teleopPeriodic() {
    if (kUseLimelight) {
      var driveState = m_robotContainer.drivetrain.getState();
      double headingDeg = driveState.Pose.getRotation().getDegrees();
      // Add 180 degrees to account for the camera being mounted at the front of the robot
      double adjustedHeadingDeg = headingDeg + 180.0;
      double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);

      LimelightHelpers.SetRobotOrientation("limelight", adjustedHeadingDeg, 0, -45, 0, 0, 0);
      var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
      if (llMeasurement != null && llMeasurement.tagCount > 0 && Math.abs(omegaRps) < 2.0) {
        m_robotContainer.drivetrain.addVisionMeasurement(llMeasurement.pose, llMeasurement.timestampSeconds);
      }
    }
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }

  @Override
  public void simulationPeriodic() {
    // Simulate Limelight data for auto alignment in simulation
    if (kUseLimelight) {
      // Get current robot state
      var driveState = m_robotContainer.drivetrain.getState();
      double headingDeg = driveState.Pose.getRotation().getDegrees();
      // Add 180 degrees to account for the camera being mounted at the front of the robot
      double adjustedHeadingDeg = headingDeg + 180.0;

      // Update Limelight simulation data
      LimelightHelpers.SetRobotOrientation("limelight", adjustedHeadingDeg, 0, -45, 0, 0, 0);
      
      // Simulate target visibility
      var ntTable = LimelightHelpers.getLimelightNTTable("limelight");
      ntTable.getEntry("tv").setDouble(1.0); // Set tv to 1 to simulate a visible target
      
      // Simulate target pose data
      // Set a simulated pose in target space - x forward, y left, z up, roll, pitch, yaw
      double[] targetSpacePose = {0.0, 0.0, -0.5, 0.0, 0.0, 0.0}; // Default value
      ntTable.getEntry("botpose_targetspace").setDoubleArray(targetSpacePose);
      
      // Create simulated MegaTag pose data too
      var pose = driveState.Pose;
      var llMeasurement = new LimelightHelpers.PoseEstimate(
          pose, // Use current robot pose
          Timer.getFPGATimestamp(), // Current time
          0.02, // Simulated latency
          1, // Simulate seeing one tag
          1.0, // Tag span
          2.0, // Avg tag distance
          0.2, // Avg tag area
          new LimelightHelpers.RawFiducial[]{
              new LimelightHelpers.RawFiducial(7, 0.0, 0.0, 0.2, 2.0, 2.0, 0.0)
          },
          true // Is MegaTag2
      );
      
      // Add the simulated vision measurement to the drivetrain
      m_robotContainer.drivetrain.addVisionMeasurement(llMeasurement.pose, llMeasurement.timestampSeconds);
    }
  }

  @Override
  public void simulationInit() {
    // Initialize simulation-specific configurations
    System.out.println("Simulation mode initialized");
    
    // Set up simulation-specific NetworkTable entries for Limelight
    if (kUseLimelight) {
      var limelightTable = LimelightHelpers.getLimelightNTTable("limelight");
      
      // Ensure the tv entry exists and is set to 1 (target visible)
      limelightTable.getEntry("tv").setDouble(1.0);
      
      // Ensure basic pose entries exist for the simulation
      double[] defaultPose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      limelightTable.getEntry("botpose").setDoubleArray(defaultPose);
      limelightTable.getEntry("botpose_wpiblue").setDoubleArray(defaultPose);
      limelightTable.getEntry("botpose_targetspace").setDoubleArray(defaultPose);
      
      // Set other necessary values
      limelightTable.getEntry("botpose_tagcount").setDouble(1.0);
      
      System.out.println("Limelight simulation initialized in Robot class");
    }
  }
}
