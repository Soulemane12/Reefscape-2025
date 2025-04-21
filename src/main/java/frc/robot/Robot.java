// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

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
    
    // Update Field2d widget with robot pose
    m_robotContainer.m_field.setRobotPose(m_robotContainer.drivetrain.getState().Pose);
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
      double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);

      // Use the appropriate LimelightHelpers based on whether we're in simulation
      if (Robot.isSimulation()) {
        // In simulation, use our simulated Limelight data
        SimLimelightHelpers.SetRobotOrientation("limelight", headingDeg, 0, 0, 0, 0, 0);
        var llMeasurement = SimLimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        if (llMeasurement != null && llMeasurement.tagCount > 0 && Math.abs(omegaRps) < 2.0) {
          m_robotContainer.drivetrain.addVisionMeasurement(llMeasurement.pose, llMeasurement.timestampSeconds);
        }
      } else {
        // In real robot, use the actual Limelight data
        LimelightHelpers.SetRobotOrientation("limelight", headingDeg, 0, 0, 0, 0, 0);
        var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        if (llMeasurement != null && llMeasurement.tagCount > 0 && Math.abs(omegaRps) < 2.0) {
          m_robotContainer.drivetrain.addVisionMeasurement(llMeasurement.pose, llMeasurement.timestampSeconds);
        }
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
  }
}
