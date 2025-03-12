package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class DriveToLimelightDistance extends Command {
    private final CommandSwerveDrivetrain drivebase;
    private final double KpDistance = -0.1; // Proportional control constant for distance
    private final SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric();
    
    public DriveToLimelightDistance(CommandSwerveDrivetrain drivebase) {
        this.drivebase = drivebase;
        addRequirements(drivebase);
    }

    @Override
    public void execute() {
        // Only adjust distance if we see a target
        if (LimelightHelpers.getTV("limelight")) {
            // Get the vertical offset from the crosshair (ty)
            // When the crosshair is calibrated, ty will be 0 at the desired distance
            double distance_error = LimelightHelpers.getTY("limelight");
            
            // Calculate the drive adjustment
            double driving_adjust = KpDistance * distance_error;
            
            // Drive forward/backward while maintaining current sideways position
            drivebase.setControl(robotCentric.withVelocityX(driving_adjust)
                                         .withVelocityY(0)
                                         .withRotationalRate(0));
        } else {
            // If no target is seen, stop the robot
            drivebase.setControl(robotCentric.withVelocityX(0)
                                         .withVelocityY(0)
                                         .withRotationalRate(0));
        }
    }

    @Override
    public void end(boolean interrupted) {
        drivebase.setControl(robotCentric.withVelocityX(0)
                                     .withVelocityY(0)
                                     .withRotationalRate(0));
    }

    @Override
    public boolean isFinished() {
        // This command runs continuously until interrupted
        return false;
    }
} 