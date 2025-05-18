package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;

/**
 * A command that sets up the simulation data for the Limelight.
 * This is only meant to be used in simulation mode to ensure that
 * the auto alignment features work properly in simulation.
 */
public class SetupLimelightSimulation extends Command {
    
    public SetupLimelightSimulation() {
    }
    
    @Override
    public void initialize() {
        // Only run this in simulation
        if (!isSimulation()) {
            return;
        }
        
        // Get the Limelight's NetworkTable
        NetworkTable limelightTable = LimelightHelpers.getLimelightNTTable("limelight");
        
        // Set up the basic entries needed for simulation
        limelightTable.getEntry("tv").setDouble(1.0);  // Target is visible
        
        // Simulate a tag being detected
        double[] targetSpacePose = {0.0, 0.0, -1.0, 0.0, 0.0, 0.0};  // Adjusted Z position to be lower
        limelightTable.getEntry("botpose_targetspace").setDoubleArray(targetSpacePose);
        
        // Simulate having MegaTag 2 detection
        double[] botpose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        limelightTable.getEntry("botpose").setDoubleArray(botpose);
        limelightTable.getEntry("botpose_wpiblue").setDoubleArray(botpose);
        limelightTable.getEntry("botpose_wpired").setDoubleArray(botpose);
        
        // Set tag count and related data
        limelightTable.getEntry("botpose_tagcount").setDouble(1.0);
        limelightTable.getEntry("botpose_span").setDouble(1.0);
        limelightTable.getEntry("botpose_avgdist").setDouble(2.0);
        limelightTable.getEntry("botpose_avgarea").setDouble(0.2);
        
        // Set pipeline data
        limelightTable.getEntry("getpipe").setDouble(0.0);
        limelightTable.getEntry("tl").setDouble(10.0); // Latency
        limelightTable.getEntry("cl").setDouble(5.0);  // Capture latency
        
        // Setup complete
        System.out.println("Limelight simulation setup complete");
    }
    
    @Override
    public boolean isFinished() {
        return true; // This command runs once and finishes immediately
    }
    
    private boolean isSimulation() {
        return edu.wpi.first.wpilibj.RobotBase.isSimulation();
    }
} 