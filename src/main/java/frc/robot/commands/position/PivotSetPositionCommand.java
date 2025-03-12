package frc.robot.commands.position;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

public class PivotSetPositionCommand extends Command {
    private final Pivot m_pivot;
    private final MotionMagicVoltage m_request;
    private final double m_targetPosition;
    private static final double POSITION_TOLERANCE = 0.05; // Adjust as needed

    public PivotSetPositionCommand(Pivot pivot, MotionMagicVoltage request, double targetPosition) {
        m_pivot = pivot;
        m_request = request;
        m_targetPosition = targetPosition;
        addRequirements(pivot);
    }

    @Override
    public void initialize() {
        m_pivot.setShooterPosition(m_targetPosition);
    }

    @Override
    public void execute() {
        // Keep setting position to hold it
        m_pivot.setShooterPosition(m_targetPosition);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(m_pivot.getCurrentPosition() - m_targetPosition) < POSITION_TOLERANCE;
    }

    @Override
    public void end(boolean interrupted) {
        // Keep motor engaged to hold position instead of stopping
        m_pivot.holdPosition();
    }
}
