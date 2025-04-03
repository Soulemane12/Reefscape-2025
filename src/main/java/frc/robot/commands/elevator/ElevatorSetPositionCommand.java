package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

public class ElevatorSetPositionCommand extends Command {
    private final Elevator m_elevator;
    private final MotionMagicVoltage m_request;
    private final double m_targetPosition;

    public ElevatorSetPositionCommand(Elevator elevator, MotionMagicVoltage request, double targetPosition) {
        m_elevator = elevator;
        m_request = request;
        m_targetPosition = targetPosition;
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        m_elevator.setPositionWithRequest(m_request.withPosition(m_targetPosition));
    }
}