package frc.robot.commands.elevator;

public class ElevatorToPoint0Position extends ElevatorPositionCommandBase {
    @Override
    public void initialize() {
        m_elevator.setPositionWithRequest(m_request.withPosition(0.25));
    }
} 