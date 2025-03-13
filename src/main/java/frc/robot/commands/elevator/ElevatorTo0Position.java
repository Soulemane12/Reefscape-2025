package frc.robot.commands.elevator;

public class ElevatorTo0Position extends ElevatorPositionCommandBase {
    @Override
    public void execute() {
        m_elevator.setPositionWithRequest(m_request.withPosition(0.0));
    }
}