package frc.robot.subsystems.gripper;

public class Gripper {
  private final GripperIO io;
  private final GripperIOInputsAutoLogged inputs = new GripperIOInputsAutoLogged();

  public Gripper(GripperIO io) {
    this.io = io;
  }
}
