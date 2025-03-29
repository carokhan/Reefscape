package frc.robot.subsystems.gripper;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import java.util.Optional;
import java.util.function.Consumer;

import frc.robot.util.ExtendedDCMotor;

public class GripperIOSim implements GripperIO {
  private final DCMotorSim motorSim;

  private Optional<Consumer<GripperIOInputsAutoLogged>> callback = Optional.empty();
  private double appliedVolts = 0.0;

  public GripperIOSim(
      ) {
    this.motorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                ExtendedDCMotor.getKrakenX44(1), GripperConstants.moi, GripperConstants.gearing),
            ExtendedDCMotor.getKrakenX44(1));
  }

  @Override
  public void updateInputs(GripperIOInputsAutoLogged inputs) {
    motorSim.update(0.02);
    inputs.motorVelocityRPM = motorSim.getAngularVelocityRPM();
    inputs.motorAppliedVolts = motorSim.getInputVoltage();
    inputs.motorCurrentAmps = motorSim.getCurrentDrawAmps();
    inputs.motorTempCelsius = 0.0;

    callback.ifPresent((cb) -> cb.accept(inputs));
  }

  @Override
  public void setVoltage(double voltage) {
    appliedVolts = voltage;
    motorSim.setInputVoltage(MathUtil.clamp(voltage, -12, 12));
  }
}