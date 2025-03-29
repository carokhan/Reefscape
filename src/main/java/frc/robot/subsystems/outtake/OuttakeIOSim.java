package frc.robot.subsystems.outtake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.util.ExtendedDCMotor;

import java.util.Optional;
import java.util.function.Consumer;

public class OuttakeIOSim implements OuttakeIO {
  private final DCMotorSim motorSim;

  private Optional<Consumer<OuttakeIOInputsAutoLogged>> callback = Optional.empty();
  private double appliedVolts = 0.0;
  
  public OuttakeIOSim(
      ) {
    this.motorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                ExtendedDCMotor.getKrakenX44(1), OuttakeConstants.moi, OuttakeConstants.gearing),
            ExtendedDCMotor.getKrakenX44(1));
  }

  @Override
  public void updateInputs(OuttakeIOInputsAutoLogged inputs) {
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