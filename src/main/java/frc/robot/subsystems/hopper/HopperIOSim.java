package frc.robot.subsystems.hopper;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import java.util.Optional;
import java.util.function.Consumer;

public class HopperIOSim implements HopperIO {
  private final DCMotorSim motorSim;

  private Optional<Consumer<HopperIOInputsAutoLogged>> callback = Optional.empty();
  private double appliedVolts = 0.0;

  public HopperIOSim(
      ) {
    this.motorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getNEO(1), HopperConstants.moi, HopperConstants.gearing),
            DCMotor.getNEO(1));
  }

  @Override
  public void updateInputs(HopperIOInputsAutoLogged inputs) {
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