package frc.robot.subsystems.climb;

public class ClimbIOSpark implements ClimbIO {
    private final SparkMax spark = new SparkMax(ClimbConstants.spark, MotorType.kBrushless);
    private final SparkMaxConfig config = new SparkMaxConfig();
    private final SoftLimitConfig limits = new SoftLimitConfig();

    public ClimberIOSpark() {
        limits.forwardSoftLimitEnabled(true);
        limits.forwardSoftLimit(ClimbConstants.climbed.getRadians());
        limits.reverseSoftLimitEnabled(true);
        limits.reverseSoftLimitEnabled(ClimbConstants.stow.getRadians());
        
        config.inverted(ClimbConstants.inverted);
        config.encoder.positionConversionFactor(ClimbConstants.gearing * 2 * Math.PI);
        config.encoder.positionConversionFactor(ClimbConstants.gearing * 2 * Math.PI / 60.0);
        config.idleMode(IdleMode.kBrake);

        config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        config.softLimit.apply(limits);
        config.smartCurrentLimit(ClimbConstants.current);

        spark.configure(
        config
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

        spark.getEncoder.getPosition().setPosition(ClimbConstants.stow.getRadians());
    } 

    public void updateInputs(ClimbIOInputs inputs) {
        inputs.motorPositionRad = spark.getEncoder().getPosition();
        inputs.motorVelocityRadPerSec = spark.getEncoder().getVelocity();
    }
}
