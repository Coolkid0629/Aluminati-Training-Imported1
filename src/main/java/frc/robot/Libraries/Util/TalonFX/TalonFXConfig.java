package frc.robot.Libraries.Util.TalonFX;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import frc.robot.Libraries.Util.PIDConfig;

public class TalonFXConfig {
    private TalonFXStatusFrames statusFrames;
    private boolean reset;
    private double allowableClosedLoopError;

    private TalonFXFeedbackDevice feedbackDevice;
    private SensorInitializationStrategy integratedSensorInitializationStrategy = null;
    private NeutralMode neutralMode;

    private boolean inverted = false;
    private boolean sensorInverted = false;

    private StatorCurrentLimitConfiguration statorCurrentLimit;
    private SupplyCurrentLimitConfiguration supplyCurrentLimit;

    private double maxVel = 0;
    private double maxAccel = 0;
    private int sCurveStrength = 0;

    private PIDConfig PIDconfig = new PIDConfig(0, 0, 0);

    int encoderCountsPerRev = 4096;

    private boolean isFollower = false;;
    private WPI_TalonFX leadTalonFX;

    /**
     * Default constructor
     * 
     * @param statusFrames             A status frame object that contains the
     *                                 period for each type of status frame
     * @param reset                    If the motor should be reset to factory
     *                                 default
     * @param allowableClosedLoopError Allowed closed loop error
     * @param feedbackDevice           The feedback device (encoder) used by the
     *                                 motor
     * @param neutralMode              The motor's behavior when not moving
     * @param statorCurrentLimit       The max output current
     * @param supplyCurrentLimit       The max input current
     * @param inverted                 Whether the motor's direction is inverted
     * @param sensorInverted           Whether the sensor's direction is inverted
     * @param encoderCountsPerRev      The ticks per one revolution of the motor's
     *                                 encoder
     */
    public TalonFXConfig(TalonFXStatusFrames statusFrames, boolean reset, double allowableClosedLoopError,
            TalonFXFeedbackDevice feedbackDevice, NeutralMode neutralMode,
            StatorCurrentLimitConfiguration statorCurrentLimit, SupplyCurrentLimitConfiguration supplyCurrentLimit,
            boolean inverted, boolean sensorInverted,
            int encoderCountsPerRev) {
        this.statusFrames = statusFrames;
        this.reset = reset;
        this.allowableClosedLoopError = allowableClosedLoopError;

        this.feedbackDevice = feedbackDevice;
        this.neutralMode = neutralMode;

        this.statorCurrentLimit = statorCurrentLimit;
        this.supplyCurrentLimit = supplyCurrentLimit;

        this.inverted = inverted;
        this.sensorInverted = sensorInverted;

        this.encoderCountsPerRev = encoderCountsPerRev;
    }

    /**
     * PID Control
     * 
     * @param statusFrames             A status frame object that contains the
     *                                 period for each type of status frame
     * @param reset                    If the motor should be reset to factory
     *                                 default
     * @param allowableClosedLoopError Allowed closed loop error
     * @param feedbackDevice           The feedback device (encoder) used by the
     *                                 motor
     * @param neutralMode              The motor's behavior when not moving
     * @param statorCurrentLimit       The max output current
     * @param supplyCurrentLimit       The max input current
     * @param inverted                 Whether the motor's direction is inverted
     * @param sensorInverted           Whether the sensor's direction is inverted
     * @param encoderCountsPerRev      The ticks per one revolution of the motor's
     *                                 encoder
     * @param PIDconfig                The PIDF config for the motor
     */
    public TalonFXConfig(TalonFXStatusFrames statusFrames, boolean reset, double allowableClosedLoopError,
            TalonFXFeedbackDevice feedbackDevice, NeutralMode neutralMode,
            StatorCurrentLimitConfiguration statorCurrentLimit, SupplyCurrentLimitConfiguration supplyCurrentLimit,
            boolean inverted, boolean sensorInverted,
            int encoderCountsPerRev,
            PIDConfig PIDconfig) {
        this(statusFrames, reset, allowableClosedLoopError,
                feedbackDevice, neutralMode,
                statorCurrentLimit, supplyCurrentLimit,
                inverted, sensorInverted,
                encoderCountsPerRev);

        this.PIDconfig = PIDconfig;

    }

    /**
     * Motion Magic Control
     * 
     * @param statusFrames             A status frame object that contains the
     *                                 period for each type of status frame
     * @param reset                    If the motor should be reset to factory
     *                                 default
     * @param allowableClosedLoopError Allowed closed loop error
     * @param feedbackDevice           The feedback device (encoder) used by the
     *                                 motor
     * @param neutralMode              The motor's behavior when not moving
     * @param statorCurrentLimit       The max output current
     * @param supplyCurrentLimit       The max input current
     * @param inverted                 Whether the motor's direction is inverted
     * @param sensorInverted           Whether the sensor's direction is inverted
     * @param encoderCountsPerRev      The ticks per one revolution of the motor's
     *                                 encoder
     * @param PIDconfig                The PIDF config for the motor
     * @param maxVel                   The maximum velocity when under motion magic
     *                                 control
     * @param maxAccel                 The maximum acceleration when under motion
     *                                 magic control
     * @param sCurveStrength           The curve strength when under motion magic
     *                                 control
     */
    public TalonFXConfig(TalonFXStatusFrames statusFrames, boolean reset, double allowableClosedLoopError,
            TalonFXFeedbackDevice feedbackDevice, NeutralMode neutralMode,
            StatorCurrentLimitConfiguration statorCurrentLimit, SupplyCurrentLimitConfiguration supplyCurrentLimit,
            boolean inverted, boolean sensorInverted,
            int encoderCountsPerRev,
            PIDConfig PIDconfig,
            Double maxVel, Double maxAccel, Integer sCurveStrength) {
        this(statusFrames, reset, allowableClosedLoopError,
                feedbackDevice, neutralMode,
                statorCurrentLimit, supplyCurrentLimit,
                inverted, sensorInverted,
                encoderCountsPerRev,
                PIDconfig);

        this.maxVel = maxVel;
        this.maxAccel = maxAccel;
        this.sCurveStrength = sCurveStrength;
    }

    /**
     * PID Control (Integrated Sensor)
     * 
     * @param statusFrames                           A status frame object that
     *                                               contains the period for each
     *                                               type of status frame
     * @param reset                                  If the motor should be reset to
     *                                               factory default
     * @param allowableClosedLoopError               Allowed closed loop error
     * @param feedbackDevice                         The feedback device (encoder)
     *                                               used by the motor
     * @param neutralMode                            The motor's behavior when not
     *                                               moving
     * @param statorCurrentLimit                     The max output current
     * @param supplyCurrentLimit                     The max input current
     * @param inverted                               Whether the motor's direction
     *                                               is inverted
     * @param sensorInverted                         Whether the sensor's direction
     *                                               is inverted
     * @param encoderCountsPerRev                    The ticks per one revolution of
     *                                               the motor's encoder
     * @param PIDconfig                              The PIDF config for the motor
     * @param integratedSensorInitializationStrategy The initalization strategy for
     *                                               the integrated sensor
     */
    public TalonFXConfig(TalonFXStatusFrames statusFrames, boolean reset, double allowableClosedLoopError,
            TalonFXFeedbackDevice feedbackDevice, NeutralMode neutralMode,
            StatorCurrentLimitConfiguration statorCurrentLimit, SupplyCurrentLimitConfiguration supplyCurrentLimit,
            boolean inverted, boolean sensorInverted,
            int encoderCountsPerRev,
            PIDConfig PIDconfig,
            SensorInitializationStrategy integratedSensorInitializationStrategy) {
        this(statusFrames, reset, allowableClosedLoopError,
                feedbackDevice, neutralMode,
                statorCurrentLimit, supplyCurrentLimit,
                inverted, sensorInverted,
                encoderCountsPerRev,
                PIDconfig);

        this.integratedSensorInitializationStrategy = integratedSensorInitializationStrategy;

    }

    /**
     * Motion magic control (Integrated Sensor)
     * 
     * @param statusFrames                           A status frame object that
     *                                               contains the period for each
     *                                               type of status frame
     * @param reset                                  If the motor should be reset to
     *                                               factory default
     * @param allowableClosedLoopError               Allowed closed loop error
     * @param feedbackDevice                         The feedback device (encoder)
     *                                               used by the motor
     * @param neutralMode                            The motor's behavior when not
     *                                               moving
     * @param statorCurrentLimit                     The max output current
     * @param supplyCurrentLimit                     The max input current
     * @param inverted                               Whether the motor's direction
     *                                               is inverted
     * @param sensorInverted                         Whether the sensor's direction
     *                                               is inverted
     * @param encoderCountsPerRev                    The ticks per one revolution of
     *                                               the motor's encoder
     * @param PIDconfig                              The PIDF config for the motor
     * @param integratedSensorInitializationStrategy The initalization strategy for
     *                                               the integrated sensor
     * @param maxVel                                 The maximum velocity when under
     *                                               motion magic control
     * @param maxAccel                               The maximum acceleration when
     *                                               under motion magic control
     * @param sCurveStrength                         The curve strength when under
     *                                               motion magic control
     */
    public TalonFXConfig(TalonFXStatusFrames statusFrames, boolean reset, double allowableClosedLoopError,
            TalonFXFeedbackDevice feedbackDevice, NeutralMode neutralMode,
            StatorCurrentLimitConfiguration statorCurrentLimit, SupplyCurrentLimitConfiguration supplyCurrentLimit,
            boolean inverted, boolean sensorInverted,
            int encoderCountsPerRev,
            PIDConfig PIDconfig,
            SensorInitializationStrategy integratedSensorInitializationStrategy,
            Double maxVel, Double maxAccel, Integer sCurveStrength) {
        this(statusFrames, reset, allowableClosedLoopError,
                feedbackDevice, neutralMode,
                statorCurrentLimit, supplyCurrentLimit,
                inverted, sensorInverted,
                encoderCountsPerRev,
                PIDconfig,
                integratedSensorInitializationStrategy);

        this.maxVel = maxVel;
        this.maxAccel = maxAccel;
        this.sCurveStrength = sCurveStrength;
    }

    /**
     * Config for a follower motor
     * 
     * @param statusFrames                           A status frame object that
     *                                               contains the period for each
     *                                               type of status frame
     * @param reset                                  If the motor should be reset to
     *                                               factory default
     * @param neutralMode                            The motor's behavior when not
     *                                               moving
     * @param statorCurrentLimit                     The max output current
     * @param supplyCurrentLimit                     The max input current
     * @param inverted                               Whether the motor's direction
     *                                               is inverted
     * @param leadTalonFX                            The TalonFX motor to follow
     */
    public TalonFXConfig(TalonFXStatusFrames statusFrames, boolean reset,
            NeutralMode neutralMode,
            StatorCurrentLimitConfiguration statorCurrentLimit, SupplyCurrentLimitConfiguration supplyCurrentLimit,
            boolean inverted,
            WPI_TalonFX leadTalonFX) {
                this.statusFrames = statusFrames;
                this.reset = reset;

                this.neutralMode = neutralMode;

                this.statorCurrentLimit = statorCurrentLimit;
                this.supplyCurrentLimit = supplyCurrentLimit;

                this.inverted = inverted;

                this.isFollower = true;
                this.leadTalonFX = leadTalonFX;
    }

    public TalonFXStatusFrames getStatusFrames() {
        return this.statusFrames;
    }

    public void setStatusFrames(TalonFXStatusFrames statusFrames) {
        this.statusFrames = statusFrames;
    }

    public boolean getReset() {
        return this.reset;
    }

    public void setReset(boolean reset) {
        this.reset = reset;
    }

    public double getAllowableClosedLoopError() {
        return this.allowableClosedLoopError;
    }

    public void setAllowableClosedLoopError(double allowableClosedLoopError) {
        this.allowableClosedLoopError = allowableClosedLoopError;
    }

    public TalonFXFeedbackDevice getFeedbackDevice() {
        return this.feedbackDevice;
    }

    public void setFeedbackDevice(TalonFXFeedbackDevice feedbackDevice) {
        this.feedbackDevice = feedbackDevice;
    }

    public SensorInitializationStrategy getIntegratedSensorInitializationStrategy() {
        return this.integratedSensorInitializationStrategy;
    }

    public void setIntegratedSensorInitializationStrategy(
            SensorInitializationStrategy integratedSensorInitializationStrategy) {
        this.integratedSensorInitializationStrategy = integratedSensorInitializationStrategy;
    }

    public NeutralMode getNeutralMode() {
        return this.neutralMode;
    }

    public void setNeutralMode(NeutralMode neutralMode) {
        this.neutralMode = neutralMode;
    }

    public boolean getInverted() {
        return this.inverted;
    }

    public void setInverted(boolean inverted) {
        this.inverted = inverted;
    }

    public boolean isSensorInverted() {
        return this.sensorInverted;
    }

    public boolean getSensorInverted() {
        return this.sensorInverted;
    }

    public void setSensorInverted(boolean sensorInverted) {
        this.sensorInverted = sensorInverted;
    }

    public StatorCurrentLimitConfiguration getStatorCurrentLimit() {
        return this.statorCurrentLimit;
    }

    public void setStatorCurrentLimit(StatorCurrentLimitConfiguration statorCurrentLimit) {
        this.statorCurrentLimit = statorCurrentLimit;
    }

    public SupplyCurrentLimitConfiguration getSupplyCurrentLimit() {
        return this.supplyCurrentLimit;
    }

    public void setSupplyCurrentLimit(SupplyCurrentLimitConfiguration supplyCurrentLimit) {
        this.supplyCurrentLimit = supplyCurrentLimit;
    }

    public double getMaxVel() {
        return this.maxVel;
    }

    public void setMaxVel(double maxVel) {
        this.maxVel = maxVel;
    }

    public double getMaxAccel() {
        return this.maxAccel;
    }

    public void setMaxAccel(double maxAccel) {
        this.maxAccel = maxAccel;
    }

    public int getSCurveStrength() {
        return this.sCurveStrength;
    }

    public void setSCurveStrength(int sCurveStrength) {
        this.sCurveStrength = sCurveStrength;
    }

    public PIDConfig getPIDconfig() {
        return this.PIDconfig;
    }

    public void setPIDconfig(PIDConfig PIDconfig) {
        this.PIDconfig = PIDconfig;
    }

    public int getEncoderCountsPerRev() {
        return this.encoderCountsPerRev;
    }

    public void setEncoderCountsPerRev(int encoderCountsPerRev) {
        this.encoderCountsPerRev = encoderCountsPerRev;
    }

    public boolean isFollower() {
        return this.isFollower;
    }

    public void setIsFollower(boolean isFollower) {
        this.isFollower = isFollower;
    }

    public WPI_TalonFX getLeadTalonFX() {
        return this.leadTalonFX;
    }

    public void setLeadTalonFX(WPI_TalonFX leadTalonFX) {
        this.leadTalonFX = leadTalonFX;
    }

}
