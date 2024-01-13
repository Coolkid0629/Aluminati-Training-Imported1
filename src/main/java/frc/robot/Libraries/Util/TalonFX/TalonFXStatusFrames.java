package frc.robot.Libraries.Util.TalonFX;

import java.util.ArrayList;

public class TalonFXStatusFrames {
    ArrayList<Integer> statusFrames = new ArrayList<Integer>();

    /**
     * Create a SparkMaxStatusFrames object
     * 
     * @param status1  Applied Motor Output, Fault Information, Limit Switch
     *                 Information
     * @param status2  Selected Sensor Position (PID 0), Selected Sensor Velocity
     *                 (PID 0), Brushed Supply Current Measurement, Sticky Fault
     *                 Information
     * @param status3  Quadrature Information
     * @param status4  Analog Input, Supply Battery Voltage, Controller Temperature
     * @param status8  Pulse Width Information
     * @param status10 Motion Profiling/Motion Magic Information
     * @param status12 Selected Sensor Position (Aux PID 1), Selected Sensor
     *                 Velocity (Aux PID 1)
     * @param status13 PID0 (Primary PID) Information
     * @param status14 PID1 (Auxiliary PID) Information
     * @param status21 Integrated Sensor Position (Talon FX), Integrated Sensor
     *                 Velocity (Talon FX)
     */

    public TalonFXStatusFrames(int status1, int status2, int status3,
            int status4, int status8, int status10,
            int status12, int status13, int status14, int status21) {
        this.statusFrames.add(status1);
        this.statusFrames.add(status2);
        this.statusFrames.add(status3);
        this.statusFrames.add(status4);
        this.statusFrames.add(status8);
        this.statusFrames.add(status10);
        this.statusFrames.add(status12);
        this.statusFrames.add(status13);
        this.statusFrames.add(status14);
        this.statusFrames.add(status21);

    }

    public ArrayList<Integer> getFrames() {
        return this.statusFrames;
    }
}
