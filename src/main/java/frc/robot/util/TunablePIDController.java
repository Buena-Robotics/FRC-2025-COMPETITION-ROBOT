package frc.robot.util;

import edu.wpi.first.math.controller.PIDController;

public class TunablePIDController extends PIDController {
    private final LoggedTunableNumber tunable_kp;
    private final LoggedTunableNumber tunable_ki;
    private final LoggedTunableNumber tunable_kd;
    public TunablePIDController(final String name, final double kp, final double ki, final double kd){
        super(kp, ki, kd);
        this.tunable_kp = new LoggedTunableNumber("PIDController/" + name + "/P", kp);
        this.tunable_ki = new LoggedTunableNumber("PIDController/" + name + "/I", ki);
        this.tunable_kd = new LoggedTunableNumber("PIDController/" + name + "/D", kd);
    }

    private void updateTuning(){
        if(tunable_kp.hasChanged(tunable_kp.hashCode())) super.setP(tunable_kp.get());
        if(tunable_ki.hasChanged(tunable_ki.hashCode())) super.setI(tunable_ki.get());
        if(tunable_kd.hasChanged(tunable_kd.hashCode())) super.setD(tunable_kd.get());
    }

    public double calculate(final double measurement){
        updateTuning();
        return super.calculate(measurement);
    }
    public double calculate(final double measurement, final double setpoint){
        updateTuning();
        return super.calculate(measurement, setpoint);
    }


}
