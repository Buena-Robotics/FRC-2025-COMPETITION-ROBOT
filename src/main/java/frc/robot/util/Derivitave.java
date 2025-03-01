package frc.robot.util;

public class Derivitave {
    private double last;
    private double current;
    public Derivitave(final double initial_value){
        this.last = initial_value;
        this.current = initial_value;
    }
    public double update(double updated_value){
        current = updated_value;
        final double ret = current - last;
        last = current;
        return ret;
    }
}
