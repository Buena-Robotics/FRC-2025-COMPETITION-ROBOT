package frc.robot.util;

import edu.wpi.first.wpilibj.SerialPort;

public class RS232DistanceSensor {
	private static final double IN_PER_MM = 1.0 / (10.0 * 2.54);
	private SerialPort serial = new SerialPort(9600, SerialPort.Port.kOnboard, 8, SerialPort.Parity.kNone, SerialPort.StopBits.kOne);
	private int current_value;

	public double getDistance() {
		return ((double) current_value) * IN_PER_MM;
	}

	public void periodic() {
		String next = serial.readString();
		System.out.print(next);
		// Content of message is R[1-4 digit number][carriage return]
		if((next.length() >= 3) && (next.length() <= 6) && (next.charAt(0) == 'R')) {
			String number = next.substring(1, next.length() - 1);
			current_value = Integer.parseInt(number);
		}
	}
}