package frc.robot.controller;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CommandSaitekController extends CommandGenericHID {
    private final Joystick saitek;

    public CommandSaitekController(final int port) {
        super(port);
        this.saitek = new Joystick(port);
    }

    public double joystickForwardAxis() {
        return saitek.getRawAxis(1);
    }

    public double joystickSideAxis() {
        return saitek.getRawAxis(0);
    }

    public double joystickRotationAxis() {
        return saitek.getRawAxis(5);
    }

    public double throttleMainAxis() {
        return saitek.getRawAxis(2);
    }

    public double throttleSliderAxis() {
        return saitek.getRawAxis(6);
    }

    public double throttleSmallDialAxis() {
        return saitek.getRawAxis(3);
    }

    public double throttleDialAxis() {
        return saitek.getRawAxis(4);
    }

    public Trigger joystickDoubleTriggerFirstStage() {
        return new Trigger(() -> saitek.getRawButton(0));
    }
    public Trigger joystickDoubleTriggerSecondStage() {
        return new Trigger(() -> saitek.getRawButton(7));
    }
    public Trigger joystickPinkyTrigger() {
        return new Trigger(() -> saitek.getRawButton(6));
    }

    public Trigger getTrigger(Button button) {
        return new Trigger(() -> saitek.getRawButton(button.getButton()));
    }

    public double getAxisValue(Axis axis) {
        return saitek.getRawAxis(axis.getAxis());
    }

    public enum Axis {
        // Joystick
        JOYSTICK_FORWARD(1),
        JOYSTICK_SIDE(0),
        JOYSTICK_ROTATION(5),
        // Throttle
        THROTTLE_MAIN(2),
        THROTTLE_SLIDER(6),
        THROTTLE_DIAL_SMALL(3),
        THROTTLE_DIAL_BIG(4);


        private int axis;

        Axis(int axis) {
            this.axis = axis;
        }

        public int getAxis() {
            return this.axis;
        }
    }

    public enum Button {
        // Joystick
        FIRE(2),
        DOUBLE_TRIGGER_1(1),
        DOUBLE_TRIGGER_2(15),
        A(3),
        B(4),
        C(5),
        PINKY_TRIGGER(6),
        DPAD_UP_JOYSTICK(16),
        DPAD_RIGHT_JOYSTICK(17),
        DPAD_DOWN_JOYSTICK(18),
        DPAD_LEFT_JOYSTICK(19),
        T1(9),
        T2(10),
        T3(11),
        T4(12),
        T5(13),
        T6(14),
        //Throttle
        D(7),
        E(8),
        I(30),
        DPAD_UP_THROTTLE(20),
        DPAD_RIGHT_THROTTLE(21),
        DPAD_DOWN_THROTTLE(22),
        DPAD_LEFT_THROTTLE(23);

        int button;

        Button(int button) {
            this.button = button;
        }

        public int getButton() {
            return this.button;
        }
    }
}
