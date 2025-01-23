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
        FIRE(1),
        DOUBLE_TRIGGER_1(0),
        DOUBLE_TRIGGER_2(14),
        A(2),
        B(3),
        C(4),
        PINKY_TRIGGER(5),
        DPAD_UP_JOYSTICK(15),
        DPAD_RIGHT_JOYSTICK(16),
        DPAD_DOWN_JOYSTICK(17),
        DPAD_LEFT_JOYSTICK(18),
        T1(8),
        T2(9),
        T3(10),
        T4(11),
        T5(12),
        T6(13),
        //Throttle
        D(6),
        E(7),
        DPAD_UP_THROTTLE(19),
        DPAD_RIGHT_THROTTLE(20),
        DPAD_DOWN_THROTTLE(21),
        DPAD_LEFT_THROTTLE(22);

        int button;

        Button(int button) {
            this.button = button;
        }

        public int getButton() {
            return this.button;
        }
    }
}
