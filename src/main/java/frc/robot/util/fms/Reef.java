package frc.robot.util.fms;

import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Reef {
    public static enum CoralLow {
        NONE, HORIZONTAL_LEFT, HORIZONTAL_RIGHT, VERTICAL_TOP, VERTICAL_BOTTOM
    }
    public static enum CoralHigh {
        NONE, PRESENT
    }
    public static enum Algae {
        NONE, PRESENT
    }

    class ReefSide {
        public final CoralLow[][] L1 = {
            {CoralLow.NONE, CoralLow.NONE, CoralLow.NONE, CoralLow.NONE},
            {CoralLow.NONE, CoralLow.NONE, CoralLow.NONE, CoralLow.NONE}
        };
        public final CoralHigh[] L2 = {CoralHigh.NONE, CoralHigh.NONE};
        public final CoralHigh[] L3 = {CoralHigh.NONE, CoralHigh.NONE};
        public final CoralHigh[] L4 = {CoralHigh.NONE, CoralHigh.NONE};
        public final Algae[] algae_low_high;

        public static Algae[] getAlgaeStartPose(final int side){
            return side >= 17 ? // BLUE
                side % 2 == 0 ? new Algae[]{Algae.PRESENT, Algae.NONE}
                    : new Algae[]{Algae.NONE, Algae.PRESENT}
                : side % 2 == 0 ? new Algae[]{Algae.NONE, Algae.PRESENT} // RED
                    : new Algae[]{Algae.PRESENT, Algae.NONE};
        }

        public ReefSide(final int side){
            this.algae_low_high = getAlgaeStartPose(side);
        }
    }

    public static final int TOTAL_REEF_SIDES = 6;
    public static final int BLUE_REEF_STARTING_ID = 17;
    public static final int RED_REEF_STARTING_ID = 6;
    public final ReefSide[] reef_sides = new ReefSide[TOTAL_REEF_SIDES];

    public Reef(Alliance alliance){
        for(int i = 0; i < TOTAL_REEF_SIDES; i++)
            reef_sides[i] = new ReefSide(i + ((alliance == Alliance.Blue) ? BLUE_REEF_STARTING_ID : RED_REEF_STARTING_ID));
    }
}
