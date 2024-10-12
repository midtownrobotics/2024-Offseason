package frc.robot.utils;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class ApriltagHelper {
    public enum Tags {
        SPEAKER_CENTER(7, 4),
        SPEAKER_OFFSET(8, 3),
        AMP(6, 5),
        HUMAN_PLAYER_SPEAKER_SIDE(2, 9), // Based on OWNER not side of field
        HUMAN_PLAYER_FAR_SIDE(1, 10), // Based on OWNER not side of field
        STAGE_CENTER(14, 13),
        STAGE_AMP_SIDE(15, 12),
        STAGE_HUMAN_PLAYER_SIDE(16, 11);

        private final int red;
        private final int blue;

        Tags(int blue, int red) {
            this.red = red;
            this.blue = blue;
        }

        public int getBlueId() {
            return blue;
        }

        public int getRedId() {
            return red;
        }

        public int getId() {
            Optional<Alliance> alliance = DriverStation.getAlliance();

            if (alliance.isPresent()) {
                if (alliance.get() == Alliance.Red) {
                    return getRedId();
                } else {
                    return getBlueId();
                }
            }

            return -1;
        }
    }
}
