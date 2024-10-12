package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Optional;

public class ApriltagHelper {
  public enum Tags {
    SPEAKER_CENTER(7, 4),
    SPEAKER_OFFSET(8, 3),
    AMP(6, 5),
    SOURCE_SPEAKER_SIDE(2, 9), // Based on OWNER not side of field
    SOURCE_FAR_SIDE(1, 10), // Based on OWNER not side of field
    STAGE_CENTER(14, 13),
    STAGE_AMP_SIDE(15, 12),
    STAGE_SOURCE_SIDE(16, 11);

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
