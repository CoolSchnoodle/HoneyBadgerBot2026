package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;

public class LocationUtils {
  public static Rotation2d getDirectionToLocation(
      Translation2d startPosition, Translation2d endPosition) {
    Translation2d difference = endPosition.minus(startPosition);
    return difference.getAngle();
  }

  public static Distance getDistanceToLocation(Translation2d position1, Translation2d position2) {
    return Meters.of(position1.getDistance(position2));
  }

  public static boolean isRedAlliance() {
    var alliance = DriverStation.getAlliance();
    return alliance.isEmpty() || (alliance.get() == DriverStation.Alliance.Red);
  }

  public static Translation3d getCurrentHubLocation() {
    return isRedAlliance()
        ? Locations.redHub
        : Locations.blueHub;
  }

  public static class Locations {
    public static final Translation3d blueHub = new Translation3d(4.6, 4.034, 1.477);
    public static final Translation3d redHub;

    public static final Translation2d leftCornerBlueLocation = new Translation2d(1, 7);
    public static final Translation2d rightCornerBlueLocation = new Translation2d(1, 1);

    public static final Translation2d leftCornerRedLocation;
    public static final Translation2d rightCornerRedLocation;

    static {
      Translation2d redHubPosition = FlippingUtil.flipFieldPosition(blueHub.toTranslation2d());
      redHub =
          new Translation3d(
              redHubPosition.getMeasureX(), redHubPosition.getMeasureY(), blueHub.getMeasureZ());

      leftCornerRedLocation = FlippingUtil.flipFieldPosition(leftCornerBlueLocation);
      rightCornerRedLocation = FlippingUtil.flipFieldPosition(rightCornerBlueLocation);
    }
  }
}
