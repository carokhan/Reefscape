// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
  public static final boolean tuningMode = true;

  public static boolean disableHAL = false;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class RobotMap {
    public static class Drive {
      public static final int pigeon = 0;

      public static final int frontLeftDrive = 1;
      public static final int frontLeftTurn = 2;
      public static final int frontLeftEncoder = 0;

      public static final int frontRightDrive = 3;
      public static final int frontRightTurn = 4;
      public static final int frontRightEncoder = 1;

      public static final int backLeftDrive = 5;
      public static final int backLeftTurn = 6;
      public static final int backLeftEncoder = 2;

      public static final int backRightDrive = 7;
      public static final int backRightTurn = 8;
      public static final int backRightEncoder = 3;

      public static final Rotation2d frontLeftOffset = new Rotation2d(0.0);
      public static final Rotation2d frontRightOffset = new Rotation2d(0.0);
      public static final Rotation2d backLeftOffset = new Rotation2d(0.0);
      public static final Rotation2d backRightOffset = new Rotation2d(0.0);
    }

    public static class Elevator {
      public static final int left = 10;
      public static final int right = 11;
    }

    public static class Outtake {
      public static final int motor = 20;
      public static final int canandcolor = 21;
    }
  }
}
