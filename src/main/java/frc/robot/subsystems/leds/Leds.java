// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.state.RobotState;
import frc.robot.util.Enums.GameObject;
import frc.robot.util.VirtualSubsystem;
import java.util.List;
import java.util.Optional;

public class Leds extends VirtualSubsystem {
  private static Leds instance;

  public static Leds getInstance() {
    if (instance == null) {
      instance = new Leds();
    }
    return instance;
  }

  // Robot state tracking
  public int loopCycleCount = 0;
  public boolean estopped = false;

  private Optional<Alliance> alliance = Optional.empty();
  private Color allianceColor = Color.kDarkTurquoise;
  private Color coralColor = Color.kPurple;
  private Color algeaColor = Color.kGold;

  // LED IO
  private final AddressableLED leds;
  private final AddressableLEDBuffer buffer;
  private final Notifier loadingNotifier;

  // Constants
  private static final int minLoopCycleCount = 10;
  private static final int length = 50;
  private static final double breathDuration = 1.0;
  private static final double waveExponent = 0.4;
  private static final double waveFastCycleLength = 25.0;
  private static final double waveFastDuration = 0.25;

  private Leds() {
    leds = new AddressableLED(1);
    buffer = new AddressableLEDBuffer(length);
    leds.setLength(length);
    leds.setData(buffer);
    leds.start();

    loadingNotifier =
        new Notifier(
            () -> {
              synchronized (this) {
                breath(Color.kWhite, Color.kBlack, System.currentTimeMillis() / 1000.0);
                leds.setData(buffer);
              }
            });
    loadingNotifier.startPeriodic(0.02);
  }

  public synchronized void periodic() {
    // Update alliance color
    if (DriverStation.isFMSAttached()) {
      alliance = DriverStation.getAlliance();
      allianceColor =
          alliance
              .map(alliance -> alliance == Alliance.Blue ? Color.kBlue : Color.kRed)
              .orElse(Color.kGold);

      if (loadingNotifier != null) {
        loadingNotifier.stop();
      }
    }

    // Update estop state
    if (DriverStation.isEStopped()) {
      estopped = true;
    }

    // Exit during initial cycles
    loopCycleCount += 1;
    if (loopCycleCount < minLoopCycleCount) {
      return;
    }

    // Stop loading notifier if running
    loadingNotifier.stop();

    // Select LED mode
    wave(Color.kGreen, Color.kBlack, waveFastCycleLength, waveFastDuration);
    if (DriverStation.isEnabled()) {
      if (RobotState.isAuto()) {
        blink(Color.kRed, Color.kDarkBlue, 0.1); // çakar
      }
      if (RobotState.getGameObject() == GameObject.CORAL) {
        solid(coralColor);
      } else if (RobotState.getGameObject() == GameObject.ALGEA) {
        solid(algeaColor);
      }
    } else { // Disabled
      breath(allianceColor, Color.kBlack, System.currentTimeMillis() / 1000.0);
    }

    if (estopped) {
      breath(Color.kRed, Color.kOrangeRed, System.currentTimeMillis() / 1000.0);
    }

    // Update LEDs
    leds.setData(buffer);
  }

  private void solid(Color color) {
    if (color != null) {
      for (int i = 0; i < length; i++) {
        buffer.setLED(i, color);
      }
    }
  }

  private void strobe(Color c1, Color c2, double duration) {
    boolean c1On = ((Timer.getFPGATimestamp() % duration) / duration) > 0.5;
    solid(c1On ? c1 : c2);
  }

  @SuppressWarnings("unused")
  private void strobe(Color color, double duration) {
    strobe(color, Color.kBlack, duration);
  }

  @SuppressWarnings("unused")
  private void breath(Color c1, Color c2) {
    breath(c1, c2, Timer.getFPGATimestamp());
  }

  private void breath(Color c1, Color c2, double timestamp) {
    double x = ((timestamp % breathDuration) / breathDuration) * 2.0 * Math.PI;
    double ratio = (Math.sin(x) + 1.0) / 2.0;
    double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
    double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
    double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
    solid(new Color(red, green, blue));
  }

  @SuppressWarnings("unused")
  private void rainbow(double cycleLength, double duration) {
    double x = (1 - ((Timer.getFPGATimestamp() / duration) % 1.0)) * 180.0;
    double xDiffPerLed = 180.0 / cycleLength;
    for (int i = 0; i < length; i++) {
      x += xDiffPerLed;
      x %= 180.0;
      buffer.setHSV(i, (int) x, 255, 255);
    }
  }

  private void blink(Color c1, Color c2, double duration) {
    boolean isC1 = ((Timer.getFPGATimestamp() % duration) / duration) > 0.5;
    solid(isC1 ? c1 : c2);
  }

  private void wave(Color c1, Color c2, double cycleLength, double duration) {
    double x = (1 - ((Timer.getFPGATimestamp() % duration) / duration)) * 2.0 * Math.PI;
    double xDiffPerLed = (2.0 * Math.PI) / cycleLength;
    for (int i = 0; i < length; i++) {
      x += xDiffPerLed;
      double ratio = (Math.pow(Math.sin(x), waveExponent) + 1.0) / 2.0;
      if (Double.isNaN(ratio)) {
        ratio = (-Math.pow(Math.sin(x + Math.PI), waveExponent) + 1.0) / 2.0;
      }
      if (Double.isNaN(ratio)) {
        ratio = 0.5;
      }
      double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
      double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
      double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
      buffer.setLED(i, new Color(red, green, blue));
    }
  }

  @SuppressWarnings("unused")
  private void stripes(List<Color> colors, int stripeLength, double duration) {
    int offset =
        (int) (Timer.getFPGATimestamp() % duration / duration * stripeLength * colors.size());
    for (int i = 0; i < length; i++) {
      int colorIndex =
          (int) (Math.floor((double) (i - offset) / stripeLength) + colors.size()) % colors.size();
      colorIndex = colors.size() - 1 - colorIndex;
      buffer.setLED(i, colors.get(colorIndex));
    }
  }
}
