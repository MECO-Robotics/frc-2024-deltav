package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.LED;

public class LEDSubsystem extends SubsystemBase {
  private final AddressableLED m_led = new AddressableLED(LED.PWMPORT);
  private final AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(LED.BUFFERSIZE);
  private int m_rainbowFirstPixelHue;
  private int chaserLocation = 0;
  private int increment = 0;

  public LEDSubsystem() {

    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  
  // --------------------------------------- Public functions?
  // I think these need to be able to override/interrupt the periodic

  public void rainbow() {
    // For every pixel
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
      // Set the value
      m_ledBuffer.setHSV(i, hue, 255, 128);
    }
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 3;
    // Check bounds
    m_rainbowFirstPixelHue %= 180;
    setLEDs();
  }

  /**
   * Moves (alliance colored?) chaser up from bottom on both sides
   */
  public void chaserIndex(boolean status) {
    int numLights = (m_ledBuffer.getLength() / 2);
    int numChaseOffOnPerPeriod = 3;
    int chaseLength = 5;
    int numIterations = numLights / numChaseOffOnPerPeriod;

    if (status) {

      setAll(Color.kBlack);

      for (int i = 0; i < chaseLength; i++) {
        int position1 = (m_ledBuffer.getLength() / 2) + (-chaserLocation) * numChaseOffOnPerPeriod - i;
        m_ledBuffer.setLED(position1, Color.kWhite);
        int position2 = (m_ledBuffer.getLength() / 2) + 1 + chaserLocation * numChaseOffOnPerPeriod + i;
        m_ledBuffer.setLED(position2, Color.kWhite);
      }

      chaserLocation = (chaserLocation + 1) % numIterations;

    }

    setLEDs();

  }


  public void setAll(Color color) {
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setLED(i, color);
    }
    setLEDs();
  }

  // --------------------------------------- Private functions?

  // alternate/shortcut to actually set the leds from the buffer
  private void setLEDs() {

    m_led.setData(m_ledBuffer);
  }

  /* 
  private void setFrontAll(Color color) {
    for (var i = 0; i < m_ledBuffer.getLength() / 2; i++) {
      m_ledBuffer.setLED(i, color);
    }
  }

  public void setFrontHalf() {
    for (int i = 0; i < m_ledBuffer.getLength() / 2; i++) {
      if (i < m_ledBuffer.getLength() / 2) {
        m_ledBuffer.setLED(i, Color.kBlue);
      } else {
        m_ledBuffer.setLED(i, Color.kRed);
      }
    }
  }

  public void setBackAll(Color color) {
    for (var i = m_ledBuffer.getLength() / 2; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setLED(i, color);
    }
  }
  */

  public void bothColors(boolean flipped) {
    Color color1;
    Color color2;
    
    if (flipped) {
      color1 = Color.kBlue;
      color2 = Color.kRed;
    } else {
      color1 = Color.kRed;
      color2 = Color.kBlue;
    }

    setAll(Color.kBlack);

    for (var i = 0; i < m_ledBuffer.getLength() / 2; i++) {
      m_ledBuffer.setLED(i, color1);
    }
    for (var i = 0; i > m_ledBuffer.getLength() / 2; i--) {
      m_ledBuffer.setLED(i, color2);
    }

  }
}