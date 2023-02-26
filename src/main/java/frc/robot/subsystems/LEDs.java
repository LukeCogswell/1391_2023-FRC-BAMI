// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
  private AddressableLED m_led = new AddressableLED(0);
  private AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(48);
  public Boolean intaking = true;
  private Color kRed = new Color(40, 0, 0);
  /** Creates a new LEDs. */
  public LEDs() {
    m_led.setLength(48);
    setLEDS(kRed);
  }
  
  @Override
  public void periodic() {
    if (intaking && Timer.getFPGATimestamp() < 15) {
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        // Sets the specified LED to the RGB values for red
        m_ledBuffer.setLED(i, kRed);
      }
      setIntakeBlock(Color.kWhite, 3, 20.0); 
      m_led.setData(m_ledBuffer);
      m_led.start();
    } else setLEDS(kRed);
    // This method will be called once per scheduler run
  }

  public void setIntakeBlock(Color color, Integer size, Double speed) {
    for (var i = 0; i < size; i++) {
      m_ledBuffer.setLED((33 + ((int) (Timer.getFPGATimestamp() * speed + i) % 16) - 1), color);
      m_ledBuffer.setLED((32 - ((int) (Timer.getFPGATimestamp() * speed + i) % 16) - 1), color);
    }
  }

  public void setLEDS(Color color) {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      m_ledBuffer.setLED(i, color);
   }
   m_led.setData(m_ledBuffer);
   m_led.start();
  }

  public void LEDsOff() {
    setLEDS(Color.kBlack);
  }

}
