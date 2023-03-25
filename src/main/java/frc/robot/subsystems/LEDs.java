// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
  // private AddressableLED m_led = new AddressableLED(0);
  // private AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(48);
  private AddressableLED m_signalLED = new AddressableLED(1);
  private AddressableLEDBuffer m_signalLEDBuffer = new AddressableLEDBuffer(15);
  public Boolean intaking = false;
  public Boolean isTeamSet = false;
  public boolean isCone = false;
  private Color kCone = new Color(255, 255, 0);
  private Color kCube = new Color(255, 0, 255);
  /** Creates a new LEDs. */
  public LEDs() {
    m_signalLED.setLength(15);
  }
  
  public void setCube() {
    for (var i = 0; i < m_signalLEDBuffer.getLength(); i++) {
      m_signalLEDBuffer.setLED(i, kCube);
    }
    m_signalLEDBuffer.setLED(0, Color.kBlack);
    m_signalLEDBuffer.setLED(4, Color.kBlack);
    m_signalLEDBuffer.setLED(5, Color.kBlack);
    m_signalLEDBuffer.setLED(9, Color.kBlack);
    m_signalLEDBuffer.setLED(10, Color.kBlack);
    m_signalLEDBuffer.setLED(14, Color.kBlack);
    m_signalLED.setData(m_signalLEDBuffer);
    m_signalLED.start();
    }

  public void setCone() {
    for (var i = 0; i < m_signalLEDBuffer.getLength(); i++) {
      m_signalLEDBuffer.setLED(i, kCone);
    }
    m_signalLEDBuffer.setLED(0, Color.kBlack);
    m_signalLEDBuffer.setLED(1, Color.kBlack);
    m_signalLEDBuffer.setLED(3, Color.kBlack);
    m_signalLEDBuffer.setLED(4, Color.kBlack);
    m_signalLEDBuffer.setLED(5, Color.kBlack);
    m_signalLEDBuffer.setLED(9, Color.kBlack);
    m_signalLED.setData(m_signalLEDBuffer);
    m_signalLED.start();
  }

  public void setSignal(Color color) {
    for (var i = 0; i < m_signalLEDBuffer.getLength(); i++) {
      m_signalLEDBuffer.setLED(i, color);
    }
    m_signalLED.setData(m_signalLEDBuffer);
    m_signalLED.start();
  }

  @Override
  public void periodic() {}
}
