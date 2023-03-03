// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  private Color kAllianceColor;
  /** Creates a new LEDs. */
  public LEDs() {
    // m_led.setLength(48);
    m_signalLED.setLength(15);
    
    // setSignal(new Color(50, 50, 0));
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
  public void periodic() {
    // if (Timer.getFPGATimestamp() % 5 < 0.021) {
    //   if (isCone) {
    //     setCube();
    //     isCone = false;
    //   }
    //   else {
    //     setCone();
    //     isCone = true;
    // }
  // }
    // if (Timer.getFPGATimestamp() > 0.2 && !isTeamSet)  {
    //   kAllianceColor  = (DriverStation.getAlliance() == Alliance.Red) ? Color.kDarkRed : Color.kBlue;
    //   isTeamSet = true;
    //   setLEDS(kAllianceColor);
    // }
    // if (intaking && isTeamSet) {
    //   for (var i = 0; i < m_ledBuffer.getLength(); i++) {
    //     // Sets the specified LED to the RGB values for red
    //     m_ledBuffer.setLED(i, kAllianceColor);
    //   }
    //   setIntakeBlock(Color.kWhite, 3, 20.0); 
    //   m_led.setData(m_ledBuffer);
    //   m_led.start();
    // }
    // This method will be called once per scheduler run
  }


  // public void setIntakeBlock(Color color, Integer size, Double speed) {
  //   // if ((int) ((Timer.getFPGATimestamp() * speed) % 16) < size) {
  //     // for (var i = 0; i < 16; i++) {
  //       // m_ledBuffer.setLED(i, color);
  //     // }
  //   // } else {
  //     // for (var i = 0; i < 16; i++) {
  //       // m_ledBuffer.setLED(i, kAllianceColor);
  //     // }
  //   // }
  //   for (var i = 0; i < size; i++) {
  //     m_ledBuffer.setLED((33 + ((int) (Timer.getFPGATimestamp() * speed + i) % 16) - 1), color);
  //     m_ledBuffer.setLED((32 - ((int) (Timer.getFPGATimestamp() * speed + i) % 16) - 1), color);
  //     if(((int) (Timer.getFPGATimestamp() * speed + i) % 16) < 8) {
  //       m_ledBuffer.setLED(((9 + (int) (Timer.getFPGATimestamp() * speed + 8 + i) % 8) - 1), color);
  //       m_ledBuffer.setLED((8 - ((int) (Timer.getFPGATimestamp() * speed + 8 + i) % 8)), color);
  //     } else {
  //       for (var x = 0; x < 16; x++) {
  //         m_ledBuffer.setLED(x, kAllianceColor);
  //       }
  //     }
  //   }
  // }

  // public void setLEDS(Color color) {
  //   for (var i = 0; i < m_ledBuffer.getLength(); i++) {
  //     // Sets the specified LED to the RGB values for red
  //     m_ledBuffer.setLED(i, color);
  //  }
  //  m_led.setData(m_ledBuffer);
  //  m_led.start();
  // }

  // public void intaking(Boolean tf) {
  //   intaking = tf;
  //   if (!tf) setLEDS(kAllianceColor);
  // }

  // public void LEDsOff() {
  //   setLEDS(Color.kBlack);
  // }

}
