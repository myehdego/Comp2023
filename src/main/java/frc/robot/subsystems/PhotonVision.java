// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVision extends SubsystemBase {
  PhotonCamera camera;
  PhotonPipelineResult result;
  /** Using a camera to get data from AprilTags
   * The data recived would be the area, pitch and yaw of an April tag 
   * Using the data we recive from the april tags we can determine the position of our robot relative to the tag
  */
  public PhotonVision() {
    camera = new PhotonCamera("photonvision");
    result = camera.getLatestResult();
  }
  

    
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
