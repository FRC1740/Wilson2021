// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/LimeLight.h"

LimeLight::LimeLight() {
    m_Limelight_Table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
    //m_nte_Limelight = m_Limelight_Table->GetEntry();
    m_sbt_Vision = &frc::Shuffleboard::GetTab(ConShuffleboard::VisionTab);
    //m_nte_Align_P = m_sbt_Vision->AddPersistent("Vision P", 1.0)  .WithSize(1, 1).WithPosition(0, 0).GetEntry();;
    //m_nte_Align_I = m_sbt_Vision->AddPersistent("Vision I", 0.0)  .WithSize(1, 1).WithPosition(0, 1).GetEntry();;
    //m_nte_Align_D = m_sbt_Vision->AddPersistent("Vision D", 100.0).WithSize(1, 1).WithPosition(0, 2).GetEntry();;

    this->SetLEDMode(ConLimelight::LED_ON);
}

// This method will be called once per scheduler run
void LimeLight::Periodic() {}

double LimeLight::GetValidTarget(){
    return m_Limelight_Table->GetNumber("tv", 0.0);
}

double LimeLight::GetHorizontalOffset(){
    return m_Limelight_Table->GetNumber("tx", 0.0);
}

double LimeLight::GetVerticalOffset(){
    return m_Limelight_Table->GetNumber("ty", 0.0);
}

double LimeLight::GetTargetArea(){
    return m_Limelight_Table->GetNumber("ta", 0.0);
}

double LimeLight::GetTargetDistance(){
    return (ConLimelight::TARGET_HEIGHT-ConLimelight::CAMERA_HEIGHT)/(tan(GetVerticalOffset()*ConMath::DEG_2_RAD));
}

void LimeLight::SetLEDMode(int mode){
    m_Limelight_Table->PutNumber("ledMode", mode);
}

void LimeLight::SetCamMode(int mode){
    m_Limelight_Table->PutNumber("camMode", mode);
}

void LimeLight::SetSnapshot(int mode){
    m_Limelight_Table->PutNumber("snapshot", mode);
}