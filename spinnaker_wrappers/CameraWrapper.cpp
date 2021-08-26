/******************************************************************************/
/*                                                                            */
/*  Copyright (C) 2018, FLIR Systems                                          */
/*  All rights reserved.                                                      */
/*                                                                            */
/******************************************************************************/
#include "CameraWrapper.h"

using namespace flir_adk_ethernet;

CameraWrapper::CameraWrapper() {}
CameraWrapper::CameraWrapper(Spinnaker::CameraPtr cam) : _cam(cam) {}

CameraWrapper::CameraWrapper(const CameraWrapper& wrapper) {
    _cam = wrapper._cam;
}

CameraWrapper::~CameraWrapper() {
    _cam = nullptr;
}

void CameraWrapper::Init() {
    _cam->Init();
}


bool CameraWrapper::IsInitialized() {
    return _cam->IsInitialized();
}

bool CameraWrapper::IsValid() {
    return _cam->IsValid();
}

INodeMap& CameraWrapper::GetNodeMap() {
    return _cam->GetNodeMap();
}

INodeMap& CameraWrapper::GetTLDeviceNodeMap() {
    return _cam->GetTLDeviceNodeMap();
}

void CameraWrapper::RegisterEvent(Spinnaker::EventHandler &e) {
    _cam->RegisterEventHandler(e);
}

void CameraWrapper::UnregisterEvent(Spinnaker::EventHandler &e) {
    _cam->UnregisterEventHandler(e);
}

void CameraWrapper::BeginAcquisition() {
    _cam->BeginAcquisition();
}

void CameraWrapper::EndAcquisition() {
    _cam->EndAcquisition();
}

void CameraWrapper::DeInit() {
    _cam->DeInit();
}
