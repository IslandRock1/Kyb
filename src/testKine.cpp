//
// Created by Kinea on 07.10.2025.
//
#include <WiFi.h>
#include <ESPUI.h>
#include "SerialControl.hpp"
#include "../ControlCode/internetPassword.hpp"

// Reference your robot controller
SerialControl serial_control;

// Tab/control IDs
uint16_t tabMain, tabAdvanced;
uint16_t panelShoulder, panelWrist;
uint16_t sliderShoulder, numberShoulder, sliderWrist, numberWrist;
uint16_t labelShoulderActual, labelWristActual;

void sliderShoulderCb(Control *sender, int type, void* userParam) {
    if(type == SL_VALUE) {
        int v = sender->value.toInt();
        serial_control.inputData.targetPosition1 = v;
        ESPUI.updateNumber(numberShoulder, v);
    }
}
void numberShoulderCb(Control *sender, int type, void* userParam) {
    if(type == N_VALUE) {
        int v = sender->value.toInt();
        serial_control.inputData.targetPosition1 = v;
        ESPUI.updateSlider(sliderShoulder, v);
    }
}
void sliderWristCb(Control *sender, int type, void* userParam) {
    if(type == SL_VALUE) {
        int v = sender->value.toInt();
        serial_control.inputData.targetPosition0 = v;
        ESPUI.updateNumber(numberWrist, v);
    }
}
void numberWristCb(Control *sender, int type, void* userParam) {
    if(type == N_VALUE) {
        int v = sender->value.toInt();
        serial_control.inputData.targetPosition0 = v;
        ESPUI.updateSlider(sliderWrist, v);
    }
}

void setup() {
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) { delay(350); Serial.print("."); }
    Serial.println("\nWiFi connected!");

    // Tabs
    tabMain     = ESPUI.addControl(Tab, "Main Control", "", ControlColor::Peterriver,   Control::noParent);
    tabAdvanced = ESPUI.addControl(Tab, "Advanced",     "", ControlColor::Sunflower, Control::noParent);

    // Group Shoulder's controls in a panel, like the HTML layout
    panelShoulder = ESPUI.addControl(ControlType::Separator, "Shoulder Panel", "", ControlColor::None, tabMain);
    ESPUI.setPanelWide(panelShoulder, true); // Force wide panel
    sliderShoulder = ESPUI.slider("Shoulder Angle", sliderShoulderCb, ControlColor::Alizarin, 0, -180, 180, reinterpret_cast<void*>(static_cast<uintptr_t>(panelShoulder)));
    numberShoulder = ESPUI.number("Shoulder (deg)", numberShoulderCb, ControlColor::Sunflower, 0, -180, 180, reinterpret_cast<void*>(static_cast<uintptr_t>(panelShoulder)));

    // Group Wrist's controls in another panel
    panelWrist = ESPUI.addControl(ControlType::Separator, "Wrist Panel", "", ControlColor::None, tabMain);
    ESPUI.setPanelWide(panelWrist, true);
    sliderWrist = ESPUI.slider("Wrist Angle", sliderWristCb, ControlColor::Carrot, 0, -180, 180, reinterpret_cast<void*>(static_cast<uintptr_t>(panelWrist)));
    numberWrist = ESPUI.number("Wrist (deg)", numberWristCb, ControlColor::Carrot, 0, -180, 180, reinterpret_cast<void*>(static_cast<uintptr_t>(panelWrist)));

    // Advanced Tab, live feedback
    //ESPUI.label("Live Feedback", ControlColor::Emerald, "<b>Current Joint Positions:</b>", tabAdvanced);
    //labelShoulderActual = ESPUI.label("Shoulder Actual", ControlColor::Wetasphalt,      "0 deg", tabAdvanced);
    //labelWristActual    = ESPUI.label("Wrist Actual",    ControlColor::Wetasphalt,      "0 deg", tabAdvanced);

    ESPUI.addControl(ControlType::Label, "Live Feedback", "<b>Current Joint Positions:</b>", ControlColor::Emerald, tabAdvanced);
    labelShoulderActual = ESPUI.addControl(ControlType::Label, "Shoulder Actual", "0 deg" ,ControlColor::Wetasphalt, tabAdvanced);
    labelWristActual    = ESPUI.addControl(ControlType::Label, "Wrist Actual", "0 deg", ControlColor::Wetasphalt, tabAdvanced);

    ESPUI.separator(""); // Visual break
    ESPUI.label("System Status", ControlColor::Sunflower, "Ready");

    ESPUI.begin("2DOF Robot Arm ESPUI");
}

void loop() {
    ESPUI.updateLabel(labelShoulderActual, String(serial_control.outputData.currentPosition1, 1) + " deg");
    ESPUI.updateLabel(labelWristActual, String(serial_control.outputData.currentPosition0, 1) + " deg");
    delay(200);
}
