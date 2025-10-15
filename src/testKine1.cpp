/* espui_2dof.ino
   Main UI for 2DOF robot using ESPUI (https://github.com/s00500/ESPUI).
   - Three tabs: Main Control, Internet/Login, Advanced (status + log)
   - Sliders <-> Number inputs kept in sync
   - Uses extern SerialControl serial_control (your existing implementation)
*/

#include <WiFi.h>
#include <ESPUI.h>
#include "SerialControl.hpp"  // keep your SerialControl implementation; extern below

extern SerialControl serial_control; // assume provided elsewhere (like you had earlier)

// --- WiFi (replace) ---
const char* WIFI_SSID = "your_ssid";
const char* WIFI_PASS = "your_password";

// --- UI IDs ---
uint16_t tabMain, tabLogin, tabAdvanced;

// Main control widgets
uint16_t sliderShoulder, numberShoulder;
uint16_t sliderWrist, numberWrist;
uint16_t btnHome, btnStop;

// Advanced widgets
uint16_t labelShoulderActual, labelWristActual;
uint16_t labelSystemStatus;
uint16_t textLog; // textarea for logs

// simple helper to append to the log textarea
void appendLog(const String &line) {
    // get current content (ESPUI doesn't provide a getter, so we maintain lastLog locally)
    static String lastLog = "";
    lastLog += line + "\n";
    ESPUI.updateText(textLog, lastLog.c_str());
}

// --- Callbacks to keep slider <-> number in sync and push targets to serial_control ---
// Shoulder
void sliderShoulderCallback(Control *sender, int type) {
    if (type == SL_VALUE) {
        int v = sender->value.toInt();
        serial_control.inputData.targetPosition1 = v;     // your mapping: shoulder is index 1
        ESPUI.updateNumber(numberShoulder, v);
    }
}
void numberShoulderCallback(Control *sender, int type) {
    if (type == N_VALUE) {
        int v = sender->value.toInt();
        serial_control.inputData.targetPosition1 = v;
        ESPUI.updateSlider(sliderShoulder, v);
    }
}

// Wrist
void sliderWristCallback(Control *sender, int type) {
    if (type == SL_VALUE) {
        int v = sender->value.toInt();
        serial_control.inputData.targetPosition0 = v;     // wrist is index 0
        ESPUI.updateNumber(numberWrist, v);
    }
}
void numberWristCallback(Control *sender, int type) {
    if (type == N_VALUE) {
        int v = sender->value.toInt();
        serial_control.inputData.targetPosition0 = v;
        ESPUI.updateSlider(sliderWrist, v);
    }
}

// Buttons
void btnHomeCallback(Control *sender, int type) {
    if (type == B_DOWN) {
        // example: home = set targets to 0
        serial_control.inputData.targetPosition0 = 0;
        serial_control.inputData.targetPosition1 = 0;
        ESPUI.updateSlider(sliderWrist, 0);
        ESPUI.updateNumber(numberWrist, 0);
        ESPUI.updateSlider(sliderShoulder, 0);
        ESPUI.updateNumber(numberShoulder, 0);
        appendLog("Home requested");
    }
}
void btnStopCallback(Control *sender, int type) {
    if (type == B_DOWN) {
        // stub: send stop command or set current targets to current actuals
        // If you have a stop protocol, call it here.
        appendLog("Stop requested");
    }
}

// Apply WiFi creds (in UI). This simply demonstrates getting text inputs and printing them;
// you can implement storing to flash and reconnect if needed.
uint16_t inputSSID, inputPass, btnApplyWiFi;
void btnApplyWiFiCallback(Control *sender, int type) {
    if (type == B_DOWN) {
        // read fields; unfortunately ESPUI does not have a direct getValue function,
        // but callbacks for text inputs can be used to store values to globals.
        appendLog("Apply WiFi pressed (use text callbacks to capture fields)");
    }
}
String pendingSSID = "";
String pendingPass = "";
void ssidTextCallback(Control *sender, int type) {
    if (type == T_ON_CHANGE) {
        pendingSSID = sender->value;
    }
}
void passTextCallback(Control *sender, int type) {
    if (type == T_ON_CHANGE) {
        pendingPass = sender->value;
    }
}

// --- Setup and Loop ---
void setup() {
    Serial.begin(115200);
    delay(100);

    // WiFi connect (attempt) - UI works even if not connected (ESPUI serves on network if available)
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    unsigned long sw = millis();
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED && millis() - sw < 8000) {
        delay(300);
        Serial.print(".");
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println();
        Serial.print("IP: "); Serial.println(WiFi.localIP());
        appendLog(String("WiFi connected: ") + WiFi.localIP().toString());
    } else {
        Serial.println();
        appendLog("WiFi not connected (UI will still start).");
    }

    // If SerialControl needs initialization (UART, PWM), do it here.
    // Example:
    // serial_control.beginUART(115200, TX_PIN, RX_PIN);
    // or
    // serial_control.beginDirectServo(shoulderPin,wristPin);

    // --- Build UI ---
    tabMain = ESPUI.addControl(Tab, "Main Control", "", ControlColor::None, Control::noParent);
    tabLogin = ESPUI.addControl(Tab, "Internet / Login", "", ControlColor::None, Control::noParent);
    tabAdvanced = ESPUI.addControl(Tab, "Advanced", "", ControlColor::None, Control::noParent);

    // MAIN tab - header
    ESPUI.label("2DOF Robot - Main", ControlColor::Turquoise, "Control the shoulder and wrist angles.", tabMain);

    // Shoulder controls: slider + number
    sliderShoulder = ESPUI.slider("Shoulder", sliderShoulderCallback, ControlColor::Peterriver, 0, -180, 180, tabMain);
    numberShoulder = ESPUI.number("Shoulder (deg)", numberShoulderCallback, ControlColor::Sunflower, 0, -180, 180, tabMain);

    // Wrist controls
    sliderWrist = ESPUI.slider("Wrist", sliderWristCallback, ControlColor::Wisteria, 0, -180, 180, tabMain);
    numberWrist = ESPUI.number("Wrist (deg)", numberWristCallback, ControlColor::Pumpkin, 0, -180, 180, tabMain);

    // Action buttons row
    btnHome = ESPUI.button("Home (0°)", btnHomeCallback, ControlColor::Nephritis, tabMain);
    btnStop = ESPUI.button("Stop", btnStopCallback, ControlColor::Alizarin, tabMain);

    // LOGIN tab
    ESPUI.label("WiFi / Login", ControlColor::Carrot, "Provide WiFi credentials (example fields)", tabLogin);
    inputSSID = ESPUI.text("SSID", ssidTextCallback, ControlColor::Wisteria, WIFI_SSID, tabLogin);
    inputPass = ESPUI.text("Password", passTextCallback, ControlColor::Alizarin, "********", tabLogin);
    ESPUI.setInputType(ESPUI.lastControl, "password");
    btnApplyWiFi = ESPUI.button("Apply Settings", btnApplyWiFiCallback, ControlColor::WetAsphalt, tabLogin);

    // ADVANCED tab - status, actuals, logs (similar idea to ui_complete.png)
    ESPUI.label("Status / Advanced", ControlColor::Emerald, "<b>Live Feedback & Logs</b>", tabAdvanced);
    labelSystemStatus = ESPUI.label("System", ControlColor::Wetasphalt, "OK", tabAdvanced);
    labelShoulderActual = ESPUI.label("Shoulder Actual", ControlColor::Wetasphalt, "0", tabAdvanced);
    labelWristActual = ESPUI.label("Wrist Actual", ControlColor::Wetasphalt, "0", tabAdvanced);
    textLog = ESPUI.textarea("Event Log", NULL, ControlColor::MidnightBlue, "Ready.", tabAdvanced);

    // Start web UI (page title)
    ESPUI.begin("2DOF ESPUI Controller");
}

unsigned long lastUIUpdate = 0;
void loop() {
    ESPUI.poll(); // ensure callbacks processed

    // Read feedback from serial_control (fill outputData), if available
    serial_control.readFeedback(); // implement in your SerialControl

    // Update actual labels every ~100ms (only if changed)
    static int lastActShoulder = 9999;
    static int lastActWrist = 9999;
    int actSh = serial_control.outputData.actualPosition1; // shoulder index 1
    int actWr = serial_control.outputData.actualPosition0; // wrist index 0

    if (actSh != lastActShoulder) {
        ESPUI.updateLabel(labelShoulderActual, String(actSh).c_str());
        lastActShoulder = actSh;
    }
    if (actWr != lastActWrist) {
        ESPUI.updateLabel(labelWristActual, String(actWr).c_str());
        lastActWrist = actWr;
    }

    // Example: update system status (you could reflect errors, battery, etc.)
    // ESPUI.updateLabel(labelSystemStatus, "OK");

    // Optionally append to log if SerialControl produced new messages
    // Example stub: if(serial_control.hasLogLine()) appendLog(serial_control.popLogLine());

    // Send outgoing position commands at your chosen interval (your SerialControl likely has sendIfChanged)
    static unsigned long lastSend = 0;
    if (millis() - lastSend > 50) {
        lastSend = millis();
        serial_control.sendIfChanged();
    }

    delay(5);
}
