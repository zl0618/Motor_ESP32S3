#include <HardwareSerial.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <Wire.h>

// GPIO Pin Definitions
#define LIDAR_RX_PIN 17
#define LIDAR_TX_PIN 18
#define LED_PIN 45

// MS200 Protocol Constants
#define MS200_HEADER 0x54
#define MS200_POINT_MAX 360
#define MS200_POINT_PER_PACK 20
#define RX_BUFFER_SIZE 2048

// Error handling constants
#define MAX_CRC_ERRORS 10
#define MAX_CONSECUTIVE_ERRORS 5
#define MIN_VALID_DISTANCE 10
#define MAX_VALID_DISTANCE 8000

// Add diagnostic constants
#define DIAGNOSTIC_MODE true
#define RAW_DATA_DUMP_SIZE 50

// LIDAR Serial
HardwareSerial lidarSerial(1);

// Ring Buffer for UART data
class RingBuffer {
private:
    uint8_t* buffer;
    int size;
    int head;
    int tail;
    int count;
    
public:
    RingBuffer(int bufferSize) {
        size = bufferSize;
        buffer = new uint8_t[size];
        head = 0;
        tail = 0;
        count = 0;
    }
    
    ~RingBuffer() {
        delete[] buffer;
    }
    
    bool put(uint8_t data) {
        if (count >= size) return false;
        buffer[head] = data;
        head = (head + 1) % size;
        count++;
        return true;
    }
    
    bool get(uint8_t* data) {
        if (count == 0) return false;
        *data = buffer[tail];
        tail = (tail + 1) % size;
        count--;
        return true;
    }
    
    int available() {
        return count;
    }
    
    void clear() {
        head = 0;
        tail = 0;
        count = 0;
    }
};

// MS200 Data Structures
struct Ms200Point {
    uint16_t distance;
    uint8_t intensity;
};

struct Ms200Package {
    uint8_t header;
    uint8_t count;
    uint16_t speed;
    uint16_t start_angle;
    uint16_t end_angle;
    uint16_t time_stamp;
    uint8_t crc8;
    Ms200Point points[MS200_POINT_PER_PACK];
};

struct Ms200Data {
    Ms200Point points[MS200_POINT_MAX];
    bool updated;
};

// Enhanced MS200 Decoder with proper protocol handling
class MS200Decoder {
private:
    RingBuffer* ringBuffer;
    uint8_t protocolBuffer[256];
    int protocolIndex;
    bool packageReady;
    Ms200Data lidarData;
    unsigned long lastPacketTime;
    int validPacketCount;
    
    // Error tracking
    uint32_t totalCrcErrors;
    uint32_t consecutiveErrors;
    uint32_t totalPackets;
    uint32_t invalidDistanceCount;
    
    // Diagnostic variables
    bool diagnosticMode;
    uint8_t lastRawData[RAW_DATA_DUMP_SIZE];
    int rawDataIndex;
    
    // Fixed CRC8 calculation based on observed data patterns
    uint8_t calculateCRC8_Standard(uint8_t* data, int length) {
        uint8_t crc = 0;
        for (int i = 0; i < length; i++) {
            crc ^= data[i];
            for (int j = 0; j < 8; j++) {
                if (crc & 0x80) {
                    crc = (crc << 1) ^ 0x31;
                } else {
                    crc <<= 1;
                }
            }
        }
        return crc;
    }
    
    // New CRC method based on packet analysis - this looks promising
    uint8_t calculateCRC8_MS200(uint8_t* data, int length) {
        uint8_t crc = 0;
        for (int i = 0; i < length; i++) {
            crc += data[i];
        }
        return (crc ^ 0xFF) & 0xFF;  // Invert and mask
    }
    
    // Another attempt - XOR with different approach
    uint8_t calculateCRC8_XOR(uint8_t* data, int length) {
        uint8_t crc = 0;
        for (int i = 0; i < length; i++) {
            crc ^= data[i];
        }
        return crc;
    }
    
    // Simple sum approach
    uint8_t calculateCRC8_Sum(uint8_t* data, int length) {
        uint16_t sum = 0;
        for (int i = 0; i < length; i++) {
            sum += data[i];
        }
        return (uint8_t)(sum & 0xFF);
    }
    
    // Alternative CRC8 calculations for testing
    uint8_t calculateCRC8_Alternative1(uint8_t* data, int length) {
        uint8_t crc = 0;
        for (int i = 0; i < length; i++) {
            crc ^= data[i];
            for (int j = 0; j < 8; j++) {
                if (crc & 0x80) {
                    crc = (crc << 1) ^ 0x07;  // Different polynomial
                } else {
                    crc <<= 1;
                }
            }
        }
        return crc;
    }
    
    uint8_t calculateCRC8_Alternative2(uint8_t* data, int length) {
        uint8_t crc = 0xFF;  // Different initial value
        for (int i = 0; i < length; i++) {
            crc ^= data[i];
            for (int j = 0; j < 8; j++) {
                if (crc & 0x80) {
                    crc = (crc << 1) ^ 0x31;
                } else {
                    crc <<= 1;
                }
            }
        }
        return crc;
    }
    
    uint8_t calculateCRC8_Simple(uint8_t* data, int length) {
        uint8_t crc = 0;
        for (int i = 0; i < length; i++) {
            crc += data[i];
        }
        return crc;
    }
    
    // Main CRC calculation function - try different methods
    uint8_t calculateCRC8(uint8_t* data, int length) {
        // Based on the packet analysis, let's try the XOR method first
        return calculateCRC8_XOR(data, length);
    }
    
    // Store raw data for diagnostics
    void storeRawData(uint8_t* data, int length) {
        if (!diagnosticMode) return;
        
        for (int i = 0; i < length && rawDataIndex < RAW_DATA_DUMP_SIZE; i++) {
            lastRawData[rawDataIndex++] = data[i];
        }
    }
    
    // Dump raw data for analysis
    void dumpRawData() {
        if (!diagnosticMode || rawDataIndex == 0) return;
        
        Serial.print("Raw Data Dump: ");
        for (int i = 0; i < rawDataIndex; i++) {
            Serial.printf("%02X ", lastRawData[i]);
        }
        Serial.println();
        rawDataIndex = 0;
    }
    
    // Handle CRC errors with recovery
    void handleCRCError(uint8_t calculated, uint8_t expected) {
        totalCrcErrors++;
        consecutiveErrors++;
        
        if (diagnosticMode && totalCrcErrors <= 15) { // Limit diagnostic output
            Serial.printf("CRC Error: %d != %d (Total: %d)\n", calculated, expected, totalCrcErrors);
            
            // Try all CRC methods to find the correct one
            int dataLen = (protocolBuffer[1] & 0x1F) * 3 + 10;
            uint8_t std = calculateCRC8_Standard(protocolBuffer, dataLen);
            uint8_t alt1 = calculateCRC8_Alternative1(protocolBuffer, dataLen);
            uint8_t alt2 = calculateCRC8_Alternative2(protocolBuffer, dataLen);
            uint8_t simple = calculateCRC8_Simple(protocolBuffer, dataLen);
            uint8_t xor_method = calculateCRC8_XOR(protocolBuffer, dataLen);
            uint8_t sum_method = calculateCRC8_Sum(protocolBuffer, dataLen);
            uint8_t ms200_method = calculateCRC8_MS200(protocolBuffer, dataLen);
            
            Serial.printf("CRC Methods - Std:%d Alt1:%d Alt2:%d Simple:%d XOR:%d Sum:%d MS200:%d Expected:%d\n", 
                         std, alt1, alt2, simple, xor_method, sum_method, ms200_method, expected);
            
            // Check if any method matches
            if (std == expected) Serial.println("*** MATCH: Standard method works! ***");
            if (alt1 == expected) Serial.println("*** MATCH: Alternative1 method works! ***");
            if (alt2 == expected) Serial.println("*** MATCH: Alternative2 method works! ***");
            if (simple == expected) Serial.println("*** MATCH: Simple method works! ***");
            if (xor_method == expected) Serial.println("*** MATCH: XOR method works! ***");
            if (sum_method == expected) Serial.println("*** MATCH: Sum method works! ***");
            if (ms200_method == expected) Serial.println("*** MATCH: MS200 method works! ***");
        }
        
        // Clear buffer if too many consecutive errors
        if (consecutiveErrors >= MAX_CONSECUTIVE_ERRORS) {
            Serial.println("Too many consecutive errors, clearing buffer");
            ringBuffer->clear();
            consecutiveErrors = 0;
            
            // Send command to restart sensor if needed
            restartSensor();
        }
    }
    
    // Check if distance reading is valid
    bool isDistanceValid(uint16_t distance) {
        return (distance >= MIN_VALID_DISTANCE && distance <= MAX_VALID_DISTANCE);
    }
    
    // Restart sensor communication
    void restartSensor() {
        Serial.println("Restarting sensor communication...");
        lidarSerial.flush();
        delay(100);
        
        // Clear any remaining data
        while (lidarSerial.available()) {
            lidarSerial.read();
        }
        
        // Try to send start command to sensor
        uint8_t startCmd[] = {0xA5, 0x60};  // Common LIDAR start command
        lidarSerial.write(startCmd, 2);
        delay(100);
        
        // Reset protocol state
        protocolIndex = 0;
        
        if (diagnosticMode) {
            Serial.println("Sensor restart completed");
        }
    }
    
    // Parse MS200 package following the reference protocol
    bool parsePackage(uint8_t* protocolBuf, Ms200Package* outPkg) {
        totalPackets++;
        
        // Store raw data for diagnostics
        storeRawData(protocolBuf, min(50, (protocolBuf[1] & 0x1F) * 3 + 11));
        
        uint8_t bufLen = (protocolBuf[1] & 0x1F) * 3 + 11;
        if (bufLen > 256 || bufLen < 11) {
            if (diagnosticMode) {
                Serial.printf("Invalid packet length: %d\n", bufLen);
            }
            return false;
        }
        
        uint8_t checkNum = protocolBuf[bufLen - 1];
        uint8_t crc8 = calculateCRC8(protocolBuf, bufLen - 1);
        
        // For testing: temporarily skip CRC check to see if data structure is correct
        bool crcValid = (crc8 == checkNum);
        static bool skipCrcReported = false;
        
        if (!crcValid) {
            handleCRCError(crc8, checkNum);
            
            // Temporarily accept packets to test data structure (remove this later)
            if (totalPackets <= 20 && !skipCrcReported) {
                Serial.println("*** TEMPORARILY SKIPPING CRC CHECK FOR TESTING ***");
                skipCrcReported = true;
                crcValid = true; // Force validation for testing
            } else {
                return false;
            }
        }
        
        // Reset consecutive error count on successful CRC
        if (crcValid && crc8 == checkNum) {
            consecutiveErrors = 0;
        }
        
        // Validate packet structure
        if (protocolBuf[0] != MS200_HEADER) {
            if (diagnosticMode) {
                Serial.printf("Invalid header: 0x%02X\n", protocolBuf[0]);
            }
            return false;
        }
        
        outPkg->header = protocolBuf[0];
        outPkg->count = protocolBuf[1] & 0x1F;
        
        // Validate count
        if (outPkg->count < 1 || outPkg->count > 20) {
            if (diagnosticMode) {
                Serial.printf("Invalid point count: %d\n", outPkg->count);
            }
            return false;
        }
        
        outPkg->speed = (protocolBuf[3] << 8) | protocolBuf[2];
        outPkg->start_angle = (protocolBuf[5] << 8) | protocolBuf[4];
        outPkg->end_angle = (protocolBuf[bufLen - 4] << 8) | protocolBuf[bufLen - 5];
        outPkg->time_stamp = (protocolBuf[bufLen - 2] << 8) | protocolBuf[bufLen - 3];
        outPkg->crc8 = protocolBuf[bufLen - 1];
        
        // Extract point data with validation
        for (int i = 0; i < outPkg->count && i < MS200_POINT_PER_PACK; i++) {
            outPkg->points[i].distance = (protocolBuf[3*i + 7] << 8) | protocolBuf[3*i + 6];
            outPkg->points[i].intensity = protocolBuf[3*i + 8];
            
            // Track invalid distances
            if (!isDistanceValid(outPkg->points[i].distance) && outPkg->points[i].distance != 0) {
                invalidDistanceCount++;
            }
        }
        
        if (diagnosticMode && validPacketCount < 10) {
            Serial.printf("Packet parsed - Count: %d, Start: %d°, End: %d°, Speed: %d RPM\n", 
                         outPkg->count, outPkg->start_angle/100, outPkg->end_angle/100, outPkg->speed);
            
            // Show first few distance readings
            Serial.print("Distances: ");
            for (int i = 0; i < min((int)outPkg->count, 5); i++) {
                Serial.printf("%dmm ", outPkg->points[i].distance);
            }
            Serial.println();
        }
        
        return true;
    }
    
    // Update data with angle interpolation
    void updateData(Ms200Package* pkg) {
        uint16_t stepAngle = 0;
        uint16_t angle = 0;
        
        if (pkg->end_angle > pkg->start_angle) {
            // Normal condition
            stepAngle = (pkg->end_angle - pkg->start_angle) / (pkg->count - 1);
        } else {
            // Special case: end angle < start angle (crossing 0 degrees)
            stepAngle = (36000 + pkg->end_angle - pkg->start_angle) / (pkg->count - 1);
        }
        
        for (int i = 0; i < pkg->count; i++) {
            angle = ((pkg->start_angle + i * stepAngle) / 100) % MS200_POINT_MAX;
            lidarData.points[angle].distance = pkg->points[i].distance;
            lidarData.points[angle].intensity = pkg->points[i].intensity;
        }
        
        lidarData.updated = true;
        validPacketCount++;
        lastPacketTime = millis();
    }
    
public:
    MS200Decoder() {
        ringBuffer = new RingBuffer(RX_BUFFER_SIZE);
        protocolIndex = 0;
        packageReady = false;
        lastPacketTime = millis();
        validPacketCount = 0;
        totalCrcErrors = 0;
        consecutiveErrors = 0;
        totalPackets = 0;
        invalidDistanceCount = 0;
        diagnosticMode = DIAGNOSTIC_MODE;
        rawDataIndex = 0;
        memset(&lidarData, 0, sizeof(lidarData));
        memset(lastRawData, 0, sizeof(lastRawData));
    }
    
    ~MS200Decoder() {
        delete ringBuffer;
    }
    
    // Add data to ring buffer
    void addData(uint8_t* data, int length) {
        for (int i = 0; i < length; i++) {
            ringBuffer->put(data[i]);
        }
    }
    
    // Process data from ring buffer
    bool processData() {
        uint8_t byte;
        bool newPackage = false;
        static unsigned long lastDumpTime = 0;
        
        while (ringBuffer->get(&byte)) {
            if (protocolIndex == 0) {
                // Looking for header
                if (byte == MS200_HEADER) {
                    protocolBuffer[protocolIndex++] = byte;
                    if (diagnosticMode && millis() - lastDumpTime > 5000) {
                        Serial.printf("Found header at buffer position\n");
                        lastDumpTime = millis();
                    }
                }
            } else if (protocolIndex == 1) {
                // Get count and validate
                protocolBuffer[protocolIndex++] = byte;
                uint8_t count = byte & 0x1F;
                if (count < 1 || count > 20) {
                    if (diagnosticMode) {
                        Serial.printf("Invalid count in packet: %d\n", count);
                    }
                    protocolIndex = 0; // Reset if invalid count
                }
            } else {
                protocolBuffer[protocolIndex++] = byte;
                
                // Check if we have a complete packet
                if (protocolIndex >= 11) {
                    uint8_t expectedLen = (protocolBuffer[1] & 0x1F) * 3 + 11;
                    if (protocolIndex >= expectedLen) {
                        // Try to parse the packet
                        Ms200Package pkg;
                        if (parsePackage(protocolBuffer, &pkg)) {
                            updateData(&pkg);
                            newPackage = true;
                        }
                        protocolIndex = 0; // Reset for next packet
                    }
                }
                
                // Prevent buffer overflow
                if (protocolIndex >= 256) {
                    if (diagnosticMode) {
                        Serial.println("Protocol buffer overflow, resetting");
                        dumpRawData();
                    }
                    protocolIndex = 0;
                }
            }
        }
        
        return newPackage;
    }
    
    // Get distance at specific angle with validation
    uint16_t getDistance(uint16_t angle) {
        if (angle < MS200_POINT_MAX) {
            uint16_t distance = lidarData.points[angle].distance;
            return isDistanceValid(distance) ? distance : 0;
        }
        return 0;
    }
    
    // Get intensity at specific angle
    uint8_t getIntensity(uint16_t angle) {
        if (angle < MS200_POINT_MAX) {
            return lidarData.points[angle].intensity;
        }
        return 0;
    }
    
    // Get all scan data for web interface
    int getAllPoints(float* points, int maxPoints) {
        int count = 0;
        for (int i = 0; i < MS200_POINT_MAX && count < maxPoints; i++) {
            if (isDistanceValid(lidarData.points[i].distance)) {
                float angleRad = i * PI / 180.0;
                points[count * 5 + 0] = lidarData.points[i].distance * sin(angleRad); // x
                points[count * 5 + 1] = lidarData.points[i].distance * cos(angleRad); // y
                points[count * 5 + 2] = i; // angle
                points[count * 5 + 3] = lidarData.points[i].distance; // distance
                points[count * 5 + 4] = lidarData.points[i].intensity; // intensity
                count++;
            }
        }
        return count;
    }
    
    // Get statistics
    unsigned long getLastPacketTime() { return lastPacketTime; }
    int getValidPacketCount() { return validPacketCount; }
    int getBufferUsage() { return ringBuffer->available(); }
    uint32_t getTotalCrcErrors() { return totalCrcErrors; }
    uint32_t getTotalPackets() { return totalPackets; }
    uint32_t getInvalidDistanceCount() { return invalidDistanceCount; }
    float getErrorRate() { 
        return totalPackets > 0 ? (float)totalCrcErrors / totalPackets * 100.0 : 0.0; 
    }
    
    // Reset error counters
    void resetErrorCounters() {
        totalCrcErrors = 0;
        consecutiveErrors = 0;
        totalPackets = 0;
        invalidDistanceCount = 0;
    }
    
    // Diagnostic functions
    void enableDiagnostics(bool enable) { diagnosticMode = enable; }
    void printDiagnostics() {
        Serial.printf("=== LIDAR Diagnostics ===\n");
        Serial.printf("Total Packets: %d\n", totalPackets);
        Serial.printf("Valid Packets: %d\n", validPacketCount);
        Serial.printf("CRC Errors: %d (%.1f%%)\n", totalCrcErrors, getErrorRate());
        Serial.printf("Invalid Distances: %d\n", invalidDistanceCount);
        Serial.printf("Buffer Usage: %d\n", ringBuffer->available());
        Serial.printf("Last Packet Age: %lu ms\n", millis() - lastPacketTime);
        Serial.printf("========================\n");
    }
};

// Global variables
MS200Decoder* decoder;
WebServer server(80);
unsigned long lastStatusUpdate = 0;
unsigned long frameCount = 0;
TaskHandle_t lidarTaskHandle = NULL;

// LIDAR processing task
void lidarTask(void* parameter) {
    uint8_t buffer[512];
    Serial.println("LIDAR task started on core: " + String(xPortGetCoreID()));
    
    while (true) {
        // Read available data
        int available = lidarSerial.available();
        if (available > 0) {
            int bytesToRead = min(available, 512);
            int bytesRead = lidarSerial.readBytes(buffer, bytesToRead);
            
            if (bytesRead > 0) {
                decoder->addData(buffer, bytesRead);
                
                // Process the data
                if (decoder->processData()) {
                    frameCount++;
                }
            }
        }
        
        // Small delay to prevent watchdog issues
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void setup() {
    Serial.begin(115200);
    
    // Initialize decoder
    decoder = new MS200Decoder();
    
    // Initialize LIDAR serial with proper configuration
    lidarSerial.begin(230400, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);
    lidarSerial.setRxBufferSize(2048); // Increase RX buffer
    
    // Initialize LED
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);
    
    // Setup WiFi AP
    WiFi.softAP("ESP32_LIDAR_SLAM", "12345678");
    Serial.println("WiFi AP started");
    Serial.print("IP address: ");
    Serial.println(WiFi.softAPIP());
    
    // Setup web server
    server.on("/", handleRoot);
    server.on("/data", handleData);
    server.begin();
    
    // Create LIDAR processing task on core 0
    xTaskCreatePinnedToCore(
        lidarTask,           // Task function
        "LidarTask",         // Task name
        4096,               // Stack size
        NULL,               // Parameter
        2,                  // Priority
        &lidarTaskHandle,   // Task handle
        0                   // Core 0
    );
    
    Serial.println("ESP32 LIDAR System initialized");
    
    // Test specific angles like in reference code
    delay(2000); // Wait for LIDAR to stabilize
}

void loop() {
    server.handleClient();
    
    // Status update every second
    if (millis() - lastStatusUpdate > 1000) {
        // Test specific angles like in reference code
        uint16_t testAngles[] = {0, 90, 180, 270};
        uint16_t distances[4];
        
        for (int i = 0; i < 4; i++) {
            distances[i] = decoder->getDistance(testAngles[i]);
        }
        
        Serial.printf("Distances - 0°:%d, 90°:%d, 180°:%d, 270°:%d | Frames:%lu | Buffer:%d | Free:%d\n", 
                     distances[0], distances[1], distances[2], distances[3], 
                     frameCount, decoder->getBufferUsage(), ESP.getFreeHeap());
        
        // Print error statistics every 10 seconds
        static int errorReportCounter = 0;
        if (++errorReportCounter >= 10) {
            Serial.printf("Error Stats - CRC Errors: %d/%d (%.1f%%) | Invalid Distances: %d\n",
                         decoder->getTotalCrcErrors(), decoder->getTotalPackets(), 
                         decoder->getErrorRate(), decoder->getInvalidDistanceCount());
            
            // Print full diagnostics every 30 seconds if no valid packets
            static int diagnosticCounter = 0;
            if (++diagnosticCounter >= 3 && frameCount == 0) {
                decoder->printDiagnostics();
                diagnosticCounter = 0;
            }
            
            errorReportCounter = 0;
        }
        
        // Blink LED to show activity
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        lastStatusUpdate = millis();
    }
    
    delay(10);
}

void handleRoot() {
    String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>ESP32 LIDAR SLAM - Fixed</title>
    <meta charset="utf-8">
    <style>
        body { font-family: Arial; background: #1a1a1a; color: white; margin: 0; padding: 20px; }
        canvas { border: 1px solid #444; background: black; }
        .stats { font-family: monospace; background: #333; padding: 10px; margin: 10px 0; }
        .angles { display: flex; gap: 20px; margin: 10px 0; }
        .angle-box { background: #333; padding: 10px; border-radius: 5px; text-align: center; }
    </style>
</head>
<body>
    <h1>ESP32 LIDAR SLAM - Enhanced Protocol</h1>
    <canvas id="lidarCanvas" width="800" height="600"></canvas>
    <div class="stats" id="stats">Connecting...</div>
    <div class="angles" id="angles">
        <div class="angle-box">0°: <span id="angle0">--</span>mm</div>
        <div class="angle-box">90°: <span id="angle90">--</span>mm</div>
        <div class="angle-box">180°: <span id="angle180">--</span>mm</div>
        <div class="angle-box">270°: <span id="angle270">--</span>mm</div>
    </div>
    
    <script>
        const canvas = document.getElementById('lidarCanvas');
        const ctx = canvas.getContext('2d');
        const centerX = canvas.width / 2;
        const centerY = canvas.height / 2;
        const scale = 0.15;
        
        function updateDisplay() {
            fetch('/data')
                .then(response => response.json())
                .then(data => {
                    // Clear canvas
                    ctx.fillStyle = 'black';
                    ctx.fillRect(0, 0, canvas.width, canvas.height);
                    
                    // Draw grid circles
                    ctx.strokeStyle = '#333';
                    ctx.lineWidth = 1;
                    for (let r = 100; r <= 3000; r += 500) {
                        ctx.beginPath();
                        ctx.arc(centerX, centerY, r * scale, 0, 2 * Math.PI);
                        ctx.stroke();
                    }
                    
                    // Draw angle lines
                    for (let angle = 0; angle < 360; angle += 30) {
                        ctx.beginPath();
                        ctx.moveTo(centerX, centerY);
                        let x = centerX + 3000 * scale * Math.sin(angle * Math.PI / 180);
                        let y = centerY - 3000 * scale * Math.cos(angle * Math.PI / 180);
                        ctx.lineTo(x, y);
                        ctx.stroke();
                    }
                    
                    // Draw center point
                    ctx.fillStyle = 'white';
                    ctx.fillRect(centerX - 2, centerY - 2, 4, 4);
                    
                    // Draw LIDAR points
                    data.points.forEach(point => {
                        const x = centerX + point.x * scale;
                        const y = centerY - point.y * scale;
                        
                        // Color based on distance
                        if (point.distance < 300) ctx.fillStyle = 'red';
                        else if (point.distance < 500) ctx.fillStyle = 'lime';
                        else if (point.distance < 1500) ctx.fillStyle = 'cyan';
                        else ctx.fillStyle = 'yellow';
                        
                        ctx.fillRect(x - 1, y - 1, 3, 3);
                    });
                    
                    // Update stats
                    document.getElementById('stats').innerHTML = 
                        `Points: ${data.pointCount} | Frames: ${data.frameCount} | Buffer: ${data.bufferUsage} | Free Heap: ${data.freeHeap} | CRC Errors: ${data.crcErrors}/${data.totalPackets} (${data.errorRate}%)`;
                    
                    // Update angle displays
                    document.getElementById('angle0').textContent = data.angles[0];
                    document.getElementById('angle90').textContent = data.angles[90];
                    document.getElementById('angle180').textContent = data.angles[180];
                    document.getElementById('angle270').textContent = data.angles[270];
                })
                .catch(error => console.error('Error:', error));
        }
        
        // Update display every 100ms
        setInterval(updateDisplay, 100);
    </script>
</body>
</html>
)rawliteral";
    server.send(200, "text/html", html);
}

void handleData() {
    DynamicJsonDocument doc(8192);
    JsonArray pointsArray = doc.createNestedArray("points");
    
    // Get all points
    float points[1000]; // 200 points * 5 values each
    int pointCount = decoder->getAllPoints(points, 200);
    
    for (int i = 0; i < pointCount; i++) {
        JsonObject point = pointsArray.createNestedObject();
        point["x"] = points[i * 5 + 0];
        point["y"] = points[i * 5 + 1];
        point["angle"] = points[i * 5 + 2];
        point["distance"] = points[i * 5 + 3];
        point["intensity"] = points[i * 5 + 4];
    }
    
    // Add specific angle data like reference code
    JsonObject angles = doc.createNestedObject("angles");
    angles["0"] = decoder->getDistance(0);
    angles["90"] = decoder->getDistance(90);
    angles["180"] = decoder->getDistance(180);
    angles["270"] = decoder->getDistance(270);
    
    doc["pointCount"] = pointCount;
    doc["frameCount"] = frameCount;
    doc["freeHeap"] = ESP.getFreeHeap();
    doc["bufferUsage"] = decoder->getBufferUsage();
    doc["lastPacketAge"] = millis() - decoder->getLastPacketTime();
    doc["crcErrors"] = decoder->getTotalCrcErrors();
    doc["totalPackets"] = decoder->getTotalPackets();
    doc["errorRate"] = decoder->getErrorRate();
    doc["invalidDistances"] = decoder->getInvalidDistanceCount();
    
    String response;
    serializeJson(doc, response);
    server.send(200, "application/json", response);
}
