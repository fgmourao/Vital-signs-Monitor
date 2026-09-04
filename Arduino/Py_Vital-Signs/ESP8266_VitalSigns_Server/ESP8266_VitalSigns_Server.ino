/**
 * @file    ESP8266_VitalSigns_Server.ino
 * PROJECT: Vital-signs monitor
 * @version 3.0 (Multi-client)
 * @author  Flávio Mourão — Mar, 2026
 *
 * ═══════════════════════════════════════════════════════════════════════════════
 * MODULE RESPONSIBILITY
 * ═══════════════════════════════════════════════════════════════════════════════
 * This sketch runs on the ESP8266 co-processor of the RobotDyn Mega+WiFi board.
 * It performs three tasks:
 *
 *   1. Wi-Fi Access Point — creates a local network (no internet required).
 *   2. HTTP Server (port 80) — serves the embedded HTML/JS dashboard to browsers.
 *   3. WebSocket Server (port 81) — forwards JSON packets received from the
 *      ATmega2560 (via Serial) to connected browser clients.
 *
 * ═══════════════════════════════════════════════════════════════════════════════
 * CHANGES vs v2.0 — MULTI-CLIENT SUPPORT (3–5 simultaneous browsers)
 * ═══════════════════════════════════════════════════════════════════════════════
 *
 * [C1] Non-blocking Serial read (replaces readStringUntil)
 *   v2.0 used Serial.readStringUntil('\n'), which blocks the entire loop()
 *   until a full line arrives or the timeout expires. With multiple WebSocket
 *   clients generating keep-alive traffic, this caused frame accumulation and
 *   eventually client disconnections.
 *
 *   v3.0 reads one byte at a time per loop() iteration, accumulating into
 *   rxBuffer until '\n' is detected. webSocket.loop() is never blocked.
 *   Maximum latency added per loop(): one Serial.read() call (~1 µs).
 *
 * [C2] Tiered dispatch: primary client at 100 Hz, secondary clients at 25 Hz
 *   Broadcasting the full 100 Hz stream to 5 clients simultaneously would
 *   require the ESP8266 to transmit 5 × 55 bytes × 100 Hz = 27.5 kB/s on
 *   the Wi-Fi channel. While within the hardware limit (~1 MB/s), the
 *   sequential sendTXT() calls per loop() iteration would block webSocket.loop()
 *   for ~2.4 ms at 115200 baud, degrading all clients.
 *
 *   The tiered approach:
 *     Primary client (first to connect): receives every packet → 100 Hz
 *     Secondary clients (subsequent):   receive 1 in 4 packets → 25 Hz
 *
 *   25 Hz is sufficient for numeric readout updates and smooth waveform
 *   display on secondary monitors (human visual persistence ~40 ms).
 *   The primary client (e.g., the surgeon's workstation) retains full
 *   100 Hz resolution for waveform detail.
 *
 * [C3] WebSocket event handler
 *   webSocket.onEvent() now tracks which client is the primary and clears
 *   the slot when the primary disconnects, allowing the next client to
 *   take the primary role automatically.
 *
 * ═══════════════════════════════════════════════════════════════════════════════
 * HARDWARE CONTEXT — RobotDyn Mega+WiFi
 * ═══════════════════════════════════════════════════════════════════════════════
 * On the RobotDyn board, the ESP8266 hardware UART (Serial, GPIO1/GPIO3) is
 * bridged via PCB traces to ATmega2560 Serial3 (pins 14/15). Data flows:
 *
 *   ATmega2560 Serial3 → [PCB bridge] → ESP8266 Serial.available()
 *
 * Baud rate: 115200. Must match vitalsigns_init() in surgery_monitor_web_server.ino.
 *
 * Flash this sketch using the Arduino IDE with board set to "Generic ESP8266".
 * The DIP switch bank on the RobotDyn board must be configured for ESP8266
 * programming mode during upload, then switched back for normal Mega operation.
 *
 * ═══════════════════════════════════════════════════════════════════════════════
 * NETWORK ARCHITECTURE
 * ═══════════════════════════════════════════════════════════════════════════════
 *
 *  [ATmega2560] --Serial3/115200--> [ESP8266] --Wi-Fi AP--> [Browsers]
 *
 *  Access Point:  SSID = AP_SSID (default: "Vital_Signs_Web_Monitor")
 *  Password:      AP_PASSWORD (empty = open network)
 *  AP IP address: 192.168.4.1 (ESP8266 SoftAP default)
 *
 *  Browser access: http://192.168.4.1
 *  WebSocket:      ws://192.168.4.1:81
 *
 *  Client capacity: up to WEBSOCKETS_SERVER_CLIENT_MAX (default 5 in library).
 *    - Client 0 (primary):   100 Hz full stream
 *    - Clients 1–4 (secondary): 25 Hz reduced stream
 *
 * ═══════════════════════════════════════════════════════════════════════════════
 * BANDWIDTH BUDGET (worst case: 5 clients)
 * ═══════════════════════════════════════════════════════════════════════════════
 *  Primary (1×):    55 B × 100 Hz = 5.5 kB/s
 *  Secondary (4×):  55 B × 25 Hz × 4 = 22 kB/s
 *  Total:           ~27.5 kB/s << ESP8266 Wi-Fi ~1 MB/s capacity
 *
 * ═══════════════════════════════════════════════════════════════════════════════
 * KNOWN LIMITATIONS
 * ═══════════════════════════════════════════════════════════════════════════════
 * [L1] Primary client assignment is first-connect order
 *   The first browser to open a WebSocket connection becomes the primary
 *   (100 Hz) client. If the surgeon's workstation connects second, it
 *   receives 25 Hz. Workaround: reload the primary browser first, or
 *   implement a URL parameter to request primary status (e.g. /?primary=1).
 *
 * [L2] Open Wi-Fi network
 *   AP_PASSWORD is empty by default — open network. Acceptable in a
 *   controlled lab. Set WPA2 password for environments with data governance
 *   requirements.
 *
 * [L3] No data persistence
 *   The ESP8266 holds no historical data. A browser connecting mid-session
 *   starts with an empty waveform buffer and sees no past data.
 *
 * ═══════════════════════════════════════════════════════════════════════════════
 * REQUIRED LIBRARIES
 * ═══════════════════════════════════════════════════════════════════════════════
 *   ESP8266WiFi       — bundled with ESP8266 Arduino core
 *   ESP8266WebServer  — bundled with ESP8266 Arduino core
 *   WebSocketsServer  — arduinoWebSockets by Markus Sattler (Library Manager)
 */

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <WebSocketsServer.h>


// ── Network configuration ────────────────────────────────────────────────────

const char* AP_SSID     = "Vital_Signs_Web_Monitor";
const char* AP_PASSWORD = "";   // Empty = open network


// ── Multi-client configuration ───────────────────────────────────────────────

// Maximum clients the WebSocketsServer library supports simultaneously.
// Default in the library: 5. Increase by editing WEBSOCKETS_SERVER_CLIENT_MAX
// in WebSockets.h, but each slot consumes ~300 B of heap on the ESP8266.
static const uint8_t MAX_CLIENTS = 5;

// Rate divisor for secondary clients.
// SECONDARY_DIVISOR = 4 → secondary clients receive 1 in 4 packets = 25 Hz.
// Increase to 8 for 12.5 Hz if more than 4 secondary clients are expected.
static const uint8_t SECONDARY_DIVISOR = 4;

// Sentinel value: no primary client assigned yet.
static const uint8_t NO_PRIMARY = 255;


// ── Runtime state ────────────────────────────────────────────────────────────

// ID of the primary (100 Hz) WebSocket client.
// Set to the first client that connects; reset when that client disconnects.
static uint8_t primaryClientId = NO_PRIMARY;

// Packet counter used to throttle secondary clients.
// Incremented on every valid packet received from the ATmega2560.
static uint8_t packetCount = 0;

// Non-blocking Serial receive buffer.
// Accumulates bytes until '\n' is received, then dispatches the complete packet.
static String rxBuffer = "";


// ── Server instances ─────────────────────────────────────────────────────────

ESP8266WebServer  server(80);
WebSocketsServer  webSocket = WebSocketsServer(81);


// ── Embedded dashboard HTML/JS ───────────────────────────────────────────────
// Stored in PROGMEM (flash) to preserve heap RAM.
// Identical to v2.0 — the multi-client logic is server-side only.
// Secondary clients display at 25 Hz; all visual behaviour is unchanged.
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Vital-Signs - Web Monitor</title>
    <style>
        * { box-sizing: border-box; margin: 0; padding: 0; }
        body { background-color: #000; color: #fff; font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; overflow: hidden; height: 100vh; }
        #header { background: #111; padding: 8px 20px; font-size: 18px; font-weight: bold; border-bottom: 2px solid #222; color: #666; display: flex; align-items: center; gap: 10px; height: 42px; }
        #status-dot { width: 10px; height: 10px; border-radius: 50%; background: #333; flex-shrink: 0; }
        #status-dot.connected { background: #0f0; }
        #client-role { font-size: 12px; color: #444; margin-left: auto; }
        .grid-container { display: flex; height: calc(100vh - 42px); }
        .sidebar { width: 260px; min-width: 200px; background: #0a0a0a; padding: 16px; display: flex; flex-direction: column; gap: 14px; border-right: 2px solid #1a1a1a; overflow: hidden; }
        .box { background: #111; padding: 12px 14px; border-radius: 8px; border-left: 5px solid; flex: 1; }
        
        .box.hr   { border-color: #b30000; color: #ccc; }
        .box.spo2 { border-color: #cc6600; color: #ccc; }
        .box.rr   { border-color: #cca300; color: #ccc; }
        .box.temp { border-color: #800080; color: #ccc; }

        .label { font-size: 11px; text-transform: uppercase; letter-spacing: 1.5px; color: #666; margin-bottom: 4px; }
        .value { font-size: 52px; font-weight: bold; text-align: right; line-height: 1; }
        .unit { font-size: 16px; color: #444; }
        .main-content { flex-grow: 1; padding: 14px; display: flex; flex-direction: column; gap: 14px; overflow: hidden; }
        .chart-container { flex: 1; background: #0d0d0d; border-radius: 8px; border: 1px solid #1a1a1a; position: relative; padding: 8px; overflow: hidden; }
        .chart-title { position: absolute; top: 8px; left: 12px; font-size: 11px; color: #444; font-weight: bold; letter-spacing: 1px; text-transform: uppercase; pointer-events: none; }
        canvas { display: block; width: 100%; height: 100%; }
    </style>
</head>
<body>
    <div id="header">
        <div id="status-dot"></div>
        VITAL-SIGNS - WEB MONITOR
        <span id="client-role"></span>
    </div>
    <div class="grid-container">
        <div class="sidebar">
            <div class="box hr">
                <div class="label">Heart Rate</div>
                <div class="value" id="val_hr">-- <span class="unit">bpm</span></div>
            </div>
            <div class="box spo2">
                <div class="label">SpO2</div>
                <div class="value" id="val_spo2">-- <span class="unit">%</span></div>
            </div>
            <div class="box rr">
                <div class="label">Resp. Rate</div>
                <div class="value" id="val_rr">-- <span class="unit">rpm</span></div>
            </div>
            <div class="box temp">
                <div class="label">Temperature</div>
                <div class="value" id="val_temp">--.- <span class="unit">°C</span></div>
            </div>
        </div>
        <div class="main-content">
            <div class="chart-container">
                <div class="chart-title" style="color:#b30000;">PPG — IR Optical (MAX30102)</div>
                <canvas id="canvas_ir"></canvas>
            </div>
            <div class="chart-container">
                <div class="chart-title" style="color:#cca300;">RESP — Piezo Belt</div>
                <canvas id="canvas_piezo"></canvas>
            </div>
        </div>
    </div>

    <script>
        const maxPoints = 300;
        let bufIR    = new Array(maxPoints).fill(0);
        let bufPiezo = new Array(maxPoints).fill(0);

        const canIR    = document.getElementById('canvas_ir');
        const ctxIR    = canIR.getContext('2d');
        const canPiezo = document.getElementById('canvas_piezo');
        const ctxPiezo = canPiezo.getContext('2d');

        function resizeCanvases() {
            canIR.width    = canIR.parentElement.clientWidth  - 16;
            canIR.height   = canIR.parentElement.clientHeight - 16;
            canPiezo.width  = canPiezo.parentElement.clientWidth  - 16;
            canPiezo.height = canPiezo.parentElement.clientHeight - 16;
        }
        window.addEventListener('resize', resizeCanvases);
        resizeCanvases();

        var gateway   = 'ws://' + window.location.hostname + ':81/';
        var websocket;
        var statusDot  = document.getElementById('status-dot');
        var roleLabel  = document.getElementById('client-role');

        function initWebSocket() {
            websocket = new WebSocket(gateway);

            websocket.onopen = function() {
                statusDot.classList.add('connected');
            };

            websocket.onclose = function() {
                statusDot.classList.remove('connected');
                roleLabel.textContent = '';
                // Automatic reconnection after 3 s (handles board resets mid-session).
                setTimeout(initWebSocket, 3000);
            };

            websocket.onmessage = onMessage;
        }

        function onMessage(event) {
            // The server prefixes the payload with a role tag before the JSON:
            //   "P:" → this client is primary   (100 Hz)
            //   "S:" → this client is secondary  (25 Hz)
            // The tag is stripped before JSON parsing so the dashboard logic
            // does not need to know about the server-side dispatch strategy.
            var raw = event.data;
            var role = raw.substring(0, 2);
            var json = raw.substring(2);

            if (role === 'P:') {
                roleLabel.textContent = 'PRIMARY — 100 Hz';
                roleLabel.style.color = '#0f0';
            } else if (role === 'S:') {
                roleLabel.textContent = 'SECONDARY — 25 Hz';
                roleLabel.style.color = '#666';
            } else {
                // Fallback: no role prefix (should not occur in normal operation)
                json = raw;
            }

            try {
                var obj = JSON.parse(json);

                document.getElementById('val_hr').innerHTML =
                    (obj.hr > 0 ? obj.hr : '--') + ' <span class="unit">bpm</span>';
                document.getElementById('val_spo2').innerHTML =
                    (obj.sp > 0 ? obj.sp : '--') + ' <span class="unit">%</span>';
                document.getElementById('val_rr').innerHTML =
                    (obj.rr > 0 ? obj.rr : '--') + ' <span class="unit">rpm</span>';
                document.getElementById('val_temp').innerHTML =
                    (obj.t > 0 ? obj.t.toFixed(1) : '--.-') + ' <span class="unit">°C</span>';

                bufIR.push(obj.ir);    bufIR.shift();
                bufPiezo.push(obj.pz); bufPiezo.shift();

                requestAnimationFrame(drawCharts);

            } catch (e) {
                console.warn('JSON parse error:', json, e);
            }
        }

        function drawSingleChart(ctx, canvas, buffer, color, minRange) {
            ctx.clearRect(0, 0, canvas.width, canvas.height);
            let maxVal = Math.max(...buffer);
            let minVal = Math.min(...buffer);
            if ((maxVal - minVal) < minRange) {
                let centre = (maxVal + minVal) / 2;
                maxVal = centre + minRange / 2;
                minVal = centre - minRange / 2;
            }
            const range = maxVal - minVal;
            ctx.beginPath();
            ctx.strokeStyle = color;
            ctx.lineWidth   = 2;
            ctx.lineJoin    = 'round';
            const stepX = canvas.width / (maxPoints - 1);
            for (let i = 0; i < maxPoints; i++) {
                const x = i * stepX;
                const y = canvas.height - ((buffer[i] - minVal) / range) * canvas.height;
                if (i === 0) ctx.moveTo(x, y);
                else         ctx.lineTo(x, y);
            }
            ctx.stroke();
        }

        function drawCharts() {
            drawSingleChart(ctxIR,    canIR,    bufIR,    '#b30000', 100); // Vermelho escuro
            drawSingleChart(ctxPiezo, canPiezo, bufPiezo, '#cca300', 20);  // Amarelo escuro
            // ────────────────────────────────────────────────────
        }

        initWebSocket();
    </script>
</body>
</html>
)rawliteral";

// ════════════════════════════════════════════════════════════════════════════
//  webSocketEvent()
//  Callback registered with webSocket.onEvent(). Called by the WebSocket
//  library for every client state change: connect, disconnect, and data.
//
//  Responsibilities:
//    - Assign the primary client slot on first connection.
//    - Clear the primary slot when the primary client disconnects, allowing
//      the next connection to take over as primary automatically.
// ════════════════════════════════════════════════════════════════════════════

void webSocketEvent(uint8_t clientId, WStype_t type,
                    uint8_t* payload, size_t length)
{
    switch (type) {

        case WStype_CONNECTED:
            // Assign primary role to the first client that connects.
            // Subsequent clients are automatically secondary.
            if (primaryClientId == NO_PRIMARY) {
                primaryClientId = clientId;
            }
            break;

        case WStype_DISCONNECTED:
            // If the primary disconnects, clear the slot.
            // The next packet dispatch will skip this ID (not connected),
            // and the next client to connect will become primary.
            if (clientId == primaryClientId) {
                primaryClientId = NO_PRIMARY;
            }
            break;

        case WStype_TEXT:
        case WStype_BIN:
            // Browser-to-server messages are not expected in this application.
            // Ignore silently.
            break;

        default:
            break;
    }
}


// ════════════════════════════════════════════════════════════════════════════
//  dispatchPacket()
//  Sends the current JSON payload to all connected WebSocket clients using
//  the tiered rate strategy:
//
//    Primary client   → every packet           (100 Hz)
//    Secondary clients → every SECONDARY_DIVISOR-th packet (25 Hz at /4)
//
//  A role prefix is prepended to the payload so the browser can identify
//  its own dispatch tier and display it in the UI:
//    "P:<json>"  — sent to the primary client
//    "S:<json>"  — sent to secondary clients
//
//  The prefix adds 2 bytes per packet — negligible overhead.
// ════════════════════════════════════════════════════════════════════════════

static void dispatchPacket(const String& json)
{
    packetCount++;

    // Primary client — every packet
    if (primaryClientId != NO_PRIMARY) {
        String msg = "P:" + json;
        webSocket.sendTXT(primaryClientId, msg);
    }

    // Secondary clients — throttled
    // Only send on packets that are multiples of SECONDARY_DIVISOR.
    if (packetCount % SECONDARY_DIVISOR == 0) {
        String msg = "S:" + json;
        for (uint8_t i = 0; i < MAX_CLIENTS; i++) {
            if (i != primaryClientId && webSocket.remoteIP(i).toString() != "0.0.0.0") {
                webSocket.sendTXT(i, msg);
            }
        }
    }
}


// ═══════════════════════════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════════════════════════

void setup()
{
    Serial.begin(115200);
    // No Serial.setTimeout() needed — v3.0 uses non-blocking byte-by-byte read.

    // ── Wi-Fi Access Point ────────────────────────────────────────────────────
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASSWORD);

    // ── HTTP server ───────────────────────────────────────────────────────────
    server.on("/", []() {
        server.send_P(200, "text/html", index_html);
    });
    server.begin();

    // ── WebSocket server ──────────────────────────────────────────────────────
    webSocket.begin();
    webSocket.onEvent(webSocketEvent);   // Register state-change callback
}


// ═══════════════════════════════════════════════════════════════════════════════
//  MAIN LOOP
// ═══════════════════════════════════════════════════════════════════════════════

void loop()
{
    // WebSocket and HTTP service calls — must execute every iteration.
    // Any blocking code between these calls degrades client responsiveness.
    webSocket.loop();
    server.handleClient();

    // ── Non-blocking Serial receive ───────────────────────────────────────────
    // Reads one byte per loop() call and appends to rxBuffer.
    // When '\n' is received, the complete JSON line is ready for dispatch.
    //
    // Why non-blocking matters with multiple clients:
    //   The WebSocket library must call webSocket.loop() frequently to process
    //   keep-alive (ping/pong) frames from each connected browser. With 5 clients
    //   each sending a ping every ~15 s, the library needs to respond within the
    //   ping timeout (~30 s). As long as webSocket.loop() is called at least
    //   once per second, this is satisfied. The non-blocking read guarantees
    //   webSocket.loop() runs every loop() iteration regardless of Serial state.
    while (Serial.available()) {
        char c = (char)Serial.read();

        if (c == '\n') {
            // End of packet — process the accumulated line
            rxBuffer.trim();   // Remove '\r' if present (Windows line endings)

            if (rxBuffer.length() > 2) {
                dispatchPacket(rxBuffer);
            }

            rxBuffer = "";   // Reset for next packet
        } else {
            rxBuffer += c;

            // Guard against a runaway buffer if '\n' never arrives
            // (e.g., Mega reset mid-packet). 128 bytes >> max packet length (~55 B).
            if (rxBuffer.length() > 128) {
                rxBuffer = "";
            }
        }
    }
}
