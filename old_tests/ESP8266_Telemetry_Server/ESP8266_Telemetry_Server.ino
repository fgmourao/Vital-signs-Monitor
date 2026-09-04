/**
 * @file    ESP8266_Telemetry_Server.ino
 * PROJECT: Vital-signs monitor — small rodents (rat / mouse)
 * @author  Flávio Mourão — Mar, 2026
 *
 * ═══════════════════════════════════════════════════════════════════════════════
 * CÉREBRO 2: ROTEADOR E SERVIDOR WEB (ESP8266)
 * ═══════════════════════════════════════════════════════════════════════════════
 * Cria uma rede Wi-Fi, hospeda o painel HTML/JS e repassa os pacotes JSON
 * recebidos do Arduino Mega via WebSockets para visualização a 100 Hz.
 */

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <WebSocketsServer.h>

// ── CONFIGURAÇÕES DA REDE WI-FI ───────────────────────────────────────────
const char* ssid = "CIRURGIA_ROEDOR";
const char* password = ""; // Deixe vazio para rede aberta no laboratório

ESP8266WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);

// ── INTERFACE GRÁFICA (Página Web Embarcada) ──────────────────────────────
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="pt-BR">
<head>
    <meta charset="UTF-8">
    <title>Monitor Cirúrgico Veterinário</title>
    <style>
        body { background-color: #000; color: #fff; font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; margin: 0; overflow: hidden; }
        #header { background: #111; padding: 10px 20px; font-size: 24px; font-weight: bold; border-bottom: 2px solid #333; color: #888; }
        
        .grid-container { display: flex; height: calc(100vh - 50px); }
        
        /* Painel Lateral de Números */
        .sidebar { width: 300px; background: #0a0a0a; padding: 20px; display: flex; flex-direction: column; gap: 20px; border-right: 2px solid #222; }
        .box { background: #111; padding: 15px; border-radius: 8px; border-left: 5px solid; }
        .box.hr { border-color: #0f0; color: #0f0; }
        .box.spo2 { border-color: #0ff; color: #0ff; }
        .box.rr { border-color: #ff0; color: #ff0; }
        .box.temp { border-color: #f0f; color: #f0f; }
        
        .label { font-size: 16px; text-transform: uppercase; letter-spacing: 1px; color: #aaa; }
        .value { font-size: 64px; font-weight: bold; text-align: right; margin-top: 5px; }
        .unit { font-size: 20px; color: #666; }

        /* Área dos Gráficos */
        .main-content { flex-grow: 1; padding: 20px; display: flex; flex-direction: column; gap: 20px; }
        .chart-container { flex-grow: 1; background: #111; border-radius: 8px; position: relative; padding: 10px; }
        .chart-title { position: absolute; top: 10px; left: 15px; font-size: 14px; color: #666; font-weight: bold; }
        canvas { width: 100%; height: 100%; display: block; }
    </style>
</head>
<body>
    <div id="header">🔴 TELEMETRIA VITAL — BANCADA ESTEREOTÁXICA</div>
    
    <div class="grid-container">
        <div class="sidebar">
            <div class="box hr">
                <div class="label">Frequência (HR)</div>
                <div class="value" id="val_hr">-- <span class="unit">bpm</span></div>
            </div>
            <div class="box spo2">
                <div class="label">Saturação (SpO2)</div>
                <div class="value" id="val_spo2">-- <span class="unit">%</span></div>
            </div>
            <div class="box rr">
                <div class="label">Respiração (RR)</div>
                <div class="value" id="val_rr">-- <span class="unit">rpm</span></div>
            </div>
            <div class="box temp">
                <div class="label">Temperatura</div>
                <div class="value" id="val_temp">--.- <span class="unit">°C</span></div>
            </div>
        </div>

        <div class="main-content">
            <div class="chart-container">
                <div class="chart-title">PLETH (Onda Óptica - MAX30102)</div>
                <canvas id="canvas_ir"></canvas>
            </div>
            <div class="chart-container" style="border-left: 3px solid #ff0;">
                <div class="chart-title" style="color:#ff0;">RESP (Piezoelétrico)</div>
                <canvas id="canvas_piezo"></canvas>
            </div>
        </div>
    </div>

    <script>
        // Buffers de desenho
        const maxPoints = 300; // Resolução do eixo X na tela
        let bufIR = new Array(maxPoints).fill(0);
        let bufPiezo = new Array(maxPoints).fill(0);

        // Setup dos Canvas
        const canIR = document.getElementById('canvas_ir');
        const ctxIR = canIR.getContext('2d');
        const canPiezo = document.getElementById('canvas_piezo');
        const ctxPiezo = canPiezo.getContext('2d');

        function resizeCanvas() {
            canIR.width = canIR.parentElement.clientWidth - 20;
            canIR.height = canIR.parentElement.clientHeight - 20;
            canPiezo.width = canPiezo.parentElement.clientWidth - 20;
            canPiezo.height = canPiezo.parentElement.clientHeight - 20;
        }
        window.addEventListener('resize', resizeCanvas);
        resizeCanvas();

        // Conexão WebSocket com o ESP8266
        var gateway = `ws://${window.location.hostname}:81/`;
        var websocket;

        function initWebSocket() {
            websocket = new WebSocket(gateway);
            websocket.onmessage = onMessage;
        }

        function onMessage(event) {
            try {
                // Parse do JSON recebido do Arduino Mega
                // Exemplo: {"hr":350,"sp":98,"rr":85,"pz":512,"t":37.2,"ir":75000}
                var obj = JSON.parse(event.data);

                // 1. Atualiza os Textos
                document.getElementById('val_hr').innerHTML   = (obj.hr > 0 ? obj.hr : '--') + ' <span class="unit">bpm</span>';
                document.getElementById('val_spo2').innerHTML = (obj.sp > 0 ? obj.sp : '--') + ' <span class="unit">%</span>';
                document.getElementById('val_rr').innerHTML   = (obj.rr > 0 ? obj.rr : '--') + ' <span class="unit">rpm</span>';
                document.getElementById('val_temp').innerHTML = (obj.t > 0 ? obj.t.toFixed(1) : '--.-') + ' <span class="unit">°C</span>';

                // 2. Adiciona dados aos buffers
                bufIR.push(obj.ir); bufIR.shift();
                bufPiezo.push(obj.pz); bufPiezo.shift();

                // 3. Pede para o navegador desenhar a próxima frame
                requestAnimationFrame(drawCharts);
            } catch (e) { console.log("Erro no JSON: " + event.data); }
        }

        function drawSingleChart(ctx, canvas, buffer, color) {
            ctx.clearRect(0, 0, canvas.width, canvas.height);
            ctx.beginPath();
            ctx.strokeStyle = color;
            ctx.lineWidth = 2.5;
            ctx.lineJoin = 'round';

            // Auto-scaling dinâmico (com limite mínimo para não tremer com ruído)
            let maxVal = Math.max(...buffer);
            let minVal = Math.min(...buffer);
            if (maxVal === minVal) { maxVal += 10; minVal -= 10; } // Evita divisão por zero
            let range = maxVal - minVal;
            if (range < 100) { maxVal += 50; minVal -= 50; range = maxVal - minVal; }

            const stepX = canvas.width / (maxPoints - 1);
            for (let i = 0; i < maxPoints; i++) {
                let x = i * stepX;
                // Inverte o Y para que valores maiores fiquem em cima
                let y = canvas.height - (((buffer[i] - minVal) / range) * canvas.height);
                if (i === 0) ctx.moveTo(x, y);
                else ctx.lineTo(x, y);
            }
            ctx.stroke();
        }

        function drawCharts() {
            drawSingleChart(ctxIR, canIR, bufIR, '#0f0'); // Verde para o coração
            drawSingleChart(ctxPiezo, canPiezo, bufPiezo, '#ff0'); // Amarelo para respiração
        }

        // Inicia a aplicação
        initWebSocket();
    </script>
</body>
</html>
)rawliteral";

// ═══════════════════════════════════════════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════════════════════════════════════════
void setup() {
    // A Serial do ESP8266 se comunicará com a Serial3 do Mega
    Serial.begin(115200);

    // 1. Configura o Wi-Fi
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid, password);
    
    // 2. Configura a página da Web
    server.on("/", []() {
        server.send(200, "text/html", index_html);
    });
    server.begin();

    // 3. Inicia o WebSocket para os gráficos de alta velocidade
    webSocket.begin();
}

// ═══════════════════════════════════════════════════════════════════════════════
// LOOP
// ═══════════════════════════════════════════════════════════════════════════════
void loop() {
    webSocket.loop();
    server.handleClient();

    // Se o Mega enviar uma linha (o JSON), o ESP repassa para o WebSocket na hora
    if (Serial.available()) {
        String jsonPayload = Serial.readStringUntil('\n');
        webSocket.broadcastTXT(jsonPayload);
    }
}