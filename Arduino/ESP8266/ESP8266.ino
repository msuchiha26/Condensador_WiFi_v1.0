#include <LittleFS.h>
#include <ESP8266WebServer.h>
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <ArduinoJson.h>
#include <WiFiClientSecure.h>
#include <SoftwareSerial.h>
#include <ESP8266mDNS.h>
#include <DNSServer.h>


// =========================
// CONFIGURACIÓN WIFI
// =========================

// =====================================================
// CONFIGURACIÓN WIFI CON LITTLEFS
// =====================================================
String wifiSSID = "";
String wifiPassword = "";

bool modoConfiguracionWiFi = false;

const char* serverConfig = "https://cyrrotinqlcnfemozcza.supabase.co/rest/v1/config_actual?id=eq.1&select=*";
const char* serverData = "https://cyrrotinqlcnfemozcza.supabase.co/rest/v1/live_data?id=eq.1";
const char* serverHistorico = "https://cyrrotinqlcnfemozcza.supabase.co/rest/v1/datos";
const char* supabaseKey = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJzdXBhYmFzZSIsInJlZiI6ImN5cnJvdGlucWxjbmZlbW96Y3phIiwicm9sZSI6ImFub24iLCJpYXQiOjE3NzczMjEwMDIsImV4cCI6MjA5Mjg5NzAwMn0._wl5C8exO7A1mZxFOiXyx61RBjE2HbhjoSenegDjBww";

// =========================
// BUFFER SERIAL
// =========================
String buffer = "";
SoftwareSerial arduinoSerial(D5, D6);  // RX, TX

// =========================
// SERVIDOR WEB CONFIG WIFI
// =========================
ESP8266WebServer server(80);
DNSServer dnsServer;

// =========================
// CONTROL DE TIEMPO
// =========================
unsigned long lastRequest = 0;

int intervalo = 2000;  // reposo


// =========================
// VARIABLES PARA CAMBIOS
// =========================
int estado_prev = -1;

int estado_actual = 0;
int experimentoActivo = 0;


bool esperandoACK = false;
unsigned long tiempoCFG = 0;
int reintentosCFG = 0;

bool arduinoReconectado = false;

// =========================
// SETUP
// =========================

void setup() {
  arduinoSerial.begin(9600);

  if (!LittleFS.begin()) {
    return;
  }

  conectarWiFiGuardado();
}


// =========================
// ENVIAR DATOS A API
// =========================
void enviarDatos(String linea) {

  linea.trim();
  if (linea.length() == 0) return;

  String v[14];
  int i = 0, start = 0;

  for (int j = 0; j < linea.length(); j++) {
    if (linea[j] == ',') {
      if (i < 14) v[i++] = linea.substring(start, j);
      start = j + 1;
    }
  }
  v[i] = linea.substring(start);

  // validar estructura
  if (i != 13) return;
  // =========================
  // VALIDAR RANGOS
  // =========================
  float tExt = v[0].toFloat() / 100.0;
  float hExt = v[1].toFloat() / 100.0;

  float tInt = v[2].toFloat() / 100.0;
  float hInt = v[3].toFloat() / 100.0;

  float c1 = v[4].toFloat() / 100.0;
  float c2 = v[5].toFloat() / 100.0;
  float c12 = v[6].toFloat() / 100.0;

  float rocioRX = v[7].toFloat() / 100.0;
  float errorRX = v[8].toFloat() / 100.0;

  int pwmRX = v[9].toInt();

  int velARX = v[10].toInt();
  int velBRX = v[11].toInt();

  float corrienteRX = v[12].toFloat() / 100.0;

  int estadoRX = v[13].toInt();

  if (

    // =========================
    // TEMPERATURAS
    // =========================
    tExt < -20 || tExt > 80 || tInt < -20 || tInt > 80 ||

    c1 < -20 || c1 > 80 || c2 < -20 || c2 > 80 || c12 < -20 || c12 > 80 ||

    // =========================
    // HUMEDAD
    // =========================
    hExt < 0 || hExt > 100 || hInt < 0 || hInt > 100 ||

    // =========================
    // ROCÍO
    // =========================
    rocioRX < -30 || rocioRX > 80 ||

    // =========================
    // ERROR
    // =========================
    errorRX < -100 || errorRX > 100 ||

    // =========================
    // PWM
    // =========================
    pwmRX < 0 || pwmRX > 255 ||

    // =========================
    // VENTILADORES
    // =========================
    velARX < 0 || velARX > 255 || velBRX < 0 || velBRX > 255 ||

    // =========================
    // POTENCIA
    // =========================
    corrienteRX < 0 || corrienteRX > 200 ||

    // =========================
    // ESTADO
    // =========================
    estadoRX < 0 || estadoRX > 2

  ) {
    return;
  }

  StaticJsonDocument<512> doc;

  doc["text"] = v[0].toFloat() / 100.0;
  doc["hext"] = v[1].toFloat() / 100.0;
  doc["tint"] = v[2].toFloat() / 100.0;
  doc["hint"] = v[3].toFloat() / 100.0;

  doc["c1"] = v[4].toFloat() / 100.0;
  doc["c2"] = v[5].toFloat() / 100.0;
  doc["c12"] = v[6].toFloat() / 100.0;

  doc["puntorocio"] = v[7].toFloat() / 100.0;

  doc["error"] = v[8].toFloat() / 100.0;

  doc["pwm"] = v[9].toInt();

  doc["vela"] = v[10].toInt();
  doc["velb"] = v[11].toInt();

  doc["corriente"] = v[12].toFloat() / 100.0;
  doc["estado"] = v[13].toInt();


  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;


  http.begin(client, serverData);

  http.addHeader("apikey", supabaseKey);
  http.addHeader("Authorization", String("Bearer ") + supabaseKey);
  http.addHeader("Prefer", "return=minimal");

  http.setTimeout(3000);
  http.addHeader("Content-Type", "application/json");


  // =========================
  // HISTÓRICO
  // =========================
  if (
    estado_actual == 1 && experimentoActivo > 0) {

    StaticJsonDocument<512> hist;

    hist["experiment_id"] = experimentoActivo;

    hist["text"] = tExt;
    hist["hext"] = hExt;

    hist["tint"] = tInt;
    hist["hint"] = hInt;

    hist["c1"] = c1;
    hist["c2"] = c2;
    hist["c12"] = c12;

    hist["puntorocio"] = rocioRX;

    hist["error"] = errorRX;

    hist["pwm"] = pwmRX;

    hist["vela"] = velARX;
    hist["velb"] = velBRX;

    hist["corriente"] = corrienteRX;

    hist["estado"] = estadoRX;

    String jsonHist;
    serializeJson(hist, jsonHist);

    WiFiClientSecure clientHist;
    clientHist.setInsecure();

    HTTPClient httpHist;

    httpHist.begin(clientHist, serverHistorico);

    httpHist.addHeader("apikey", supabaseKey);
    httpHist.addHeader("Authorization", String("Bearer ") + supabaseKey);

    httpHist.addHeader("Content-Type", "application/json");

    httpHist.addHeader("Prefer", "return=minimal");

    httpHist.POST(jsonHist);

    httpHist.end();
  }


  String json;
  serializeJson(doc, json);

  http.PATCH(json);

  http.end();
}


// -----------------------------------------------------
// CARGAR CONFIGURACIÓN WIFI
// -----------------------------------------------------

bool cargarConfiguracionWiFi() {

  if (!LittleFS.exists("/wifi.txt")) {
    return false;
  }

  File archivo = LittleFS.open("/wifi.txt", "r");

  if (!archivo) {
    return false;
  }

  wifiSSID = archivo.readStringUntil('\n');
  wifiPassword = archivo.readStringUntil('\n');

  wifiSSID.trim();
  wifiPassword.trim();

  archivo.close();

  if (wifiSSID.length() == 0) {
    return false;
  }
  return true;
}


// -----------------------------------------------------
// GUARDAR CONFIGURACIÓN WIFI
// -----------------------------------------------------

bool guardarConfiguracionWiFi(String nuevoSSID, String nuevaPassword) {

  File archivo = LittleFS.open("/wifi.txt", "w");

  if (!archivo) {
    return false;
  }

  archivo.println(nuevoSSID);
  archivo.println(nuevaPassword);

  archivo.close();

  wifiSSID = nuevoSSID;
  wifiPassword = nuevaPassword;
  return true;
}


// -----------------------------------------------------
// PÁGINA PRINCIPAL DE CONFIGURACIÓN
// -----------------------------------------------------

void paginaConfiguracion() {

  String html = R"rawliteral(
<!DOCTYPE html>
<html lang="es">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">

<title>Configuración WiFi</title>

<style>

body {
  font-family: Arial, sans-serif;
  background: #f2f2f2;
  margin: 0;
  padding: 20px;
}

.container {
  max-width: 450px;
  margin: 30px auto;
  background: white;
  padding: 25px;
  border-radius: 12px;
  box-shadow: 0 2px 10px rgba(0,0,0,0.15);
}

h1 {
  text-align: center;
}

p {
  color: #555;
}

label {
  display: block;
  margin-top: 15px;
  font-weight: bold;
}

input {
  width: 100%;
  padding: 12px;
  margin-top: 6px;
  box-sizing: border-box;
  border: 1px solid #ccc;
  border-radius: 6px;
  font-size: 16px;
}

button {
  width: 100%;
  padding: 12px;
  margin-top: 20px;
  border: none;
  border-radius: 6px;
  background: #333;
  color: white;
  font-size: 16px;
  cursor: pointer;
}

</style>
</head>

<body>

<div class="container">

<h1>Condensador</h1>

<p>Configuración de red WiFi</p>

<form action="/guardar" method="POST">

<label>Nombre de la red WiFi</label>

<input
  type="text"
  name="ssid"
  placeholder="SSID"
  required>

<label>Contraseña</label>

<div style="display: flex; gap: 8px;">

  <input
    type="password"
    id="password"
    name="password"
    placeholder="Contraseña"
    style="flex: 1;"
    required>

  <button
    type="button"
    onclick="mostrarPassword()"
    style="width: auto; margin-top: 6px;">
    Mostrar
  </button>

</div>

<button type="submit">
  Guardar configuración
</button>

</form>

</div>

<script>

function mostrarPassword() {

  const campo = document.getElementById("password");

  if (campo.type === "password") {

    campo.type = "text";

  } else {

    campo.type = "password";

  }

}

</script>

</body>
</html>
)rawliteral";

  server.send(200, "text/html", html);
}


// -----------------------------------------------------
// GUARDAR CONFIGURACIÓN DESDE LA PÁGINA
// -----------------------------------------------------

void guardarDesdeWeb() {

  if (!server.hasArg("ssid") || !server.hasArg("password")) {

    server.send(
      400,
      "text/plain",
      "Faltan datos");

    return;
  }

  String nuevoSSID = server.arg("ssid");
  String nuevaPassword = server.arg("password");

  nuevoSSID.trim();
  nuevaPassword.trim();

  if (nuevoSSID.length() == 0) {

    server.send(
      400,
      "text/plain",
      "SSID vacío");

    return;
  }

  if (guardarConfiguracionWiFi(
        nuevoSSID,
        nuevaPassword)) {

    server.send(
      200,
      "text/html",
      "<html><body><h1>Configuración guardada</h1>"
      "<p>El ESP8266 se reiniciará y tratará de conectarse a la red.</p>"
      "</body></html>");

    delay(2000);

    ESP.restart();

  } else {

    server.send(
      500,
      "text/plain",
      "No se pudo guardar la configuración");
  }
}


// -----------------------------------------------------
// BORRAR CONFIGURACIÓN WIFI
// -----------------------------------------------------

void borrarConfiguracionWiFi() {

  if (LittleFS.exists("/wifi.txt")) {

    LittleFS.remove("/wifi.txt");
    server.send(
      200,
      "text/html",
      "<html><body><h1>Configuración eliminada</h1>"
      "<p>El ESP8266 se reiniciará.</p>"
      "</body></html>");

    delay(2000);

    ESP.restart();

  } else {

    server.send(
      200,
      "text/plain",
      "No había configuración guardada");
  }
}


// -----------------------------------------------------
// INICIAR SERVIDOR DE CONFIGURACIÓN
// -----------------------------------------------------

void iniciarServidorConfiguracion() {

  server.on(
    "/",
    HTTP_GET,
    paginaConfiguracion);

  server.on(
    "/guardar",
    HTTP_POST,
    guardarDesdeWeb);

  server.on(
    "/borrar",
    HTTP_GET,
    borrarConfiguracionWiFi);

  server.begin();
}


// -----------------------------------------------------
// MODO ACCESS POINT
// -----------------------------------------------------

void iniciarModoConfiguracion() {

  modoConfiguracionWiFi = true;

  WiFi.disconnect(true);
  delay(500);

  WiFi.mode(WIFI_AP);

  WiFi.softAP("esp");

  IPAddress IP = WiFi.softAPIP();

  // Servidor DNS para el modo de configuración
  dnsServer.start(53, "*", IP);

  iniciarServidorConfiguracion();
}


// -----------------------------------------------------
// CONECTAR A WIFI GUARDADO
// -----------------------------------------------------

bool conectarWiFiGuardado() {

  if (!cargarConfiguracionWiFi()) {
    iniciarModoConfiguracion();
    return false;
  }

  modoConfiguracionWiFi = false;

  WiFi.mode(WIFI_STA);

  WiFi.begin(
    wifiSSID.c_str(),
    wifiPassword.c_str());

  unsigned long inicio = millis();

  while (
    WiFi.status() != WL_CONNECTED && millis() - inicio < 20000) {

    delay(500);
  }

  if (WiFi.status() == WL_CONNECTED) {

    iniciarServidorConfiguracion();

    MDNS.begin("esp");
    MDNS.addService("http", "tcp", 80);

    return true;
  }

  iniciarModoConfiguracion();

  return false;
}


// =========================
// LOOP
// =========================
void loop() {

  // =========================
  // 1. RECIBIR DATOS DEL ARDUINO
  // =========================
  while (arduinoSerial.available()) {
    char c = arduinoSerial.read();

    if (c == '\n') {

      buffer.trim();

      // =========================
      // ACK CFG
      // =========================
      if (
        buffer == "ACK" && buffer.length() == 3) {

        buffer = "";
        esperandoACK = false;
        reintentosCFG = 0;

        estado_prev = estado_actual;
      } else {

        if (WiFi.status() == WL_CONNECTED) {
          enviarDatos(buffer);
        }

        arduinoReconectado = true;
      }

      buffer = "";
    } else {
      buffer += c;

      if (buffer.length() > 120) {
        buffer = "";
      }
    }
  }


  // Actualizar el servicio mDNS mientras hay conexión WiFi
  if (WiFi.status() == WL_CONNECTED) {
    MDNS.update();
  }

  // =========================
  // RESINCRONIZAR CFG
  // =========================
  if (
    arduinoReconectado && !esperandoACK) {

    estado_prev = -1;

    arduinoReconectado = false;
  }

  // =========================
  // SERVIDOR WEB / CONFIG WIFI
  // =========================

  server.handleClient();

  if (modoConfiguracionWiFi) {


    dnsServer.processNextRequest();
    // Mientras está en modo configuración
    // no intentamos consultar Supabase.
    return;
  }


  // =========================
  // RECONEXIÓN WIFI
  // =========================

  static unsigned long ultimoIntentoWiFi = 0;
  static unsigned long inicioDesconexion = 0;

  if (WiFi.status() != WL_CONNECTED) {

    if (inicioDesconexion == 0) {

      inicioDesconexion = millis();
    }


    // Intentar reconectar cada 10 segundos

    if (millis() - ultimoIntentoWiFi > 10000) {

      ultimoIntentoWiFi = millis();
      WiFi.disconnect();

      delay(500);

      WiFi.mode(WIFI_STA);

      WiFi.begin(
        wifiSSID.c_str(),
        wifiPassword.c_str());
    }


    // Si lleva 30 segundos sin conexión,
    // entrar en modo configuración.

    if (millis() - inicioDesconexion > 30000) {
      iniciarModoConfiguracion();
    }

    return;
  }

  // WiFi recuperado

  if (inicioDesconexion != 0) {

    inicioDesconexion = 0;
  }

  // =========================
  // 2. CONSULTAR CONFIG
  // =========================
  if (WiFi.status() == WL_CONNECTED) {

    if (millis() - lastRequest > intervalo) {

      lastRequest = millis();

      WiFiClientSecure client;
      client.setInsecure();
      HTTPClient http;

      http.begin(client, serverConfig);
      http.addHeader("apikey", supabaseKey);
      http.addHeader("Authorization", String("Bearer ") + supabaseKey);

      int httpCode = http.GET();

      if (httpCode == 200) {

        String payload = http.getString();

        StaticJsonDocument<256> doc;
        DeserializationError error = deserializeJson(doc, payload);

        if (error) {
          http.end();
          return;
        }

        JsonArray arr = doc.as<JsonArray>();

        if (arr.size() == 0) {
          http.end();
          return;
        }

        JsonObject obj = arr[0];
        estado_actual = obj["estado_control"];

        experimentoActivo = obj["experimento_id"];

        int estado = estado_actual;

        int modo = obj["modomanual"];
        int pwm = obj["pwm"];

        float kp = obj["kp"];
        float ki = obj["ki"];

        int velA = obj["vela"];
        int velB = obj["velb"];

        float humObj = obj["humedad_objetivo"];
        float tserial = obj["tserial"];

        if (
          pwm < 0 || pwm > 255 || velA < 0 || velA > 255 || velB < 0 || velB > 255 || kp < 0 || kp > 100 || ki < 0 || ki > 100 || estado < 0 || estado > 1) {
          http.end();
          return;
        }

        // =========================
        // CAMBIOS DE ESTADO
        // =========================

        // =========================
        // ENVIAR CONFIG
        // =========================
        if (
          estado != estado_prev && !esperandoACK) {

          //if (true) {

          String cfg = "CFG,";

          cfg += String(modo) + ",";
          cfg += String(pwm) + ",";
          cfg += String((int)(kp * 100)) + ",";
          cfg += String((int)(ki * 100)) + ",";
          cfg += String(velA) + ",";
          cfg += String(velB) + ",";
          cfg += String(humObj, 2) + ",";
          cfg += String(tserial, 0) + ",";
          cfg += String(estado);

          if (!esperandoACK) {
            reintentosCFG = 0;
          }

          arduinoSerial.println(cfg);

          esperandoACK = true;
          tiempoCFG = millis();
          reintentosCFG++;
        }

        // =========================
        // AJUSTAR FRECUENCIA
        // =========================

        intervalo = 2000;
      }

      http.end();
      // =========================
      // REINTENTO CFG
      // =========================
      if (
        esperandoACK && millis() - tiempoCFG > 3000) {

        esperandoACK = false;
        tiempoCFG = millis();
      }
      if (reintentosCFG >= 3) {

        esperandoACK = false;
        reintentosCFG = 0;
      }
    }
  }
}
