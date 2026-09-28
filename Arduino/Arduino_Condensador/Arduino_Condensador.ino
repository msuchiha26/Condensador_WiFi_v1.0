#include <Wire.h>
#include <DHT.h>
#include <INA226_WE.h>
#include <OneWire.h>
#include <DallasTemperature.h>

int AltDHT22 = 0;

#define espSerial Serial1  // RX, TX

// ================== INA226 ==================
#define INA226_ADDRESS 0x40

INA226_WE ina226(INA226_ADDRESS);

// ================== DHT22 ==================

#define DHT_EXT_PIN 26
#define DHT_INT_PIN 27
#define DHT_TYPE DHT22

DHT dhtExt(DHT_EXT_PIN, DHT_TYPE);
DHT dhtInt(DHT_INT_PIN, DHT_TYPE);


// ================== DS18B20 ==================

#define ONE_WIRE_BUS 28

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature ds18b20(&oneWire);


// ================== DIRECCIONES DS18B20 ==================

// SensorTemperatura01 -> c1
DeviceAddress sensorC1 = {
  0x28, 0xD4, 0xF8, 0xE6,
  0x54, 0x25, 0x0B, 0x25
};

// SensorTemperatura02 -> c2
DeviceAddress sensorC2 = {
  0x28, 0xA7, 0x8F, 0x10,
  0x54, 0x25, 0x0B, 0xA0
};

// =====================================================
// ACTUADORES
// =====================================================

// BTS7960 - 2 Peltier
#define PELTIER_RPWM 6
#define PELTIER_LPWM 7

// L298N - ventiladores
// Canal A -> cámara interna
#define FAN_A_PWM 44
#define FAN_A_LOW 45

// Canal B -> disipación de calor
#define FAN_B_LOW 46
#define FAN_B_PWM 47

// 🔥 buffer de recepción
String buffer = "";

// ================== CONFIG ==================
#define BAUD 9600

const unsigned long INTERVALO_SENSOR = 2000;
const unsigned long INTERVALO_CONTROL = 5000;

// ---------- TIEMPO ----------

float intervaloDatos = 15;  //Periodo de muestreo para base de datos

unsigned long intervaloSensor;
unsigned long intervaloControl;
unsigned long intervaloSerial;

// ---------- VAriables iniciales ----------
float c12 = 0;
float c1 = 0;
float c2 = 0;

float I = 0;  // Potencia consumida [W]

float puntoRocio = 0;
float error = 0;

float pwm = 0;


float hInt = 0;
float tInt = 0;

float hExt = 0;
float tExt = 0;

bool okInt = true, okExt = true;

// ---------- CONTROL ----------
unsigned long TSensor = 0, TControl = 0, TSerial = 0;

float kp = 1.3, ki = 0.05;
float errorAcumulado = 0.0;
unsigned long tiempoAnterior = 0;
const float maxIntegracion = 100.0;

bool modoManual = false;
int pwmManual = 0;

int Vel_PWM_A = 0, Vel_PWM_B = 0;

int estadoSistema = 0;
// 0 = Operación normal
// 1 = Falla de sensores
// 2 = Humedad objetivo alcanzada
int estadoControl = 0;  // 0 reposo, 1 ejecución

float humedadObjetivo = 0;
bool pausaHumedad = false;

// =========================
// PROMEDIOS
// =========================
float sumaCorriente = 0;
float sumaPWM = 0;

float suma_tExt = 0;
float suma_tInt = 0;
float suma_hExt = 0;
float suma_hInt = 0;
float suma_c12 = 0;
float suma_c1 = 0;
float suma_c2 = 0;
float sumaRocio = 0;
float sumaError = 0;

int contadorMuestras = 0;

// promedios finales
float corrienteProm = 0;
float pwmProm = 0;

float tExtProm = 0;
float tIntProm = 0;
float hExtProm = 0;
float hIntProm = 0;
float c12Prom = 0;
float c1Prom = 0;
float c2Prom = 0;
float rocProm = 0;
float errorProm = 0;


// =========================
// FUNCIONES AUXILIARES
// =========================

//---------------- Calcular PuntoRocio --------------------------

float calcularPuntoRocio(float tempC, float humRel) {
  float a = 17.62, b = 243.12;
  float gamma = log(humRel / 100.0) + (a * tempC) / (b + tempC);
  return ((b * gamma) / (a - gamma));
}

//-------------------- Leer Potencia del Sistema ---------------------------

float leerPotenciaINA226() {

  float voltaje = ina226.getBusVoltage_V();
  float corriente = ina226.getCurrent_mA() / 1000.0;

  // Protección contra valores inválidos
  if (isnan(voltaje) || isnan(corriente)) {
    return 0.0;
  }

  // Potencia = Voltaje × Corriente
  return voltaje * corriente;
}

//-----------------Actualizar Variables-------------------------

// =========================
// ACTUALIZAR VARIABLES REALES
// =========================

void actualizarVariables() {

// =====================================================
// DHT22
// Un sensor por ciclo
// Si falla, se reintenta inmediatamente hasta 3 veces
// =====================================================

if (AltDHT22 == 0) {

  bool lecturaOK = false;

  for (int intento = 1; intento <= 3; intento++) {

    float nueva_tExt = dhtExt.readTemperature();
    float nueva_hExt = dhtExt.readHumidity();

    if (!isnan(nueva_tExt) && !isnan(nueva_hExt)) {

      tExt = nueva_tExt;
      hExt = nueva_hExt;
      okExt = true;

      lecturaOK = true;

      Serial.print("DHT22 EXTERIOR OK - intento ");
      Serial.println(intento);

      break;
    }

    // Si falló, intentar inmediatamente otra vez
    if (intento < 3) {
      Serial.print("DHT22 EXTERIOR fallo - reintento ");
      Serial.println(intento + 1);
    }
  }

  if (lecturaOK) {
    AltDHT22 = 1;
  } else {
    okExt = false;
    Serial.println("ERROR: DHT22 EXTERIOR - 3 intentos fallidos");
  }

} else {

  bool lecturaOK = false;

  for (int intento = 1; intento <= 10; intento++) {

    float nueva_tInt = dhtInt.readTemperature();
    float nueva_hInt = dhtInt.readHumidity();

    if (!isnan(nueva_tInt) && !isnan(nueva_hInt)) {

      tInt = nueva_tInt;
      hInt = nueva_hInt;
      okInt = true;

      lecturaOK = true;

      Serial.print("DHT22 INTERIOR OK - intento ");
      Serial.println(intento);

      break;
    }

    if (intento < 3) {
      Serial.print("DHT22 INTERIOR fallo - reintento ");
      Serial.println(intento + 1);
    }
  }

  if (lecturaOK) {
    AltDHT22 = 0;
  } else {
    okInt = false;
    Serial.println("ERROR: DHT22 INTERIOR - 3 intentos fallidos");
  }
}


  // =====================================================
  // DS18B20
  // =====================================================

  ds18b20.requestTemperatures();

  float nueva_c1 = ds18b20.getTempC(sensorC1);
  float nueva_c2 = ds18b20.getTempC(sensorC2);

  if (nueva_c1 != DEVICE_DISCONNECTED_C &&
      nueva_c2 != DEVICE_DISCONNECTED_C &&
      !isnan(nueva_c1) &&
      !isnan(nueva_c2)) {

    c1 = nueva_c1;
    c2 = nueva_c2;

    c12 = (c1 + c2) / 2.0;

  } else {

    c1 = NAN;
    c2 = NAN;
    c12 = NAN;
  }


  // =====================================================
  // INA226
  // =====================================================

  I = leerPotenciaINA226();


  // =====================================================
  // PUNTO DE ROCÍO
  // =====================================================
  
  // Solo se calcula cuando tenemos una lectura exterior válida
  if (okExt) {

    puntoRocio = calcularPuntoRocio(tExt, hExt);

  } else {

    puntoRocio = NAN;
  }
}



//------------------Actualizar Estados----------------------

void actualizarEstadoSistema() {

  // 0 = Operación normal
  // 1 = Falla de sensores
  // 2 = Humedad objetivo alcanzada

  if (isnan(c1) || isnan(c2) || !okInt || !okExt) {

    estadoSistema = 1;
  } else {

    estadoSistema = 0;
  }

  if (estadoSistema == 0 && estadoControl == 1 && !isnan(hExt)) {

    if (hExt < humedadObjetivo) {

      pausaHumedad = true;
    } else if (hExt > humedadObjetivo) {

      pausaHumedad = false;
    }

    if (pausaHumedad) {

      estadoSistema = 2;
    }
  }
}

//---------------Acumular promedios-------------------------

void acumularPromedios() {

  sumaCorriente += I;

  pwm = constrain(pwm, 0, 255);

  sumaPWM += pwm;

  suma_tExt += tExt;
  suma_tInt += tInt;
  suma_hExt += hExt;
  suma_hInt += hInt;
  suma_c12 += c12;
  suma_c1 += c1;
  suma_c2 += c2;
  sumaRocio += puntoRocio;

  if (estadoControl == 1 && !modoManual) {

    sumaError += error;
  }

  contadorMuestras++;
}

//-----------------Deshabilitar Actuadores--------------------

void deshabilitarActuadores() {

  // =========================
  // PELTIER
  // =========================

  analogWrite(PELTIER_RPWM, 0);
  analogWrite(PELTIER_LPWM, 0);

  // =========================
  // VENTILADORES - CÁMARA
  // =========================

  analogWrite(FAN_A_PWM, 0);
  digitalWrite(FAN_A_LOW, LOW);

  // =========================
  // VENTILADORES - DISIPACIÓN
  // =========================

  analogWrite(FAN_B_PWM, 0);
  digitalWrite(FAN_B_LOW, LOW);

  // =========================
  // IMPORTANTE
  // =========================
  // NO modificar aquí:
  //
  // pwmManual
  // Vel_PWM_A
  // Vel_PWM_B
  //
  // Estos valores corresponden a la
  // configuración recibida desde ESP8266.

  pwm = 0;

  errorAcumulado = 0;
}

// =====================================================
// APLICAR ACTUADORES
// =====================================================

void aplicarActuadores() {

  // =========================
  // LIMITAR VALORES
  // =========================

  pwm = constrain(pwm, 0, 255);

  Vel_PWM_A = constrain(Vel_PWM_A, 0, 255);
  Vel_PWM_B = constrain(Vel_PWM_B, 0, 255);


  // =========================
  // PELTIER
  // =========================

  // Las dos Peltier trabajan juntas
  // RPWM recibe el PWM
  // LPWM permanece en LOW

  analogWrite(PELTIER_RPWM, (int)pwm);
  analogWrite(PELTIER_LPWM, 0);


  // =========================
  // VENTILADORES - CÁMARA
  // =========================

  // IN1 = PWM
  // IN2 = LOW

  analogWrite(FAN_A_PWM, Vel_PWM_A);
  digitalWrite(FAN_A_LOW, LOW);


  // =========================
  // VENTILADORES - DISIPACIÓN
  // =========================

  // IN3 = LOW
  // IN4 = PWM

  digitalWrite(FAN_B_LOW, LOW);
  analogWrite(FAN_B_PWM, Vel_PWM_B);
}

//-----------------Control Auto------------------------------
void ejecutarControlAutomatico() {

  error = puntoRocio - c12;

  float dt_control = ((millis() - tiempoAnterior) / 1000.0);

  tiempoAnterior = millis();

  if (pwm > 0 && pwm < 255) {
    errorAcumulado += error * dt_control;
  }

  errorAcumulado = constrain(errorAcumulado, -maxIntegracion, maxIntegracion);

  float salidaPI = kp * error + ki * errorAcumulado;

  float tempObjetivo = salidaPI + c12;

  pwm = (tempObjetivo - 28.0) / -0.0745;

  pwm = constrain(pwm, 0, 255);
}

//-----------------Control Manual-----------------------------

void ejecutarControlManual() {

  pwm = constrain(pwmManual, 0, 255);
}

//----------------Recibir Configuracion----------------------

void recibirConfiguracion() {
  // =========================
  // RECEPCIÓN SERIAL NO BLOQUEANTE
  // =========================
  while (espSerial.available()) {

    char c = espSerial.read();

    if (c == '\n') {

      buffer.trim();

      if (buffer.startsWith("CFG,")) {

        //Serial.println(buffer);

        String datos = buffer.substring(4);

        Serial.println();
        Serial.println("========== CONFIGURACION RECIBIDA ==========");
        Serial.println("Trama completa:");
        Serial.println(buffer);
        Serial.println();

        int idx[8];
        int start = 0;

        for (int i = 0; i < 8; i++) {
          idx[i] = datos.indexOf(',', start);
          start = idx[i] + 1;
        }

        // =========================
        // VALIDAR TRAMA
        // =========================
        bool valido = true;

        for (int i = 0; i < 8; i++) {

          if (idx[i] == -1) {
            valido = false;
          }
        }

        if (!valido) {

          buffer = "";
          break;
        }

        modoManual = datos.substring(
                            0,
                            idx[0])
                       .toInt();

        pwmManual = datos.substring(
                           idx[0] + 1,
                           idx[1])
                      .toInt();

        kp = datos.substring(
                    idx[1] + 1,
                    idx[2])
               .toInt()
             / 100.0;

        ki = datos.substring(
                    idx[2] + 1,
                    idx[3])
               .toInt()
             / 100.0;

        Vel_PWM_A = datos.substring(
                           idx[3] + 1,
                           idx[4])
                      .toInt();

        Vel_PWM_B = datos.substring(
                           idx[4] + 1,
                           idx[5])
                      .toInt();

        humedadObjetivo = datos.substring(
                                 idx[5] + 1,
                                 idx[6])
                            .toFloat();

        intervaloDatos = datos.substring(
                                idx[6] + 1,
                                idx[7])
                           .toFloat();

        estadoControl = datos.substring(
                               idx[7] + 1)
                          .toInt();


        Serial.println("Valores recibidos:");

        Serial.print("modoManual       = ");
        Serial.println(modoManual);

        Serial.print("pwmManual        = ");
        Serial.println(pwmManual);

        Serial.print("kp               = ");
        Serial.println(kp, 2);

        Serial.print("ki               = ");
        Serial.println(ki, 2);

        Serial.print("Vel_PWM_A / velA = ");
        Serial.println(Vel_PWM_A);

        Serial.print("Vel_PWM_B / velB = ");
        Serial.println(Vel_PWM_B);

        Serial.print("humedadObjetivo  = ");
        Serial.println(humedadObjetivo, 2);

        Serial.print("intervaloDatos   = ");
        Serial.println(intervaloDatos, 2);

        Serial.print("estadoControl    = ");
        Serial.println(estadoControl);

        Serial.println("============================================");

        espSerial.println("ACK");


        intervaloSerial = calcularTSerial();

        errorAcumulado = 0;
      }

      // limpiar buffer
      buffer = "";
    }

    else {
      buffer += c;
      if (buffer.length() > 80) buffer = "";
    }
  }
}

//-----------------Enviar Datos-------------------------------

void enviarDatos() {
  // =========================
  // ENVÍO SERIAL (ESCALADO x100 SIN VARIABLES EXTRA)
  // =========================

  // =========================
  // CALCULAR PROMEDIOS
  // =========================
  if (contadorMuestras > 0) {

    corrienteProm = sumaCorriente / contadorMuestras;
    pwmProm = sumaPWM / contadorMuestras;

    tExtProm = suma_tExt / contadorMuestras;
    tIntProm = suma_tInt / contadorMuestras;
    hExtProm = suma_hExt / contadorMuestras;
    hIntProm = suma_hInt / contadorMuestras;
    c12Prom = suma_c12 / contadorMuestras;
    c1Prom = suma_c1 / contadorMuestras;
    c2Prom = suma_c2 / contadorMuestras;
    rocProm = sumaRocio / contadorMuestras;
    errorProm = sumaError / contadorMuestras;
  }


  // =========================
  // DEBUG: DATOS QUE VAN A ESP
  // =========================

  Serial.println();
  Serial.println("========== DATOS A ESP8266 ==========");

  Serial.print("tExt = ");
  Serial.println(tExtProm, 2);

  Serial.print("hExt = ");
  Serial.println(hExtProm, 2);

  Serial.print("tInt = ");
  Serial.println(tIntProm, 2);

  Serial.print("hInt = ");
  Serial.println(hIntProm, 2);

  Serial.print("c1 = ");
  Serial.println(c1Prom, 2);

  Serial.print("c2 = ");
  Serial.println(c2Prom, 2);

  Serial.print("c12 = ");
  Serial.println(c12Prom, 2);

  Serial.print("PuntoRocio = ");
  Serial.println(rocProm, 2);

  Serial.print("Error = ");
  Serial.println(errorProm, 2);

  Serial.print("PWM = ");
  Serial.println(pwmProm, 0);

  Serial.print("VelA = ");
  Serial.println(Vel_PWM_A);

  Serial.print("VelB = ");
  Serial.println(Vel_PWM_B);

  Serial.print("Potencia = ");
  Serial.println(corrienteProm, 3);

  Serial.print("Estado = ");
  Serial.println(estadoSistema);

  Serial.println("====================================");


  // =========================
  // ENVÍO REAL A ESP8266
  // =========================

  espSerial.print((long)(tExtProm * 100));
  espSerial.print(",");
  espSerial.print((long)(hExtProm * 100));
  espSerial.print(",");
  espSerial.print((long)(tIntProm * 100));
  espSerial.print(",");
  espSerial.print((long)(hIntProm * 100));
  espSerial.print(",");
  espSerial.print((long)(c1Prom * 100));
  espSerial.print(",");
  espSerial.print((long)(c2Prom * 100));
  espSerial.print(",");
  espSerial.print((long)(c12Prom * 100));
  espSerial.print(",");
  espSerial.print((long)(rocProm * 100));
  espSerial.print(",");
  espSerial.print((long)(errorProm * 100));
  espSerial.print(",");
  espSerial.print((int)(pwmProm));
  espSerial.print(",");
  espSerial.print(Vel_PWM_A);
  espSerial.print(",");
  espSerial.print(Vel_PWM_B);
  espSerial.print(",");
  espSerial.print((long)(corrienteProm * 100));
  espSerial.print(",");
  espSerial.println(estadoSistema);

  // =========================
  // RESET PROMEDIOS
  // =========================
  sumaCorriente = 0;
  sumaPWM = 0;

  suma_tExt = 0;
  suma_tInt = 0;
  suma_hExt = 0;
  suma_hInt = 0;
  suma_c12 = 0;
  suma_c1 = 0;
  suma_c2 = 0;
  sumaRocio = 0;
  sumaError = 0;

  contadorMuestras = 0;
}

//-----------------Ejecutar Control--------------------------

void ejecutarControl() {

  Serial.println();
  Serial.println("========== EJECUTANDO CONTROL ==========");

  Serial.print("estadoControl = ");
  Serial.println(estadoControl);

  Serial.print("estadoSistema = ");
  Serial.println(estadoSistema);

  Serial.print("modoManual = ");
  Serial.println(modoManual);

  Serial.print("pwmManual = ");
  Serial.println(pwmManual);

  Serial.print("Vel_PWM_A = ");
  Serial.println(Vel_PWM_A);

  Serial.print("Vel_PWM_B = ");
  Serial.println(Vel_PWM_B);

  // =========================
  // REPOSO
  // =========================

  if (estadoControl == 0) {

    Serial.println(">>> REPOSO");

    deshabilitarActuadores();

    pausaHumedad = false;
    humedadObjetivo = 0;

    pwmProm = 0;
    corrienteProm = 0;

    sumaPWM = 0;
    sumaCorriente = 0;
  }

  // =========================
  // ESTADOS QUE DETIENEN
  // =========================

  else if (estadoSistema == 1 || estadoSistema == 2) {

    Serial.println(">>> SISTEMA DETENIDO POR ESTADO");

    deshabilitarActuadores();
  }

  // =========================
  // EJECUCIÓN NORMAL
  // =========================

  else {

    Serial.println(">>> EJECUCION NORMAL");

    if (!modoManual) {

      Serial.println(">>> CONTROL AUTOMATICO");

      ejecutarControlAutomatico();

    } else {

      Serial.println(">>> CONTROL MANUAL");

      ejecutarControlManual();
    }

    aplicarActuadores();

    Serial.print("PWM aplicado = ");
    Serial.println(pwm);

    Serial.print("VelA aplicado = ");
    Serial.println(Vel_PWM_A);

    Serial.print("VelB aplicado = ");
    Serial.println(Vel_PWM_B);
  }

  Serial.println(">>> SALIDAS FISICAS <<<");

  Serial.print("D8 PELTIER_RPWM = ");
  Serial.println(analogRead(PELTIER_RPWM));

  Serial.print("D44 FAN_A_PWM = ");
  Serial.println(analogRead(FAN_A_PWM));

  Serial.print("D46 FAN_B_PWM = ");
  Serial.println(analogRead(FAN_B_PWM));

  Serial.println("========================================");
}
//-----------------Tiempos para la ejecucion-------------------

unsigned long calcularTSerial() {

  unsigned long t = intervaloDatos * 1000UL;

  return t;
}



// =====================================================
// MOSTRAR SENSORES EN SERIAL DEL PC
// =====================================================

void mostrarSensoresSerial() {

  Serial.println();
  Serial.println("======================================");
  Serial.println("         LECTURA DE SENSORES");
  Serial.println("======================================");

  // DHT22 EXTERIOR
  Serial.println("DHT22 EXTERIOR:");

  if (okExt) {
    Serial.print("  Temperatura: ");
    Serial.print(tExt, 2);
    Serial.println(" °C");

    Serial.print("  Humedad:     ");
    Serial.print(hExt, 2);
    Serial.println(" %");
  } else {
    Serial.println("  ERROR DE LECTURA");
  }


  // DHT22 INTERIOR
  Serial.println();
  Serial.println("DHT22 INTERIOR:");

  if (okInt) {
    Serial.print("  Temperatura: ");
    Serial.print(tInt, 2);
    Serial.println(" °C");

    Serial.print("  Humedad:     ");
    Serial.print(hInt, 2);
    Serial.println(" %");
  } else {
    Serial.println("  ERROR DE LECTURA");
  }


  // DS18B20
  Serial.println();
  Serial.println("DS18B20:");

  if (!isnan(c1)) {
    Serial.print("  Sensor C1: ");
    Serial.print(c1, 2);
    Serial.println(" °C");
  } else {
    Serial.println("  Sensor C1: ERROR");
  }

  if (!isnan(c2)) {
    Serial.print("  Sensor C2: ");
    Serial.print(c2, 2);
    Serial.println(" °C");
  } else {
    Serial.println("  Sensor C2: ERROR");
  }

  if (!isnan(c12)) {
    Serial.print("  Promedio C1/C2: ");
    Serial.print(c12, 2);
    Serial.println(" °C");
  }


  // PUNTO DE ROCÍO
  Serial.println();
  Serial.println("PUNTO DE ROCÍO:");

  if (!isnan(puntoRocio)) {
    Serial.print("  ");
    Serial.print(puntoRocio, 2);
    Serial.println(" °C");
  } else {
    Serial.println("  ERROR");
  }


  // INA226
  Serial.println();
  Serial.println("INA226:");

  Serial.print("  Potencia: ");
  Serial.print(I, 3);
  Serial.println(" W");


  // ESTADO
  Serial.println();
  Serial.print("Estado sistema: ");
  Serial.println(estadoSistema);

  Serial.println("======================================");
}





// =========================
// SETUP
// =========================
void setup() {

  // =====================================================
  // CONFIGURAR PINES DE ACTUADORES
  // =====================================================

  // BTS7960
  pinMode(PELTIER_RPWM, OUTPUT);
  pinMode(PELTIER_LPWM, OUTPUT);

  // L298N
  pinMode(FAN_A_PWM, OUTPUT);
  pinMode(FAN_A_LOW, OUTPUT);

  pinMode(FAN_B_LOW, OUTPUT);
  pinMode(FAN_B_PWM, OUTPUT);


  // Inicialmente todo apagado
  analogWrite(PELTIER_RPWM, 0);
  analogWrite(PELTIER_LPWM, 0);

  analogWrite(FAN_A_PWM, 0);
  digitalWrite(FAN_A_LOW, LOW);

  digitalWrite(FAN_B_LOW, LOW);
  analogWrite(FAN_B_PWM, 0);


  Serial.begin(BAUD);
  espSerial.begin(BAUD);

  // =========================
  // INICIALIZAR I2C
  // =========================

  Wire.begin();

  // Velocidad I2C estándar
  Wire.setClock(100000);

  // Evita que una falla del bus I2C deje bloqueado
  // indefinidamente al Arduino.
  Wire.setWireTimeout(25000, true);

  // =========================
  // INICIALIZAR DHT22
  // =========================

  dhtExt.begin();
  dhtInt.begin();

  Serial.println("DHT22 inicializados");


  // =========================
  // INICIALIZAR DS18B20
  // =========================

  ds18b20.begin();

  if (ds18b20.isConnected(sensorC1)) {
    Serial.println("SensorTemperatura01 detectado");
  } else {
    Serial.println("ERROR: SensorTemperatura01 NO detectado");
  }

  if (ds18b20.isConnected(sensorC2)) {
    Serial.println("SensorTemperatura02 detectado");
  } else {
    Serial.println("ERROR: SensorTemperatura02 NO detectado");
  }

  // =========================
  // INICIALIZAR INA226
  // =========================

  if (!ina226.init()) {

    Serial.println("ERROR: No se encontro el INA226");

    while (1) {
      // Si el INA226 no existe, detenemos el Mega.
    }
  }

  // CJMCU-226
  // Shunt R010 = 0.01 ohm
  // Rango configurado hasta 10 A

  ina226.setResistorRange(0.01, 10.0);

  Serial.println("INA226 detectado correctamente");

  // =========================
  // RESTO DEL SETUP
  // =========================

  intervaloSensor = INTERVALO_SENSOR;
  intervaloControl = INTERVALO_CONTROL;
  intervaloSerial = calcularTSerial();

  deshabilitarActuadores();

  tiempoAnterior = millis();
}

// =========================
// LOOP
// =========================
void loop() {

  unsigned long TA = millis();
  // =========================
  // RECEPCIÓN SERIAL
  // =========================

  recibirConfiguracion();

  // =========================
  // LECTURA SENSORES
  // =========================
  if (TA - TSensor >= intervaloSensor) {

    TSensor = TA;

    actualizarVariables();

    actualizarEstadoSistema();

    mostrarSensoresSerial();


    if (estadoSistema == 0) {
      acumularPromedios();
    }
  }

  // =========================
  // CONTROL
  // =========================
  if (TA - TControl >= intervaloControl) {

    TControl = TA;

    ejecutarControl();
  }

  // =========================
  // ENVÍO SERIAL
  // =========================
  if (TA - TSerial >= intervaloSerial) {
    TSerial = TA;

    enviarDatos();
  }
}
