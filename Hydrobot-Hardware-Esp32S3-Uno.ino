// ====== HydroBot ESP32-S3 – BLE + AUTO + MANUAL ======
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

/* ---------------- BLUETOOTH BLE ---------------- */
// Nordic UART Service UUIDs (compatível com seu app)
#define SERVICE_UUID "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define CHARACTERISTIC_UUID_RX "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  // App -> ESP32
#define CHARACTERISTIC_UUID_TX "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  // ESP32 -> App

BLEServer *pServer = NULL;
BLECharacteristic *pTxCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;
String rxBuffer = "";

/* ---------------- PINOS ESP32-S3 ---------------- */
// Motores (ponte H) - GPIO disponíveis no ESP32-S3
// #define IN1 18
// #define IN2 17
// #define IN3 19
// #define IN4 20
#define IN1 10  // IO10 (lado direito, acima do meio)
#define IN2 11  // IO11 (lado direito)
#define IN3 13  // IO12 (lado direito)
#define IN4 12  // IO13 (lado direito)

// Sensores KY-026 (pinos ADC do ESP32-S3)
#define SENSOR_FOGO_ESQ 4   // GPIO4 (ADC1_CH3)
#define SENSOR_FOGO_MEIO 5  // GPIO5 (ADC1_CH4)
#define SENSOR_FOGO_DIR 6   // GPIO6 (ADC1_CH5)

// Bomba / Nível de água / LED
#define BOMBA_PIN 1       // PWM
#define NIVEL_AGUA_PIN 7  // GPIO7 (ADC1_CH6)
#define LED_NIVEL 8       // GPIO8
#define BOTAO 9           // GPIO9

/* ---------------- PARÂMETROS AUTO ---------------- */
static int DIFERENCA_FOGO = 200;  // Ajustado para ADC de 12 bits
static int DIFERENCA_CLEAR = 120;
static int INTENSIDADE_PERIGO = 1400;  // Ajustado para ADC de 12 bits
static int INTENSIDADE_IDEAL = 800;

static const uint8_t CONFIRM_READS = 2;
static const uint8_t CONFIRM_MIN = 2;
static const uint16_t CONFIRM_SPACING_MS = 10;
static const uint16_t FIRE_HOLD_MS = 200;
static const uint16_t TEMPO_CALIBRACAO_MS = 2000;

// MOVIMENTO
static uint8_t VEL_PERC = 60;
static const uint16_t T_FWD_MS = 300;
static const uint16_t T_TURN_MS = 200;
static const uint16_t T_BACK_MS = 350;
static const uint16_t KICK_RIGHT_MS = 80;  // ajusta depois se precisar

/* ---------------- PARÂMETROS PWM BOMBA ---------------- */
static uint8_t PWM_MIN = 180;
static uint8_t PWM_MAX = 255;
static const uint16_t PWM_RAMP_MS = 2000;

// Configuração PWM ESP32-S3 (API 3.x)
#define PWM_FREQ 5000
#define PWM_RESOLUTION 8

/* ---------------- ESTADO ---------------- */
enum Modo : uint8_t { MODO_MANUAL = 0,
                      MODO_AUTO = 1 };
enum Dir : uint8_t { DIR_STOP = 0,
                     DIR_FWD,
                     DIR_BACK,
                     DIR_LEFT,
                     DIR_RIGHT };

struct State {
  Modo modo = MODO_MANUAL;

  int baseEsq = 0, baseMeio = 0, baseDir = 0;
  bool calibrado = false;

  uint8_t nivelAguaPct = 0;
  uint8_t pwmBombaAtual = PWM_MIN;
  bool bombaLigada = false;
  unsigned long bombaInicioMs = 0;

  unsigned long ledTick = 0;
  bool ledState = false;

  bool fogoConfirmado = false;
  unsigned long fogoHoldUntil = 0;
  int intensidadeMaxAtual = 0;
  bool muitoPerto = false;
  unsigned long tempoEmCombate = 0;

  Dir currentDir = DIR_STOP;
  bool motionActive = false;
  unsigned long motionEndMs = 0;
  unsigned long lastMoveCmdAt = 0;

  bool lastBtn = HIGH;
  unsigned long lastBtnDebounce = 0;
  bool btnPressed = false;
} G;

unsigned long lastTelemMs = 0;

/* ---------------- CALLBACKS BLE ---------------- */
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    deviceConnected = true;
    Serial.println("✅ Cliente BLE conectado!");
  }

  void onDisconnect(BLEServer *pServer) {
    deviceConnected = false;
    Serial.println("❌ Cliente BLE desconectado!");
  }
};

class MyCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    String rxValue = pCharacteristic->getValue().c_str();
    void processCommand(String cmd);

    if (rxValue.length() > 0) {
      for (int i = 0; i < rxValue.length(); i++) {
        char c = rxValue[i];
        if (c == '\n' || c == '\r') {
          if (rxBuffer.length() > 0) {
            processCommand(rxBuffer);
            rxBuffer = "";
          }
        } else {
          rxBuffer += c;
        }
      }
    }
  }
};

void processCommand(String cmd);

/* ---------------- FUNÇÕES BLE ---------------- */
void bleSend(String msg) {
  if (deviceConnected && pTxCharacteristic != NULL) {
    msg += "\n";
    pTxCharacteristic->setValue(msg.c_str());
    pTxCharacteristic->notify();
  }
  Serial.println(msg);  // Debug serial também
}

void setupBLE() {
  Serial.println("🔷 Iniciando BLE...");

  BLEDevice::init("HydroBot");

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  // TX characteristic (ESP32 -> App)
  pTxCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID_TX,
    BLECharacteristic::PROPERTY_NOTIFY);
  pTxCharacteristic->addDescriptor(new BLE2902());

  // RX characteristic (App -> ESP32)
  BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID_RX,
    BLECharacteristic::PROPERTY_WRITE);
  pRxCharacteristic->setCallbacks(new MyCallbacks());

  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();

  Serial.println("✅ BLE pronto! Nome: HydroBot");
}

/* ---------------- PROTÓTIPOS ---------------- */
void motorsStop();
void motorsFwd();
void motorsBack();
void motorsLeftTurn();
void motorsRightTurn();
void motorsFwdBasic();
void motorsBackBasic();
void motorsLeftTurnBasic();
void motorsRightTurnBasic();

void pwmSoftware(void (*mov)(), int velPerc, int duracaoMs);
void pumpPWM(uint8_t pwmValue);
void pumpOff();
uint8_t waterPercent();
void updateWaterLED(uint8_t pct);

void calibrarSensores();
void leituraDelta(int &dE, int &dM, int &dD, int &side, int &maxIntensity);
bool detectarFogoConfirmado(int &sideOut, int &intensityOut);
bool isClearNow();

void handleAuto();
void handleManual();
void sendTelemetry();

/* ---------------- PARÂMETROS DE KICK ---------------- */
// Adicione junto com os outros parâmetros no topo do código
static uint16_t KICK_RIGHT_FWD_MS = 80;   // Kick para motor direito FRENTE
static uint16_t KICK_RIGHT_BACK_MS = 150; // Kick para motor direito RÉ (precisa mais tempo!)

/* ---------------- MOTORES COM KICK COMPENSADO ---------------- */

// Funções básicas (sem kick)
void motorsFwdBasic(){  
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void motorsBackBasic(){ 
  digitalWrite(IN1, LOW); 
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); 
  digitalWrite(IN4, HIGH);
}

void motorsLeftTurnBasic(){  
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH); 
  digitalWrite(IN3, HIGH); 
  digitalWrite(IN4, LOW);
}

void motorsRightTurnBasic(){ 
  digitalWrite(IN1, HIGH); 
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void motorsStop(){ 
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW); 
  G.currentDir = DIR_STOP; 
  G.motionActive = false;
}

void motorsFwd(){  
  // Motor direito precisa ir FRENTE
  // 1. Kick no motor direito para frente
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);  // Motor direito FRENTE
  digitalWrite(IN4, LOW);
  delay(KICK_RIGHT_FWD_MS);
  
  // 2. Aplica movimento completo
  motorsFwdBasic();
  
  G.currentDir = DIR_FWD; 
  G.lastMoveCmdAt = millis();
}

void motorsBack(){ 
  // Motor direito precisa ir RÉ (PROBLEMA AQUI!)
  // 1. Kick MAIS FORTE no motor direito para trás
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);   // Motor direito RÉ
  digitalWrite(IN4, HIGH);
  delay(KICK_RIGHT_BACK_MS); // Tempo MAIOR para vencer resistência
  
  // 2. Aplica movimento completo SEM parar
  motorsBackBasic();
  
  G.currentDir = DIR_BACK; 
  G.lastMoveCmdAt = millis();
}

void motorsLeftTurn(){  
  // Motor direito vai FRENTE no giro à esquerda
  // 1. Kick no motor direito para frente
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);  // Motor direito FRENTE
  digitalWrite(IN4, LOW);
  delay(KICK_RIGHT_FWD_MS);
  
  // 2. Aplica o giro completo
  motorsLeftTurnBasic();
  
  G.currentDir = DIR_LEFT; 
  G.lastMoveCmdAt = millis();
}

void motorsRightTurn(){ 
  // Motor direito vai RÉ no giro à direita (PROBLEMA AQUI!)
  // 1. Kick MAIS FORTE no motor direito para trás
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);   // Motor direito RÉ
  digitalWrite(IN4, HIGH);
  delay(KICK_RIGHT_BACK_MS); // Tempo MAIOR
  
  // 2. Aplica o giro completo SEM parar
  motorsRightTurnBasic();
  
  G.currentDir = DIR_RIGHT; 
  G.lastMoveCmdAt = millis();
}

/* ---------------- COMANDOS PARA DIAGNÓSTICO E AJUSTE ---------------- */
// Adicione dentro da função processCommand()

/*
  // Ajustar tempo do kick para frente
  if (cmd.startsWith("SET_KICK_FWD:")) {
    int kick = cmd.substring(13).toInt();
    KICK_RIGHT_FWD_MS = constrain(kick, 30, 300);
    bleSend("KICK_FWD_SET:" + String(KICK_RIGHT_FWD_MS));
    return;
  }
  
  // Ajustar tempo do kick para trás (IMPORTANTE!)
  if (cmd.startsWith("SET_KICK_BACK:")) {
    int kick = cmd.substring(14).toInt();
    KICK_RIGHT_BACK_MS = constrain(kick, 30, 300);
    bleSend("KICK_BACK_SET:" + String(KICK_RIGHT_BACK_MS));
    return;
  }
  
  // Teste: motor direito para FRENTE
  else if (cmd == "TEST_RIGHT_FWD") {
    bleSend("TEST:Right motor FORWARD only");
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    delay(2000);
    motorsStop();
    bleSend("OK:TEST_DONE");
  }
  
  // Teste: motor direito para TRÁS
  else if (cmd == "TEST_RIGHT_BACK") {
    bleSend("TEST:Right motor BACKWARD only");
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    delay(2000);
    motorsStop();
    bleSend("OK:TEST_DONE");
  }
  
  // Teste: motor esquerdo para FRENTE
  else if (cmd == "TEST_LEFT_FWD") {
    bleSend("TEST:Left motor FORWARD only");
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    delay(2000);
    motorsStop();
    bleSend("OK:TEST_DONE");
  }
  
  // Teste: motor esquerdo para TRÁS
  else if (cmd == "TEST_LEFT_BACK") {
    bleSend("TEST:Left motor BACKWARD only");
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    delay(2000);
    motorsStop();
    bleSend("OK:TEST_DONE");
  }
*/

void pwmSoftware(void (*mov)(), int vel, int duracao) {
  unsigned long inicio = millis();

  if (vel >= 100) {
    G.motionActive = true;
    G.motionEndMs = millis() + duracao;
    mov();
    delay(duracao);
    G.motionActive = false;
    return;
  }

  const int ciclo = 20;
  const int onMs = (ciclo * vel) / 100;
  const int offMs = ciclo - onMs;

  G.motionActive = true;
  G.motionEndMs = millis() + duracao;

  while ((int)(millis() - inicio) < duracao) {
    if (!G.motionActive) break;
    mov();
    delay(onMs);
    if (offMs > 0) {
      motorsStop();
      delay(offMs);
    }
  }
  G.motionActive = false;
}

/* ---------------- BOMBA ---------------- */
void pumpPWM(uint8_t pwmValue) {
  ledcWrite(BOMBA_PIN, pwmValue);
  G.pwmBombaAtual = pwmValue;
  G.bombaLigada = (pwmValue > 0);

  if (pwmValue > 0 && G.bombaInicioMs == 0) {
    G.bombaInicioMs = millis();
  } else if (pwmValue == 0) {
    G.bombaInicioMs = 0;
  }
}

void pumpOff() {
  ledcWrite(BOMBA_PIN, 0);
  G.bombaLigada = false;
  G.bombaInicioMs = 0;
  G.pwmBombaAtual = PWM_MIN;
}

void updatePumpPWM() {
  if (!G.bombaLigada || G.bombaInicioMs == 0) return;

  unsigned long tempoDecorrido = millis() - G.bombaInicioMs;

  if (tempoDecorrido >= PWM_RAMP_MS) {
    pumpPWM(PWM_MAX);
  } else {
    uint8_t pwm = map(tempoDecorrido, 0, PWM_RAMP_MS, PWM_MIN, PWM_MAX);
    pumpPWM(pwm);
  }
}

/* ---------------- ÁGUA / LED ---------------- */
uint8_t waterPercent() {
  int raw = analogRead(NIVEL_AGUA_PIN);
  // ESP32-S3 tem ADC de 12 bits (0-4095)
  int pct = map(raw, 0, 4095, 0, 100);
  return (uint8_t)constrain(pct, 0, 100);
}

void updateWaterLED(uint8_t pct) {
  unsigned long now = millis();
  if (pct >= 60) {
    if (G.ledState) {
      G.ledState = false;
      digitalWrite(LED_NIVEL, LOW);
    }
    return;
  }
  if (pct >= 40) {
    if (!G.ledState) {
      G.ledState = true;
      digitalWrite(LED_NIVEL, HIGH);
    }
    return;
  }
  uint16_t periodo = (pct >= 20) ? 800 : (pct >= 10 ? 400 : 200);
  if (now - G.ledTick >= periodo) {
    G.ledTick = now;
    G.ledState = !G.ledState;
    digitalWrite(LED_NIVEL, G.ledState ? HIGH : LOW);
  }
}

/* ---------------- SENSORES ---------------- */
void calibrarSensores() {
  bleSend("CAL_START");
  Serial.println("⚙️ Calibrando sensores...");

  unsigned long t0 = millis();
  long sE = 0, sM = 0, sD = 0;
  int n = 0;

  while (millis() - t0 < TEMPO_CALIBRACAO_MS) {
    sE += analogRead(SENSOR_FOGO_ESQ);
    sM += analogRead(SENSOR_FOGO_MEIO);
    sD += analogRead(SENSOR_FOGO_DIR);
    n++;
    delay(100);
  }

  if (n == 0) n = 1;
  G.baseEsq = sE / n;
  G.baseMeio = sM / n;
  G.baseDir = sD / n;
  G.calibrado = true;

  String calMsg = "CAL_DONE:" + String(G.baseEsq) + "," + String(G.baseMeio) + "," + String(G.baseDir);
  bleSend(calMsg);

  Serial.println("✅ Calibração concluída!");
}

void leituraDelta(int &dE, int &dM, int &dD, int &side, int &maxIntensity) {
  int aE = analogRead(SENSOR_FOGO_ESQ);
  int aM = analogRead(SENSOR_FOGO_MEIO);
  int aD = analogRead(SENSOR_FOGO_DIR);

  dE = G.baseEsq - aE;
  dM = G.baseMeio - aM;
  dD = G.baseDir - aD;

  maxIntensity = max(dM, max(dE, dD));

  side = 0;
  if (dM >= dE && dM >= dD) side = 0;
  else if (dE > dD) side = -1;
  else side = +1;
}

bool detectarFogoConfirmado(int &sideOut, int &intensityOut) {
  uint8_t hits = 0;
  int bestDelta = -32768;
  int sideBest = 0;

  for (uint8_t i = 0; i < CONFIRM_READS; i++) {
    int dE, dM, dD, side, maxInt;
    leituraDelta(dE, dM, dD, side, maxInt);

    bool fogo = (dE > DIFERENCA_FOGO) || (dM > DIFERENCA_FOGO) || (dD > DIFERENCA_FOGO);
    if (fogo) hits++;

    if (maxInt > bestDelta) {
      bestDelta = maxInt;
      sideBest = side;
    }

    delay(CONFIRM_SPACING_MS);
  }

  sideOut = sideBest;
  intensityOut = bestDelta;
  return (hits >= CONFIRM_MIN);
}

bool isClearNow() {
  int dE, dM, dD, sideDummy, maxInt;
  leituraDelta(dE, dM, dD, sideDummy, maxInt);
  return (dE < DIFERENCA_CLEAR) && (dM < DIFERENCA_CLEAR) && (dD < DIFERENCA_CLEAR);
}

/* ---------------- MODO AUTOMÁTICO ---------------- */
void handleAuto() {
  if (!G.calibrado) {
    calibrarSensores();
    return;
  }

  G.nivelAguaPct = waterPercent();
  updateWaterLED(G.nivelAguaPct);

  if (G.nivelAguaPct <= 10) {
    pumpOff();
  }

  int side = 0;
  int intensity = 0;
  bool fireNow = detectarFogoConfirmado(side, intensity);
  unsigned long now = millis();

  G.intensidadeMaxAtual = intensity;

  if (fireNow) {
    G.fogoConfirmado = true;
    G.fogoHoldUntil = now + FIRE_HOLD_MS;

    if (intensity > INTENSIDADE_PERIGO) {
      G.muitoPerto = true;
      bleSend("FIRE_TOO_CLOSE");

      pumpOff();
      pwmSoftware(motorsBackBasic, VEL_PERC, T_BACK_MS);
      motorsStop();
      delay(300);

    } else if (intensity >= INTENSIDADE_IDEAL) {
      G.muitoPerto = false;
      bleSend("FIRE_FIGHTING");

      if (G.nivelAguaPct > 10) {
        if (!G.bombaLigada) {
          pumpPWM(PWM_MIN);
        }
        updatePumpPWM();
      }
      motorsStop();

    } else {
      G.muitoPerto = false;
      bleSend("FIRE_APPROACHING");

      if (G.nivelAguaPct > 10) {
        pumpPWM(PWM_MIN);
      }

      if (side < 0) {
        pwmSoftware(motorsLeftTurnBasic, VEL_PERC, T_TURN_MS);
        pwmSoftware(motorsFwdBasic, VEL_PERC, T_FWD_MS);
      } else if (side > 0) {
        pwmSoftware(motorsRightTurnBasic, VEL_PERC, T_TURN_MS);
        pwmSoftware(motorsFwdBasic, VEL_PERC, T_FWD_MS);
      } else {
        pwmSoftware(motorsFwdBasic, VEL_PERC, T_FWD_MS);
      }
    }

    G.tempoEmCombate = now;

  } else {
    if (G.fogoConfirmado && now > G.fogoHoldUntil) {
      if (isClearNow()) {
        bleSend("FIRE_OUT");
        G.fogoConfirmado = false;
        G.muitoPerto = false;
      }
    }

    if (!G.fogoConfirmado) {
      pumpOff();
      motorsStop();
    } else if (now - G.tempoEmCombate > 3000) {
      bleSend("FIRE_LOST");
      pumpOff();
      pwmSoftware(motorsBackBasic, VEL_PERC, T_BACK_MS);
      G.fogoConfirmado = false;
    }
  }

  if (G.bombaLigada) {
    updatePumpPWM();
  }
}

/* ---------------- MODO MANUAL ---------------- */
void handleManual() {
  G.nivelAguaPct = waterPercent();
  updateWaterLED(G.nivelAguaPct);

  if (G.bombaLigada) {
    updatePumpPWM();
  }
}

void processCommand(String cmd) {
  cmd.trim();

  Serial.println("📩 Comando: " + cmd);

  // ===== COMANDOS DE CONFIGURAÇÃO =====
  if (cmd.startsWith("SET_SPEED:")) {
    int vel = cmd.substring(10).toInt();
    VEL_PERC = constrain(vel, 30, 100);
    bleSend("SPEED_SET:" + String(VEL_PERC));
    return;
  }

  if (cmd.startsWith("SET_PWM_MIN:")) {
    int pwm = cmd.substring(12).toInt();
    PWM_MIN = constrain(pwm, 150, 255);
    bleSend("PWM_MIN_SET:" + String(PWM_MIN));
    return;
  }

  if (cmd.startsWith("SET_PWM_MAX:")) {
    int pwm = cmd.substring(12).toInt();
    PWM_MAX = constrain(pwm, 180, 255);
    bleSend("PWM_MAX_SET:" + String(PWM_MAX));
    return;
  }

  if (cmd.startsWith("SET_FIRE_THRESH:")) {
    int thresh = cmd.substring(16).toInt();
    DIFERENCA_FOGO = constrain(thresh, 80, 800);
    bleSend("FIRE_THRESH_SET:" + String(DIFERENCA_FOGO));
    return;
  }

  if (cmd.startsWith("SET_FIRE_DANGER:")) {
    int danger = cmd.substring(16).toInt();
    INTENSIDADE_PERIGO = constrain(danger, 800, 2400);
    bleSend("FIRE_DANGER_SET:" + String(INTENSIDADE_PERIGO));
    return;
  }

  if (cmd.startsWith("SET_FIRE_IDEAL:")) {
    int ideal = cmd.substring(15).toInt();
    INTENSIDADE_IDEAL = constrain(ideal, 400, 1600);
    bleSend("FIRE_IDEAL_SET:" + String(INTENSIDADE_IDEAL));
    return;
  }

  // ===== COMANDOS DE CONTROLE =====
  if (cmd == "FWD") {
    motorsFwd();
    bleSend("OK:FWD");
  } else if (cmd == "BACK") {
    motorsBack();
    bleSend("OK:BACK");
  } else if (cmd == "LEFT") {
    motorsLeftTurn();
    bleSend("OK:LEFT");
  } else if (cmd == "RIGHT") {
    motorsRightTurn();
    bleSend("OK:RIGHT");
  } else if (cmd == "STOP") {
    motorsStop();
    bleSend("OK:STOP");
  } else if (cmd == "PUMP_ON") {
    if (G.nivelAguaPct > 10) {
      pumpPWM(PWM_MIN);
      bleSend("OK:PUMP_ON");
    } else {
      bleSend("ERR:LOW_WATER");
    }
  } else if (cmd == "PUMP_OFF") {
    pumpOff();
    bleSend("OK:PUMP_OFF");
  } else if (cmd == "MODE_AUTO") {
    G.modo = MODO_AUTO;
    G.calibrado = false;
    motorsStop();
    pumpOff();
    bleSend("OK:MODE_AUTO");
  } else if (cmd == "MODE_MANUAL") {
    G.modo = MODO_MANUAL;
    motorsStop();
    pumpOff();
    bleSend("OK:MODE_MANUAL");
  } else if (cmd == "CALIBRATE") {
    G.calibrado = false;
    bleSend("OK:CALIBRATING");
  } else if (cmd == "GET_STATUS") {
    sendTelemetry();
  }
}

void sendTelemetry() {
  String json = "{";
  json += "\"water\":" + String(G.nivelAguaPct);
  json += ",\"pump\":" + String(G.bombaLigada ? G.pwmBombaAtual : 0);
  json += ",\"intensity\":" + String(G.intensidadeMaxAtual);
  json += ",\"mode\":\"" + String(G.modo == MODO_AUTO ? "AUTO" : "MANUAL") + "\"";
  json += ",\"fire\":" + String(G.fogoConfirmado ? "true" : "false");
  json += ",\"speed\":" + String(VEL_PERC);
  json += ",\"pwm_min\":" + String(PWM_MIN);
  json += ",\"pwm_max\":" + String(PWM_MAX);
  json += "}";

  bleSend(json);
}

/* ---------------- SETUP / LOOP ---------------- */
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n🔥 HydroBot ESP32-S3 Starting...");

  // Configurar resolução ADC para 12 bits
  analogReadResolution(12);

  // Configurar PWM para bomba (API atualizada ESP32-S3)
  ledcAttach(BOMBA_PIN, PWM_FREQ, PWM_RESOLUTION);

  // Motores
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  motorsStop();

  // Bomba e LED
  pumpOff();
  pinMode(LED_NIVEL, OUTPUT);
  digitalWrite(LED_NIVEL, LOW);

  // Botão
  pinMode(BOTAO, INPUT_PULLUP);

  // Iniciar BLE
  setupBLE();

  Serial.println("✅ HydroBot pronto! Aguardando conexão BLE...");
}

void loop() {
  // Gerenciar conexão BLE
  if (!deviceConnected && oldDeviceConnected) {
    delay(500);
    pServer->startAdvertising();
    Serial.println("🔄 Reiniciando advertising...");
    oldDeviceConnected = deviceConnected;
  }

  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
    bleSend("READY");
  }

  // Watchdog de movimento
  if (G.motionActive && (long)(millis() - G.motionEndMs) >= 0) {
    motorsStop();
  }

  // Watchdog de segurança
  unsigned long timeoutMs = (G.modo == MODO_AUTO) ? 10000 : 5000;
  if ((millis() - G.lastMoveCmdAt) > timeoutMs && G.currentDir != DIR_STOP) {
    motorsStop();
  }

  // Debounce do botão
  bool btn = digitalRead(BOTAO);
  unsigned long now = millis();

  if (btn != G.lastBtn) {
    G.lastBtnDebounce = now;
  }

  if ((now - G.lastBtnDebounce) > 50) {
    if (btn != G.btnPressed) {
      G.btnPressed = btn;

      if (btn == LOW) {
        G.modo = (G.modo == MODO_AUTO) ? MODO_MANUAL : MODO_AUTO;

        if (G.modo == MODO_AUTO) {
          G.calibrado = false;
          bleSend("MODE:AUTO");
        } else {
          bleSend("MODE:MANUAL");
        }

        motorsStop();
        pumpOff();
      }
    }
  }

  G.lastBtn = btn;

  // Executar modo atual
  if (G.modo == MODO_AUTO) {
    handleAuto();
  } else {
    handleManual();
  }

  // Telemetria automática a cada 500ms (apenas se conectado)
  if (deviceConnected && (millis() - lastTelemMs >= 500)) {
    lastTelemMs = millis();
    sendTelemetry();
  }

  delay(10);
}