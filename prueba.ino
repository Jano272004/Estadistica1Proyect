#include <VarSpeedServo.h>
#include <EEPROM.h>
#include <Arduino.h>
#include <math.h>

// ---------------------- Configuración ----------------------
constexpr int NUM_SERVOS   = 4;
constexpr int CRC_DIVISOR  = 256;
constexpr unsigned long BAUD = 115200;

// Umbral y periodo mínimo para escribir en EEPROM (evitar desgaste)
constexpr float  RESISTANCE_EPSILON     = 0.05f;     // cambio mínimo significativo
constexpr unsigned long EEPROM_SAVE_MIN_PERIOD_MS = 5000; // periodo mínimo entre escrituras

struct ServoConfig {
  uint8_t pin;
  uint8_t minPos;
  uint8_t maxPos;
  uint8_t step;
  uint16_t delayMs;
  int eepromAddr;
};

static const ServoConfig servoConfigs[NUM_SERVOS] = {
  {11,  0,   180, 5, 50,  0},  // Base
  {10,  80,  160, 5, 50, 10},  // Hombro
  {9,   120, 180, 5, 50, 20},  // Codo
  {5,   0,   180, 5, 50, 30},  // Pinza
};

// ---------------------- Clase ServoManager ----------------------
class ServoManager {
public:
  VarSpeedServo servo;
  const ServoConfig& cfg;

  float resistanceFactor;       // factor actual (1.0 .. 2.0)
  float lastSavedResistance;    // último valor persistido en EEPROM
  unsigned long lastSaveMs;     // último instante de guardado en EEPROM

  bool  moving;
  int   targetPos;
  int   baseSpeed;
  unsigned long maxTime;
  unsigned long startTime;
  unsigned long lastTime;

  ServoManager(const ServoConfig& config)
    : cfg(config),
      resistanceFactor(1.0f),
      lastSavedResistance(1.0f),
      lastSaveMs(0),
      moving(false),
      targetPos(config.minPos),
      baseSpeed(0),
      maxTime(0),
      startTime(0),
      lastTime(0) {}

  void begin() {
    servo.attach(cfg.pin);
    loadResistance();
    int mid = (cfg.minPos + cfg.maxPos) / 2;
    moveBlocking(mid, 50, 500);
  }

  void loadResistance() {
    float stored;
    EEPROM.get(cfg.eepromAddr, stored);
    if (isnan(stored) || stored < 1.0f || stored > 2.0f) {
      stored = 1.0f;
    }
    resistanceFactor    = stored;
    lastSavedResistance = stored;
    lastSaveMs          = millis();
  }

  // Guarda en EEPROM sólo si el cambio es significativo y ha pasado tiempo mínimo
  void saveResistanceIfNeeded(bool force = false) {
    unsigned long now = millis();
    if (force) {
      EEPROM.put(cfg.eepromAddr, resistanceFactor);
      lastSavedResistance = resistanceFactor;
      lastSaveMs = now;
      return;
    }
    if ((fabsf(resistanceFactor - lastSavedResistance) >= RESISTANCE_EPSILON) &&
        (now - lastSaveMs >= EEPROM_SAVE_MIN_PERIOD_MS)) {
      EEPROM.put(cfg.eepromAddr, resistanceFactor);
      lastSavedResistance = resistanceFactor;
      lastSaveMs = now;
    }
  }

  void startMove(int target, int speed, unsigned long timeoutMs) {
    targetPos = constrain(target, cfg.minPos, cfg.maxPos);
    baseSpeed = speed;
    maxTime   = timeoutMs;
    startTime = millis();
    lastTime  = startTime;
    moving    = true;
  }

  void update() {
    if (!moving) return;

    unsigned long now = millis();
    if (now - lastTime < cfg.delayMs) return;
    lastTime = now;

    int pos = servo.read();
    if (pos == targetPos) {
      moving = false;
      // Al finalizar con éxito, intenta normalizar (hacia 1.0) y guardar si aplica
      resistanceFactor = fmaxf(resistanceFactor * 0.98f, 1.0f); // relajación suave
      saveResistanceIfNeeded(); // chequea umbral y periodo
      return;
    }

    int dir = (targetPos > pos) ? 1 : -1;
    int nextPos = pos + dir * cfg.step;
    if ((dir > 0 && nextPos > targetPos) || (dir < 0 && nextPos < targetPos)) {
      nextPos = targetPos;
    }

    int adjSpeed = constrain(int(baseSpeed * resistanceFactor), 10, 255);
    servo.write(nextPos, adjSpeed, true);

    // Timeout: si excede, aumentar resistencia y terminar este movimiento
    if (now - startTime > maxTime) {
      Serial.println("TIMEOUT_RESISTANCE");
      resistanceFactor = fminf(resistanceFactor * 1.10f, 2.0f);
      // Guardar inmediatamente pero respetando el mecanismo (forzar escritura opcional)
      saveResistanceIfNeeded(/*force=*/true);
    } else {
      // Si no hay timeout, podemos ajustar suavemente hacia 1.0 (salud del sistema)
      resistanceFactor = fmaxf(resistanceFactor * 0.98f, 1.0f);
      // Guardado diferido y con umbral/periodo
      saveResistanceIfNeeded();
    }
  }

  void moveBlocking(int target, int speed, unsigned long timeoutMs) {
    startMove(target, speed, timeoutMs);
    while (moving) {
      update();
      delay(1);
    }
  }
};

// ---------------------- Instancias ----------------------
ServoManager servos[NUM_SERVOS] = {
  ServoManager{servoConfigs[0]},
  ServoManager{servoConfigs[1]},
  ServoManager{servoConfigs[2]},
  ServoManager{servoConfigs[3]},
};

// ---------------------- CRC ----------------------
bool validateCRC(const String& data) {
  int sep = data.indexOf(';');
  if (sep < 0) return false;
  String payload = data.substring(0, sep);
  int recv = data.substring(sep + 1).toInt();
  long sum = 0;
  for (char c : payload) sum += c;
  return (sum % CRC_DIVISOR) == recv;
}

// ---------------------- Setup & Loop ----------------------
void setup() {
  Serial.begin(BAUD);
  for (int i = 0; i < NUM_SERVOS; ++i) {
    servos[i].begin();
  }
  Serial.println("Arduino listo para recibir comandos");
}

void processCommand(const String& line) {
  if (!validateCRC(line)) {
    Serial.println("ERROR_CRC");
    return;
  }

  String payload = line.substring(0, line.indexOf(';'));
  int angles[NUM_SERVOS];
  int idx = 0;

  char buf[payload.length() + 1];
  payload.toCharArray(buf, sizeof(buf));
  char* tok = strtok(buf, ",");
  while (tok && idx < NUM_SERVOS) {
    angles[idx++] = atoi(tok);
    tok = strtok(nullptr, ",");
  }

  if (idx != NUM_SERVOS) {
    Serial.println("ERROR_DATO");
    return;
  }

  // Movimiento no bloqueante en paralelo (la función espera a que terminen)
  for (int i = 0; i < NUM_SERVOS; ++i) {
    int current = servos[i].servo.read();  // Posición actual del servo
    int dist    = abs(current - angles[i]);

    // Calcular velocidad y timeout proporcional a la distancia
    int speed = constrain(map(dist, 0, 180, 100, 255), 100, 255);//valor de speed 100/255 proporcional a angulo 0-180
    unsigned long timeout = 400 + dist * 10; // base 400ms + 10ms por grado. variable proporcional a la distancia

    servos[i].startMove(angles[i], speed, timeout);
  }

  bool anyMoving;
  do {
    anyMoving = false;
    for (int i = 0; i < NUM_SERVOS; ++i) {
      servos[i].update();
      anyMoving |= servos[i].moving;
    }
  } while (anyMoving);

  // Limpiar buffer RX antes de enviar OK
  while (Serial.available()) Serial.read();

  Serial.println("OK");
  Serial.flush();//Bloquea programa hasta que buffer de transmision esté vacio (HASTA QUE SE ENVIE POR COMPLETO)
}

void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    processCommand(line);
  }
}