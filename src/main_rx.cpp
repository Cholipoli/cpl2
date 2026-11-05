// Récepteur CPL (environnement PlatformIO « receiver »), utilisé avec interface_cpl.py.
// Lecture ANALOGIQUE de A0 et détecteur d'enveloppe logiciel.
// (L'ancienne version qui lisait D2 en 0/1 est conservée dans main_rx_d2.cpp.)
//
// Montage : sortie du double passe-bande -> 100 nF -> A0, avec A0 polarisé
// à 1,65 V par deux 10 kΩ (vers 3,3 V et vers GND). La porteuse 100 kHz arrive
// donc centrée sur 1,65 V, entre 0 et 3,3 V.
//
// Principe :
//  1. L'ADC de la Due mesure A0 en continu (~1 million de mesures/s) et la DMA
//     range les mesures dans des tampons, sans occuper le processeur.
//  2. Pour chaque mesure : on retire le centre (moyenne lente), on prend la
//     valeur absolue, puis un "détecteur de crête" logiciel (montée immédiate,
//     descente exponentielle) donne l'ENVELOPPE de la porteuse.
//  3. Enveloppe > seuil = porteuse présente = bit 1. Le seuil s'adapte tout
//     seul : à mi-chemin entre la crête de l'enveloppe et son niveau au repos.
//  4. Décodage UART comme main_rx : front montant = début du bit de départ,
//     puis lecture au milieu de chaque bit (départ, 8 bits LSB d'abord, fin).
//  5. Trames F0 / type / métadonnées / données / 0F décodées comme avant.
//
// Commandes (moniteur série 115200, terminer par Entrée) :
//   SET_BAUD:10000   (1000 à 50000)    choisit le débit
//   STATS            bascule l'affichage d'une ligne de statistiques par seconde
//                    (conseillé pour les tests en boucle : moins de texte envoyé)
//   DEBUG            bascule l'affichage, 2 fois par seconde, de ce que la Due
//                    voit sur A0 : tension min / max / centre, enveloppe, seuil,
//                    et nombre de fois où la porteuse a été détectée
#include <Arduino.h>
#include <stdlib.h>
#include <string.h>

namespace {

// ---------------------------------------------------------------- réglages
constexpr uint32_t DEFAULT_BIT_RATE = 10000;
constexpr uint32_t MIN_BIT_RATE = 1000;
constexpr uint32_t MAX_BIT_RATE = 50000;
constexpr float SAMPLE_POSITION = 0.5f;   // lecture au milieu du bit
constexpr int32_t SCALE = 16;             // calcul en 1/16 de pas ADC
constexpr int32_t TH_MIN = 100 * SCALE;   // seuil minimal ≈ 80 mV
constexpr int32_t NOISE_FACTOR_X2 = 5;    // seuil >= 2,5 x niveau de bruit

// ---------------------------------------------------------------- ADC + DMA
constexpr uint8_t NB_BUFFERS = 8;
constexpr uint16_t BUFFER_SIZE = 1024;    // ≈ 1 ms par tampon
uint16_t adcBuffer[NB_BUFFERS][BUFFER_SIZE];
volatile uint32_t buffersDone = 0;        // tampons remplis par la DMA
uint32_t buffersProcessed = 0;
uint32_t buffersLost = 0;
float sampleRate = 1.0e6f;                // mesurée au démarrage

}  // namespace

// Interruption de fin de tampon : on donne à la DMA le tampon suivant.
extern "C" void ADC_Handler(void) {
  const uint32_t status = ADC->ADC_ISR;
  if (status & ADC_ISR_ENDRX) {
    ++buffersDone;
    ADC->ADC_RNPR = reinterpret_cast<uint32_t>(adcBuffer[(buffersDone + 1) % NB_BUFFERS]);
    ADC->ADC_RNCR = BUFFER_SIZE;
  }
}

namespace {

void startAdc() {
  pmc_enable_periph_clk(ID_ADC);
  adc_init(ADC, SystemCoreClock, ADC_FREQ_MAX, ADC_STARTUP_FAST);
  ADC->ADC_MR |= ADC_MR_FREERUN_ON;       // conversions en continu
  ADC->ADC_CHDR = 0xFFFFFFFF;
  ADC->ADC_CHER = 1U << 7;                // A0 = canal 7 de l'ADC
  ADC->ADC_IDR = 0xFFFFFFFF;
  ADC->ADC_IER = ADC_IER_ENDRX;
  ADC->ADC_RPR = reinterpret_cast<uint32_t>(adcBuffer[0]);
  ADC->ADC_RCR = BUFFER_SIZE;
  ADC->ADC_RNPR = reinterpret_cast<uint32_t>(adcBuffer[1]);
  ADC->ADC_RNCR = BUFFER_SIZE;
  NVIC_EnableIRQ(ADC_IRQn);
  ADC->ADC_PTCR = ADC_PTCR_RXTEN;         // DMA en marche
  ADC->ADC_CR = ADC_CR_START;
}

void measureSampleRate() {
  const uint32_t b0 = buffersDone;
  const uint32_t t0 = micros();
  delay(200);
  const uint32_t b1 = buffersDone;
  const uint32_t t1 = micros();
  if (b1 > b0) {
    sampleRate = static_cast<float>(b1 - b0) * BUFFER_SIZE * 1.0e6f / static_cast<float>(t1 - t0);
  }
  buffersProcessed = buffersDone;         // on repart des tampons à venir
}

// ---------------------------------------------------------------- démodulation
uint32_t bitRate = DEFAULT_BIT_RATE;
uint32_t bitOffsets[10];                  // instants de lecture des 10 bits
uint8_t decayShift = 4;                   // constante de temps de l'enveloppe
uint32_t quarterBit = 0;                  // quart de bit, en mesures
uint32_t tenBits = 0;                     // durée d'une trame (10 bits), en mesures
bool prevBlockClean = false;              // bloc de 32 mesures précédent au repos ?
uint32_t lowCount = 0;                    // mesures "basses" pendant le bit de départ

int32_t mid = 2048 * SCALE;               // centre du signal (≈ 1,65 V)
int32_t env = 0;                          // enveloppe
int32_t noise = 0;                        // plancher de l'enveloppe (silences)
int32_t ampOn = 0;                        // crête de l'enveloppe (porteuse)
int32_t bufferMax = 0;                    // max / min de l'enveloppe sur le tampon
int32_t bufferMin = 0x7FFFFFFF;            // bruit : plus petit des maxima par bloc de 32 mesures
int32_t thHigh = TH_MIN;
int32_t thLow = TH_MIN * 3 / 4;
bool level = false;
bool prevLevel = false;
uint32_t sampleIndex = 0;

enum class BitStage : uint8_t { Idle, Frame, Stop };
BitStage bitStage = BitStage::Idle;
uint32_t frameStart = 0;
uint32_t nextSampleAt = 0;
uint8_t bitIndex = 0;
uint8_t currentByte = 0;

// statistiques
uint32_t statBytes = 0;
uint32_t statFraming = 0;
uint32_t statFramesOk = 0;
uint32_t statFramesErr = 0;
uint32_t statDropped = 0;
bool statsMode = false;
uint32_t lastStats = 0;

// débogage : ce que la Due voit réellement sur A0
bool debugMode = false;
uint32_t lastDebug = 0;
uint16_t dbgMin = 4095;
uint16_t dbgMax = 0;
int32_t dbgEnvMax = 0;
uint32_t dbgEdges = 0;

void configureDemodulator() {
  const float samplesPerBit = sampleRate / static_cast<float>(bitRate);
  for (uint8_t k = 0; k < 10; ++k) {
    bitOffsets[k] = static_cast<uint32_t>((k + SAMPLE_POSITION) * samplesPerBit + 0.5f);
  }
  quarterBit = static_cast<uint32_t>(samplesPerBit / 4.0f);
  tenBits = static_cast<uint32_t>(10.0f * samplesPerBit + 0.5f);
  // Constante de temps ≈ moitié d'un bit, entre 14 et 32 µs. En dessous de
  // ~12 µs, l'enveloppe retombe entre deux crêtes de la porteuse (une crête
  // toutes les 5 µs) et des "1" sont lus "0" à 50 kbit/s.
  float tau = 0.5f / static_cast<float>(bitRate);
  if (tau < 14e-6f) tau = 14e-6f;
  if (tau > 32e-6f) tau = 32e-6f;
  const float tauSamples = tau * sampleRate;
  decayShift = 1;
  while ((1UL << (decayShift + 1)) <= tauSamples && decayShift < 8) {
    ++decayShift;
  }
  bitStage = BitStage::Idle;
}

// Seuil adaptatif, recalculé à chaque tampon (≈ 1 ms), indépendamment du
// décodage : à mi-chemin entre la crête de l'enveloppe (porteuse) et son
// plancher (silences). La crête redescend lentement (≈ 30 ms) pour suivre
// un signal qui devient plus faible.
void computeThresholds() {
  int32_t th = (ampOn + noise) / 2;
  if (th < NOISE_FACTOR_X2 * noise / 2) th = NOISE_FACTOR_X2 * noise / 2;
  if (th < TH_MIN) th = TH_MIN;
  thHigh = th;
  thLow = th - th / 4;                     // hystérésis
}

// Niveau de bruit : pour chaque bloc de 32 mesures (~50 µs) on prend le maximum
// de l'enveloppe, et on garde le plus petit de ces maxima. Un bloc tombé dans un
// silence donne ainsi la hauteur typique des pointes de bruit.
// Il descend tout de suite mais ne remonte que lentement (≈ 60 ms) : une longue
// suite de "1" (bits de 1 ms à 1 kbit/s) n'est pas prise pour du bruit.
void updateThreshold() {
  if (bufferMax > ampOn) {
    ampOn = bufferMax;
  } else {
    ampOn -= ampOn >> 7;                   // ≈ 130 ms
  }
  if (bufferMin != 0x7FFFFFFF) {           // au moins un bloc mesuré au repos
    if (bufferMin < noise) {
      noise = bufferMin;
    } else {
      noise += (bufferMin - noise) >> 6;
    }
  }
  bufferMax = 0;
  bufferMin = 0x7FFFFFFF;
  computeThresholds();
}

// ---------------------------------------------------------------- protocole
constexpr uint16_t RX_BUFFER_SIZE = 512;
constexpr uint16_t RX_BUFFER_MASK = RX_BUFFER_SIZE - 1;
// Le cahier des charges code lignes et colonnes sur 2 octets sans fixer de
// maximum ; 4096 garde le calcul de taille sur 32 bits et couvre tout cas réaliste.
constexpr uint16_t MAX_IMAGE_DIMENSION = 4096;
constexpr uint8_t SOT = 0xF0;
constexpr uint8_t EOT = 0x0F;
constexpr uint8_t TYPE_TEXT = 0x8F;
constexpr uint8_t TYPE_BW = 0xF8;
constexpr uint8_t TYPE_256 = 0x55;

uint8_t rxBuffer[RX_BUFFER_SIZE];
uint16_t rxHead = 0;
uint16_t rxTail = 0;

void enqueueReceivedByte(uint8_t value) {
  const uint16_t next = (rxHead + 1) & RX_BUFFER_MASK;
  if (next != rxTail) {
    rxBuffer[rxHead] = value;
    rxHead = next;
  }
  ++statBytes;
}

// Traitement d'un tampon complet. Toutes les variables d'état sont copiées
// dans des variables locales (registres du processeur) pendant la boucle :
// c'est nettement plus rapide que de relire / réécrire la mémoire à chaque
// mesure, ce qui compte à ~665 000 mesures par seconde.
void processBuffer(const uint16_t *buf) {
  int32_t lMid = mid, lEnv = env, lThH = thHigh, lThL = thLow;
  int32_t lBufMax = bufferMax, lBufMin = bufferMin, lDbgEnvMax = dbgEnvMax;
  uint16_t lDbgMin = dbgMin, lDbgMax = dbgMax;
  uint32_t lEdges = dbgEdges;
  bool lLevel = level, lPrev = prevLevel;
  uint32_t n = sampleIndex;
  BitStage lStage = bitStage;
  uint32_t lNext = nextSampleAt, lStart = frameStart;
  uint8_t lBit = bitIndex, lByte = currentByte;
  uint32_t lLow = lowCount;
  const uint8_t shift = decayShift;
  const uint32_t quarter = quarterBit;
  const uint32_t frameLen = tenBits;
  int32_t blkMax = 0;
  uint8_t blk = 0;
  bool blkDirty = false;
  bool lPrevClean = prevBlockClean;

  for (uint16_t i = 0; i < BUFFER_SIZE; ++i) {
    const uint16_t raw12 = buf[i] & 0x0FFF;
    if (raw12 < lDbgMin) lDbgMin = raw12;
    if (raw12 > lDbgMax) lDbgMax = raw12;
    const int32_t x = static_cast<int32_t>(raw12) * SCALE;
    lMid += (x - lMid) >> 12;                // centre : moyenne sur ≈ 6 ms
    int32_t a = x - lMid;
    if (a < 0) a = -a;
    lEnv = (a > lEnv) ? a : lEnv - (lEnv >> shift);
    if (lEnv > lBufMax) lBufMax = lEnv;
    if (lEnv > blkMax) blkMax = lEnv;
    // Le bruit n'est mesuré que dans des blocs entièrement au repos (pas de
    // réception en cours, niveau bas), et qui suivent un autre bloc au repos :
    // on évite ainsi la traîne de l'enveloppe et les longues suites de porteuse
    // d'une image (octets 0xFF), qui faisaient monter le seuil.
    if (lStage != BitStage::Idle || lLevel) blkDirty = true;
    if (++blk == 32) {
      if (!blkDirty && lPrevClean && blkMax < lBufMin) lBufMin = blkMax;
      lPrevClean = !blkDirty;
      blk = 0;
      blkMax = 0;
      blkDirty = false;
    }

    if (lLevel) {
      if (lEnv < lThL) lLevel = false;
    } else if (lEnv > lThH) {
      lLevel = true;
    }
    ++n;

    if (lStage == BitStage::Stop) {
      // Bit de fin (0) attendu : on accepte l'octet dès que l'enveloppe retombe.
      // Si elle ne retombe pas (traîne de la porteuse quand le dernier bit de
      // données vaut 1, fréquent dans une image), l'octet suivant commence à
      // l'instant prévu, 10 bits après le début du précédent.
      if (!lLevel) {
        enqueueReceivedByte(lByte);
        lStage = BitStage::Idle;
      } else if (n - lStart >= frameLen) {
        enqueueReceivedByte(lByte);
        lStage = BitStage::Frame;
        lStart += frameLen;
        lBit = 0;
        lByte = 0;
        lLow = 0;
        lNext = lStart + bitOffsets[0];
      }
    } else if (lStage == BitStage::Idle) {
      if (lLevel && !lPrev) {                // début du bit de départ
        ++lEdges;
        lStage = BitStage::Frame;
        lStart = n;
        lBit = 0;
        lByte = 0;
        lLow = 0;
        lNext = lStart + bitOffsets[0];
      }
    } else if (lBit == 0 && n != lNext && !lLevel) {
      // Pendant le bit de départ, l'enveloppe ne doit pas retomber plus d'un
      // quart de bit d'affilée ; sinon c'était un parasite : on se remet à l'affût.
      if (++lLow > quarter) lStage = BitStage::Idle;
    } else if (n == lNext) {
      const bool bit = lLevel;
      if (lBit == 0) {
        if (!bit) {
          lStage = BitStage::Idle;           // pas de porteuse au milieu : faux départ
        } else if (lEnv > ampOn) {
          // Premier message après un long silence : on cale tout de suite le
          // seuil sur l'amplitude du bit de départ.
          ampOn = lEnv;
          computeThresholds();
          lThH = thHigh;
          lThL = thLow;
        }
      } else if (lBit <= 8) {
        if (bit) lByte |= static_cast<uint8_t>(1U << (lBit - 1));
      } else {                               // bit de fin : doit être 0
        if (!bit) {
          enqueueReceivedByte(lByte);
          lStage = BitStage::Idle;
        } else {
          ++statFraming;                     // pas encore retombé : on attend
          lStage = BitStage::Stop;
        }
      }
      if (lStage == BitStage::Frame) {
        ++lBit;
        lNext = lStart + bitOffsets[lBit];
      }
    }
    lPrev = lLevel;
  }
  if (lBufMax > lDbgEnvMax) lDbgEnvMax = lBufMax;

  mid = lMid; env = lEnv; bufferMax = lBufMax; bufferMin = lBufMin;
  dbgEnvMax = lDbgEnvMax; dbgMin = lDbgMin; dbgMax = lDbgMax; dbgEdges = lEdges;
  level = lLevel; prevLevel = lPrev; sampleIndex = n; bitStage = lStage;
  nextSampleAt = lNext; frameStart = lStart; bitIndex = lBit; currentByte = lByte;
  lowCount = lLow;
  prevBlockClean = lPrevClean;
}

uint32_t busyMicros = 0;                    // temps passé à traiter (charge CPU)

void processAdcBuffers() {
  // Au plus 2 tampons par appel : loop() garde toujours du temps pour lire
  // les commandes et envoyer les messages sur le port série.
  for (uint8_t k = 0; k < 2 && buffersProcessed != buffersDone; ++k) {
    if (buffersDone - buffersProcessed >= NB_BUFFERS - 1) {
      // Le traitement a pris trop de retard : on saute et on repart propre.
      buffersLost += buffersDone - buffersProcessed - 1;
      buffersProcessed = buffersDone - 1;
      bitStage = BitStage::Idle;
    }
    __asm__ volatile("" ::: "memory");      // relire les tampons écrits par la DMA
    const uint32_t t0 = micros();
    processBuffer(adcBuffer[buffersProcessed % NB_BUFFERS]);
    ++buffersProcessed;
    updateThreshold();
    busyMicros += micros() - t0;
  }
}

// Écriture série sans jamais bloquer (sinon l'ADC déborde).
void safeWrite(const char *text) {
  const size_t n = strlen(text);
  if (static_cast<size_t>(Serial.availableForWrite()) >= n) {
    Serial.write(text);
  } else {
    ++statDropped;
  }
}

void safeWriteByte(uint8_t value) {
  if (Serial.availableForWrite() >= 1) {
    Serial.write(value);
  } else {
    ++statDropped;
  }
}

enum class ProtocolStage : uint8_t { WaitSot, WaitType, ReadMetadata, ReadPayload, ReadEot };
ProtocolStage protocolStage = ProtocolStage::WaitSot;
uint8_t frameType = 0;
uint8_t metadata[4] = {};
uint8_t metadataSize = 0;
uint8_t metadataRead = 0;
uint16_t rows = 0;
uint16_t columns = 0;
uint32_t payloadRemaining = 0;

void resetProtocol(uint8_t possibleSot = 0) {
  protocolStage = possibleSot == SOT ? ProtocolStage::WaitType : ProtocolStage::WaitSot;
  frameType = 0;
  metadataSize = 0;
  metadataRead = 0;
  rows = 0;
  columns = 0;
  payloadRemaining = 0;
}

void sendReceiveHeader(uint32_t payloadSize) {
  if (statsMode) return;
  char line[48];
  if (frameType == TYPE_TEXT) {
    snprintf(line, sizeof(line), "RCV_TXT %lu\r\n", static_cast<unsigned long>(payloadSize));
  } else {
    snprintf(line, sizeof(line), "%s %u %u %lu\r\n",
             frameType == TYPE_BW ? "RCV_IMGBW" : "RCV_IMG256",
             rows, columns, static_cast<unsigned long>(payloadSize));
  }
  safeWrite(line);
}

void finishMetadata() {
  if (frameType == TYPE_TEXT) {
    payloadRemaining = static_cast<uint16_t>(metadata[0]) |
                       (static_cast<uint16_t>(metadata[1]) << 8);
  } else {
    rows = static_cast<uint16_t>(metadata[0]) | (static_cast<uint16_t>(metadata[1]) << 8);
    columns = static_cast<uint16_t>(metadata[2]) | (static_cast<uint16_t>(metadata[3]) << 8);
    if (rows == 0 || columns == 0 || rows > MAX_IMAGE_DIMENSION || columns > MAX_IMAGE_DIMENSION) {
      ++statFramesErr;
      resetProtocol();
      return;
    }
    const uint32_t pixels = static_cast<uint32_t>(rows) * columns;
    payloadRemaining = frameType == TYPE_BW ? (pixels + 7) / 8 : 768 + pixels;
  }
  sendReceiveHeader(payloadRemaining);
  protocolStage = payloadRemaining == 0 ? ProtocolStage::ReadEot : ProtocolStage::ReadPayload;
}

void consumeReceivedByte(uint8_t value) {
  switch (protocolStage) {
    case ProtocolStage::WaitSot:
      if (value == SOT) protocolStage = ProtocolStage::WaitType;
      break;
    case ProtocolStage::WaitType:
      if (value == TYPE_TEXT || value == TYPE_BW || value == TYPE_256) {
        frameType = value;
        metadataSize = frameType == TYPE_TEXT ? 2 : 4;
        metadataRead = 0;
        protocolStage = ProtocolStage::ReadMetadata;
      } else {
        ++statFramesErr;
        resetProtocol(value);
      }
      break;
    case ProtocolStage::ReadMetadata:
      metadata[metadataRead++] = value;
      if (metadataRead == metadataSize) finishMetadata();
      break;
    case ProtocolStage::ReadPayload:
      if (!statsMode) safeWriteByte(value);
      if (--payloadRemaining == 0) protocolStage = ProtocolStage::ReadEot;
      break;
    case ProtocolStage::ReadEot:
      if (value == EOT) {
        ++statFramesOk;
        if (!statsMode) safeWrite("RCV_OK\r\n");
        resetProtocol();
      } else {
        ++statFramesErr;
        if (!statsMode) safeWrite("RCV_ERR_EOT\r\n");
        resetProtocol(value);
      }
      break;
  }
}

// ---------------------------------------------------------------- commandes
char command[32];
uint8_t commandLength = 0;
bool commandTooLong = false;

int32_t toMillivolts(int32_t v) {
  return v * 3300 / (4096 * SCALE);
}

void printStats() {
  char line[160];
  snprintf(line, sizeof(line),
           "STAT ok=%lu err=%lu oct=%lu fin=%lu amp=%ldmV seuil=%ldmV bruit=%ldmV perdus=%lu\r\n",
           static_cast<unsigned long>(statFramesOk), static_cast<unsigned long>(statFramesErr),
           static_cast<unsigned long>(statBytes), static_cast<unsigned long>(statFraming),
           static_cast<long>(toMillivolts(ampOn)), static_cast<long>(toMillivolts(thHigh)),
           static_cast<long>(toMillivolts(noise)),
           static_cast<unsigned long>(buffersLost + statDropped));
  safeWrite(line);
  statFramesOk = statFramesErr = statBytes = statFraming = 0;
}

void printDebug() {
  char line[127];
  snprintf(line, sizeof(line),
           "DBG A0 min=%ldmV max=%ldmV centre=%ldmV | env=%ldmV seuil=%ldmV | porteuse %lu | charge %lu%% perdus %lu\r\n",
           static_cast<long>(dbgMin * 3300L / 4096), static_cast<long>(dbgMax * 3300L / 4096),
           static_cast<long>(toMillivolts(mid)), static_cast<long>(toMillivolts(dbgEnvMax)),
           static_cast<long>(toMillivolts(thHigh)), static_cast<unsigned long>(dbgEdges),
           static_cast<unsigned long>(busyMicros / 5000), static_cast<unsigned long>(buffersLost));
  busyMicros = 0;
  safeWrite(line);
  dbgMin = 4095;
  dbgMax = 0;
  dbgEnvMax = 0;
  dbgEdges = 0;
}

void processCommand(const char *line) {
  if (strcmp(line, "DEBUG") == 0) {
    debugMode = !debugMode;
    safeWrite(debugMode ? "OK DEBUG ON\r\n" : "OK DEBUG OFF\r\n");
    return;
  }
  if (strcmp(line, "STATS") == 0) {
    statsMode = !statsMode;
    safeWrite(statsMode ? "OK STATS ON\r\n" : "OK STATS OFF\r\n");
    return;
  }
  if (strncmp(line, "SET_BAUD:", 9) != 0) {
    if (line[0] != '\0') safeWrite("ERR UNKNOWN_COMMAND\r\n");
    return;
  }
  char *end = nullptr;
  const unsigned long requestedRate = strtoul(line + 9, &end, 10);
  if (line[9] == '\0' || end == line + 9 || *end != '\0' ||
      requestedRate < MIN_BIT_RATE || requestedRate > MAX_BIT_RATE) {
    safeWrite("ERR INVALID_BAUD\r\n");
    return;
  }
  bitRate = requestedRate;
  configureDemodulator();
  char reply[32];
  snprintf(reply, sizeof(reply), "OK BAUD %lu\r\n", static_cast<unsigned long>(bitRate));
  safeWrite(reply);
}

void consumeCommandByte(uint8_t value) {
  if (value == '\r') return;
  if (value == '\n') {
    if (commandTooLong) {
      safeWrite("ERR COMMAND_TOO_LONG\r\n");
    } else {
      command[commandLength] = '\0';
      processCommand(command);
    }
    commandLength = 0;
    commandTooLong = false;
    return;
  }
  if (static_cast<size_t>(commandLength) + 1 >= sizeof(command)) {
    commandTooLong = true;
    return;
  }
  command[commandLength++] = static_cast<char>(value);
}

}  // namespace

void setup() {
  Serial.begin(115200);
  startAdc();
  measureSampleRate();
  configureDemodulator();
  char line[80];
  snprintf(line, sizeof(line), "READY RCV ADC (A0, %lu mesures/s, %lu bit/s)\r\n",
           static_cast<unsigned long>(sampleRate), static_cast<unsigned long>(bitRate));
  Serial.print(line);
  Serial.print("Commandes : SET_BAUD:10000 | STATS | DEBUG\r\n");
  lastStats = millis();
}

void loop() {
  processAdcBuffers();

  while (rxTail != rxHead) {
    const uint8_t value = rxBuffer[rxTail];
    rxTail = (rxTail + 1) & RX_BUFFER_MASK;
    consumeReceivedByte(value);
  }

  while (Serial.available() > 0) {
    const int received = Serial.read();
    if (received >= 0) consumeCommandByte(static_cast<uint8_t>(received));
  }

  if (statsMode && millis() - lastStats >= 1000) {
    lastStats += 1000;
    printStats();
  }
  if (debugMode && millis() - lastDebug >= 500) {
    lastDebug = millis();  // charge = temps de traitement sur ces 500 ms
    printDebug();
  }
}
