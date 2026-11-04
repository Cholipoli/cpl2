// Programme pour faire les mesures SEUL : la Due emet toute seule, en boucle,
// pendant qu'on va mesurer aux autres poteaux. Pas besoin d'ordinateur
// branche sur la Due une fois le programme envoye : un chargeur USB suffit.
//
// Cycle automatique (5 s par etape, puis on recommence) :
//   [0] porteuse continue 100 kHz        -> mesurer VC (amplitude)
//   [1] lettre U en boucle a 10 kbit/s   -> enregistrer le message
//   [2] lettre U en boucle a 25 kbit/s
//   [3] lettre U en boucle a 50 kbit/s
//   [4] silence                          -> rien n'est emis
// La LED "L" de la Due est allumee pendant l'etape [0] (porteuse continue).
//
// Commandes facultatives dans le moniteur serie (115200 bauds), une lettre + Entree :
//   a = cycle automatique (par defaut au demarrage)
//   p = porteuse continue fixe     0 = silence fixe
//   1 = U a 10k fixe   2 = U a 25k fixe   5 = U a 50k fixe
//
// Le message U est exactement celui de main_tx : octets F0 8F 01 00 55 0F,
// chaque octet = 1 bit de depart (1), 8 bits poids faible d'abord, 1 bit de fin (0).
// Entre deux messages : 1 ms de silence.
#include <Arduino.h>
#include <DueTimer.h>

namespace {

constexpr uint16_t PWM_CHANNEL = 7;  // D6 = PC24 = canal PWM 7
constexpr uint16_t PWM_PERIOD = 840; // 84 MHz / 840 = 100 kHz
constexpr uint16_t PWM_HALF_PERIOD = PWM_PERIOD / 2;
constexpr uint32_t DUREE_ETAPE_MS = 5000;

constexpr uint8_t MESSAGE[] = {0xF0, 0x8F, 0x01, 0x00, 0x55, 0x0F};
constexpr uint16_t NB_BITS_MESSAGE = sizeof(MESSAGE) * 10;
constexpr uint32_t DEBITS[] = {10000, 25000, 50000};
constexpr uint8_t NB_ETAPES = 5;

volatile uint16_t indexBit = 0;
volatile uint16_t bitsSilence = 10;

bool automatique = true;
uint8_t etape = 0;
uint32_t debutEtape = 0;

void appliquerPorteuse(bool active) {
  PWM->PWM_CH_NUM[PWM_CHANNEL].PWM_CDTYUPD = active ? PWM_HALF_PERIOD : 0;
}

void configurerPorteuse() {
  pmc_enable_periph_clk(ID_PWM);
  PWM->PWM_CLK = PWM_CLK_DIVA(1) | PWM_CLK_PREA(0);
  PIO_Configure(PIOC, PIO_PERIPH_B, PIO_PC24, PIO_DEFAULT);
  PWM->PWM_CH_NUM[PWM_CHANNEL].PWM_CMR = PWM_CMR_CPRE_CLKA;
  PWM->PWM_CH_NUM[PWM_CHANNEL].PWM_CPRD = PWM_PERIOD;
  PWM->PWM_CH_NUM[PWM_CHANNEL].PWM_CDTY = 0;
  PWM->PWM_ENA = 1U << PWM_CHANNEL;
}

// Valeur du bit n° i du message (0 a 59) ; au-dela : silence entre messages.
bool bitDuMessage(uint16_t i) {
  if (i >= NB_BITS_MESSAGE) {
    return false;
  }
  const uint8_t octet = MESSAGE[i / 10];
  const uint8_t position = i % 10;
  if (position == 0) {
    return true;   // bit de depart
  }
  if (position == 9) {
    return false;  // bit de fin
  }
  return ((octet >> (position - 1)) & 1U) != 0;  // poids faible d'abord
}

// Appelee a chaque debut de bit (frequence = debit).
void bitSuivant() {
  appliquerPorteuse(bitDuMessage(indexBit));
  ++indexBit;
  if (indexBit >= NB_BITS_MESSAGE + bitsSilence) {
    indexBit = 0;
  }
}

void lancerEtape(uint8_t n) {
  Timer3.stop();
  indexBit = 0;
  digitalWrite(LED_BUILTIN, n == 0 ? HIGH : LOW);
  if (n == 0) {
    appliquerPorteuse(true);
    Serial.println("[0] PORTEUSE CONTINUE 100 kHz");
  } else if (n <= 3) {
    const uint32_t debit = DEBITS[n - 1];
    appliquerPorteuse(false);
    bitsSilence = static_cast<uint16_t>(debit / 1000);  // 1 ms de silence
    Timer3.setFrequency(static_cast<double>(debit));
    Timer3.start();
    Serial.print("[");
    Serial.print(n);
    Serial.print("] LETTRE U EN BOUCLE A ");
    Serial.print(debit / 1000);
    Serial.println(" kbit/s");
  } else {
    appliquerPorteuse(false);
    Serial.println("[4] SILENCE");
  }
}

void choisirFixe(uint8_t n) {
  automatique = false;
  etape = n;
  lancerEtape(n);
  Serial.println("    (mode fixe ; taper a pour revenir au cycle automatique)");
}

void traiterCommande(char c) {
  switch (c) {
    case 'a':
    case 'A':
      automatique = true;
      etape = 0;
      debutEtape = millis();
      Serial.println("Cycle automatique (5 s par etape)");
      lancerEtape(0);
      break;
    case 'p':
    case 'P':
      choisirFixe(0);
      break;
    case '1':
      choisirFixe(1);
      break;
    case '2':
      choisirFixe(2);
      break;
    case '5':
      choisirFixe(3);
      break;
    case '0':
      choisirFixe(4);
      break;
    default:
      break;
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  configurerPorteuse();
  Timer3.attachInterrupt(bitSuivant);
  Serial.println("test_seul : cycle automatique (5 s par etape)");
  Serial.println("Commandes : a = auto, p = porteuse, 1/2/5 = U a 10/25/50k, 0 = silence");
  debutEtape = millis();
  lancerEtape(0);
}

void loop() {
  while (Serial.available() > 0) {
    traiterCommande(static_cast<char>(Serial.read()));
  }
  if (automatique && millis() - debutEtape >= DUREE_ETAPE_MS) {
    etape = (etape + 1) % NB_ETAPES;
    debutEtape = millis();
    lancerEtape(etape);
  }
}
