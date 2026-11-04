// Programme de test pour la séance de mesures (cahier d'expérience).
// Il allume la porteuse sur D6 EN CONTINU, sans message, pour mesurer
// amplitudes, impédance et réponse des filtres.
//
// Utilisation :
//   1. pio run -e test_porteuse -t upload
//   2. pio device monitor -b 115200   (ou le moniteur série de VS Code)
//   3. Taper une fréquence en kHz puis Entrée : 100, 85, 150...
//      Taper 0 pour éteindre la porteuse.
// Au démarrage, la porteuse est à 100 kHz.
//
// La fréquence réelle vaut 84 MHz / période, la période étant un entier :
// le programme affiche la fréquence exacte obtenue.
#include <Arduino.h>

namespace {

constexpr uint16_t PWM_CHANNEL = 7;      // D6 = PC24 = canal PWM 7
constexpr uint32_t HORLOGE_HZ = 84000000;
constexpr uint32_t PERIODE_MIN = 168;    // 500 kHz au maximum
constexpr uint32_t PERIODE_MAX = 65535;  // ~1,3 kHz au minimum

char saisie[16];
uint8_t longueur = 0;

void configurerPwm() {
  pmc_enable_periph_clk(ID_PWM);
  PWM->PWM_CLK = PWM_CLK_DIVA(1) | PWM_CLK_PREA(0);
  PIO_Configure(PIOC, PIO_PERIPH_B, PIO_PC24, PIO_DEFAULT);
  PWM->PWM_CH_NUM[PWM_CHANNEL].PWM_CMR = PWM_CMR_CPRE_CLKA;
  PWM->PWM_CH_NUM[PWM_CHANNEL].PWM_CPRD = 840;
  PWM->PWM_CH_NUM[PWM_CHANNEL].PWM_CDTY = 0;
  PWM->PWM_ENA = 1U << PWM_CHANNEL;
}

void eteindre() {
  PWM->PWM_CH_NUM[PWM_CHANNEL].PWM_CDTYUPD = 0;
  Serial.println("Porteuse ETEINTE");
}

void regler(uint32_t periode) {
  // CPRDUPD / CDTYUPD : changement pris en compte à la fin de la période
  // en cours, sans glitch. Rapport cyclique 50 %.
  PWM->PWM_CH_NUM[PWM_CHANNEL].PWM_CPRDUPD = periode;
  PWM->PWM_CH_NUM[PWM_CHANNEL].PWM_CDTYUPD = periode / 2;
  Serial.print("Porteuse ALLUMEE : periode = ");
  Serial.print(periode);
  Serial.print(" -> frequence = ");
  Serial.print(static_cast<double>(HORLOGE_HZ) / periode / 1000.0, 3);
  Serial.println(" kHz");
}

void traiter(const char *texte) {
  const double khz = atof(texte);
  if (khz <= 0.0) {
    eteindre();
    return;
  }
  uint32_t periode = static_cast<uint32_t>(HORLOGE_HZ / (khz * 1000.0) + 0.5);
  if (periode < PERIODE_MIN || periode > PERIODE_MAX) {
    Serial.println("Frequence hors limites (2 a 500 kHz)");
    return;
  }
  if (periode % 2 != 0) {
    ++periode;  // période paire : rapport cyclique exactement 50 %
  }
  regler(periode);
}

}  // namespace

void setup() {
  Serial.begin(115200);
  configurerPwm();
  regler(840);  // 100 kHz au démarrage
  Serial.println("Tapez une frequence en kHz (0 = eteindre).");
}

void loop() {
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\r') continue;
    if (c == '\n') {
      saisie[longueur] = '\0';
      if (longueur > 0) traiter(saisie);
      longueur = 0;
    } else if (longueur < sizeof(saisie) - 1) {
      saisie[longueur++] = c;
    }
  }
}
