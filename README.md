# Communication CPL — Arduino Due

Le projet utilise deux Arduino Due distincts : un émetteur et un récepteur,
reliés par les circuits de couplage CPL. L’application `interface_cpl.py`
propose uniquement les rôles Émetteur et Récepteur.

## Installation et lancement

```powershell
py -3 -m pip install -r requirements.txt
py -3 interface_cpl.py
```

Choisir le rôle, le port série et le débit CPL (1 000 à 50 000 bit/s). En mode
Émetteur, sélectionner Texte, IMGBW ou IMG256. En mode Récepteur, le programme
écoute en arrière-plan et propose d’enregistrer chaque message TXT validé comme
`.txt`, ou chaque image reconstruite comme `.png`.

Le format suit l’annexe du cahier des charges. Les dimensions (lignes puis
colonnes, 2 octets chacune, poids faible d’abord) sont acceptées jusqu’à
4096 × 4096 ; au-delà de 320 × 320, l’interface propose de réduire l’image
(une image 320 × 320 prend ≈ 5 s en IMG-BW et ≈ 41 s en IMG-256 à 25 kbit/s).
Les images BW sont seuillées à 128, parcourues ligne par ligne depuis le coin
supérieur gauche, 8 pixels par octet, le dernier octet complété par des 0.
Deux conventions ne sont pas fixées par le cahier des charges et se règlent en
tête de `interface_cpl.py` : `BW_WHITE_IS_ONE` (blanc = 1) et
`BW_FIRST_PIXEL_IN_MSB` (1er pixel dans le bit 7). Les images IMG256 contiennent
une palette RGB de 256 entrées (768 octets) puis un octet par pixel. Le texte
est ASCII.

En réception, rien n’est enregistré automatiquement : le bouton « Enregistrer le
dernier message reçu… » sauvegarde le dernier texte (.txt) ou la dernière image
(.png).

## Firmware

Les deux environnements PlatformIO compilent chacun un unique fichier source :

```powershell
pio run -e emitter
pio run -e receiver
pio run -e emitter -t upload
pio run -e receiver -t upload
```

Flasher `emitter` sur l’Arduino émettrice et `receiver` sur la réceptrice.
L’émetteur génère la porteuse PWM de 100 kHz sur la sortie D6 (PC24, canal PWM7)
et cadence les bits avec un timer matériel.

Le récepteur (`main_rx.cpp`) lit la sortie du double filtre passe-bande sur
**A0** : 100 nF en série, puis 10 kΩ en série, puis A0 polarisé à ~1,65 V par
deux 10 kΩ (vers 3,3 V et vers GND). L’ADC mesure A0 en continu (~665 000
mesures/s, DMA) ; le programme en déduit l’enveloppe de la porteuse, la compare
à un seuil qui s’adapte seul au niveau du signal et du bruit, puis lit chaque
bit en son milieu (bit de départ, 8 bits poids faible d’abord, bit de fin).
Débit conseillé : **25 000 bit/s** (validé à travers un poteau) ; 50 000 bit/s
est à la limite et produit des erreurs.

Commandes de diagnostic dans le moniteur série (à ne pas laisser actives avec
l’interface) : `DEBUG` affiche deux fois par seconde la tension vue sur A0,
l’enveloppe, le seuil et la charge du processeur ; `STATS` affiche une ligne de
statistiques par seconde au lieu des messages.

Programmes de test : `test_porteuse` (porteuse continue réglable) et
`test_seul` (émission autonome en boucle pour les mesures aux poteaux).

La sortie Due est logique 3,3 V et ne doit jamais être reliée directement au
secteur ou à une prise électrique. Utiliser un circuit de couplage CPL isolé,
protégé et conçu pour les tensions et normes applicables. Valider le signal à
l’oscilloscope sur une charge basse tension appropriée.

## Protocole

La liaison USB série fonctionne à 115200 bauds. L’émetteur reçoit les commandes
`SET_BAUD:<débit>\n`, puis `SEND_TXT\n`, `SEND_IMGBW\n` ou `SEND_IMG256\n`.
Il répond respectivement `OK BAUD <débit>`, `READY`, puis `DONE` après émission.

Une trame CPL est composée de SOT (`0xF0`), du type (`0x8F` TXT, `0xF8` IMGBW,
`0x55` IMG256), des métadonnées little-endian, du payload et d’EOT (`0x0F`).
La longueur TXT occupe 2 octets. Les images transmettent les lignes et les
colonnes sur 2 octets chacune. Le récepteur vérifie l’EOT, puis transmet au PC
un entête USB `RCV_TXT`, `RCV_IMGBW` ou `RCV_IMG256`, suivi du payload binaire et
du statut `RCV_OK`.
