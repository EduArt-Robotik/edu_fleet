# Karte aufnehmen und für die Navigation verwenden

Diese Anleitung beschreibt, wie du mit dem Snackbot eine Umgebungskarte
aufnimmst, speicherst und anschließend für die Navigation verwendest.

Während der Aufnahme erstellt **SLAM** (*Simultaneous Localization and Mapping*)
eine Karte aus den Sensordaten des Roboters. Danach wird diese Karte vom Map
Server geladen und von **AMCL** zur Lokalisierung verwendet.

## Voraussetzungen

- Du hast Terminalzugriff auf den Roboter, beispielsweise über SSH.
- Der Roboter lässt sich mit dem Gamepad steuern.
- Der Fahrbereich ist frei und kann sicher befahren werden.
- Im Terminal, in dem du die Karte speicherst, ist die ROS-2-Umgebung geladen
  und das Paket `nav2_map_server` verfügbar.
- Falls du die Karte auf einem anderen Rechner speicherst, muss dieser die
  ROS-Topics des Roboters empfangen können. Dazu müssen insbesondere
  Netzwerkverbindung, ROS-Domain und DDS-Konfiguration zusammenpassen.

> **Wichtig:** Die folgenden Befehle verwenden den Namespace
> `/snack_bot/pepperl`. Passe den Map-Topic an, falls dein Roboter einen anderen
> Namespace verwendet. Alle `systemctl`-Befehle werden auf dem Roboter ausgeführt.

## 1. Navigation stoppen

Halte den Roboter an und beende die bestehende Navigation, bevor du die
Kartenaufnahme startest:

```bash
sudo systemctl stop snackbot-navigation.service
```

## 2. SLAM starten

Starte anschließend den SLAM-Dienst auf dem Roboter:

```bash
sudo systemctl start snackbot-slam.service
```

Prüfe, ob der Dienst erfolgreich gestartet wurde:

```bash
systemctl status snackbot-slam.service --no-pager
```

Bei Startproblemen kannst du die letzten Logmeldungen anzeigen:

```bash
journalctl -u snackbot-slam.service -n 100 --no-pager
```

## 3. Umgebung abfahren

Wähle am Gamepad die **Mecanum-Kinematik** über die **Kreis-Taste** aus.

Fahre die Umgebung langsam und möglichst gleichmäßig ab. Achte dabei auf
Folgendes:

- Vermeide abrupte Drehungen und schnelle Richtungswechsel.
- Erfasse alle Bereiche, die später für die Navigation benötigt werden.
- Fahre wichtige Durchgänge und Randbereiche vollständig ab.
- Besuche bereits erfasste Bereiche erneut, damit SLAM diese wiedererkennen
  und die Karte gegebenenfalls korrigieren kann.
- Halte ausreichend Abstand zu Hindernissen.

Beobachte die Karte während der Aufnahme, beispielsweise in RViz. Prüfe, ob
Wände, Hindernisse und freie Flächen plausibel dargestellt werden.

## 4. Karte speichern

Sobald die benötigten Bereiche vollständig erfasst sind, halte den Roboter an.
Lasse den SLAM-Dienst weiterlaufen und speichere die Karte in einem
beschreibbaren Arbeitsverzeichnis:

```bash
ros2 run nav2_map_server map_saver_cli \
  -f map_name \
  --ros-args \
  -r map:=/snack_bot/pepperl/map
```

Ersetze `map_name` durch einen aussagekräftigen Namen, beispielsweise
`snackbot_halle_2026_11_09`. Verwende einen neuen Namen, wenn du eine bestehende
Karte behalten möchtest. Ersetze diesen Platzhalter auch in allen weiteren
Befehlen und Konfigurationsbeispielen.

Der Befehl erstellt im aktuellen Arbeitsverzeichnis zwei Dateien:

| Datei | Inhalt |
|---|---|
| `map_name.yaml` | Kartenbeschreibung mit Bildpfad, Auflösung, Ursprung und Schwellwerten |
| `map_name.pgm` | Kartenbild mit freien, belegten und unbekannten Bereichen |

Prüfe die Ausgabe des Befehls auf Fehler und kontrolliere, ob beide Dateien
erstellt wurden:

```bash
ls -lh map_name.yaml map_name.pgm
```

> **Hinweis:** Diese Dateien speichern die Umgebungskarte für die Navigation,
> nicht den vollständigen SLAM-Zustand zur späteren Fortsetzung der Aufnahme.

## 5. Kartendateien auf den Roboter übertragen

Beide Dateien müssen auf dem Roboter in folgendem Verzeichnis liegen:

```text
/home/user/eduart/ros2/src/edu_fleet/docker/snackbot_on_raspi/launch_content
```

Wenn du die Karte auf einem anderen Rechner gespeichert hast, kannst du sie
aus dem Aufnahmeverzeichnis beispielsweise per SCP übertragen:

```bash
scp map_name.yaml map_name.pgm \
  user@ROBOT_IP:/home/user/eduart/ros2/src/edu_fleet/docker/snackbot_on_raspi/launch_content/
```

Ersetze `ROBOT_IP` durch die IP-Adresse oder den Hostnamen des Roboters.

Falls du die Karte direkt auf dem Roboter gespeichert hast, kopiere die
Dateien aus dem Aufnahmeverzeichnis in das Zielverzeichnis:

```bash
cp map_name.yaml map_name.pgm \
  /home/user/eduart/ros2/src/edu_fleet/docker/snackbot_on_raspi/launch_content/
```

Wenn du die Karte bereits im Zielverzeichnis gespeichert hast, entfällt
dieser Kopierschritt.

Prüfe anschließend in `map_name.yaml`, ob der Eintrag `image` auf die passende
Bilddatei verweist:

```yaml
image: map_name.pgm
```

Ein relativer Bildpfad wird relativ zum Speicherort der YAML-Datei aufgelöst.
Deshalb sollten beide Dateien zusammen im Zielverzeichnis liegen. Falls der
Eintrag einen absoluten Pfad vom Aufnahmerechner enthält, ändere ihn auf den
passenden relativen Dateinamen.

## 6. Neue Karte für die Navigation auswählen

Öffne auf dem Roboter die Datei
[`amcl.launch.py`](../../docker/snackbot_on_raspi/launch_content/amcl.launch.py)
im folgenden Verzeichnis:

```text
/home/user/eduart/ros2/src/edu_fleet/docker/snackbot_on_raspi/launch_content
```

Ändere die Zuweisung von `map_file` auf den Namen deiner neuen YAML-Datei:

```python
map_file = "./map_name.yaml"
```

Speichere die Änderung. Der relative Pfad setzt voraus, dass die Navigation
wie vorgesehen aus dem eingebundenen Verzeichnis `launch_content` startet.

> **Wichtig:** Bei `map_file` wird die **YAML-Datei** eingetragen, nicht die
> PGM-Datei. Behalte die bisherige Karte, bis die neue erfolgreich getestet wurde.

## 7. SLAM stoppen und Navigation starten

Beende zunächst die Kartenaufnahme:

```bash
sudo systemctl stop snackbot-slam.service
```

Starte anschließend die Navigation:

```bash
sudo systemctl start snackbot-navigation.service
```

Prüfe den Status:

```bash
systemctl status snackbot-navigation.service --no-pager
```

Bei Problemen helfen die Logmeldungen:

```bash
journalctl -u snackbot-navigation.service -n 100 --no-pager
```

## 8. Ergebnis überprüfen

Kontrolliere vor der ersten Navigationsfahrt:

- Wird die neue Karte angezeigt?
- Stimmen sichtbare Hindernisse mit der realen Umgebung überein?
- Ist der Roboter korrekt auf der Karte lokalisiert?
- Sind im Navigationslog keine Fehler beim Laden der Karte zu sehen?

Falls die Lokalisierung nicht stimmt, setze bei Bedarf die initiale
Roboterpose in RViz über **2D Pose Estimate**. Achte dabei sowohl auf die
Position als auch auf die Orientierung.

Teste die Navigation zunächst mit einem nahegelegenen Ziel in einem freien
Bereich. Beobachte den Roboter während der Fahrt und halte dich bereit, ihn
bei unerwartetem Verhalten zu stoppen.

## Häufige Probleme

| Problem | Prüfung |
|---|---|
| Die Karte kann nicht gespeichert werden. | Läuft der SLAM-Dienst? Ist `/snack_bot/pepperl/map` im speichernden Terminal erreichbar und das Arbeitsverzeichnis beschreibbar? |
| Die neue Karte wird nicht geladen. | Stimmt `map_file` in `amcl.launch.py`, und wurde die Navigation anschließend neu gestartet? |
| Das Kartenbild wird nicht gefunden. | Liegt die PGM-Datei im Zielverzeichnis, und stimmt der `image`-Eintrag in der YAML-Datei? |
| Der Roboter erscheint an einer falschen Position. | Prüfe die Lokalisierung und setze gegebenenfalls eine initiale Pose. |
| Wände erscheinen doppelt oder versetzt. | Fahre langsamer, vermeide abrupte Bewegungen und besuche bereits erfasste Bereiche erneut. Prüfe vor dem Speichern, ob sich die Darstellung verbessert. |

Wenn die neue Karte nicht verwendbar ist, stoppe die Navigation, trage die
bisherige YAML-Datei wieder als `map_file` ein und starte die Navigation erneut.
