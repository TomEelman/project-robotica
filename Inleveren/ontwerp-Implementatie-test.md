# Objectgeoriënteerd ontwerp, implementatie en testen

## Ontwerp (UML)

De software is opgezet volgens een objectgeoriënteerd ontwerp waarbij elke
verantwoordelijkheid in een eigen klasse is ondergebracht. Het systeem is
verdeeld over twee processoren: de Raspberry Pi Pico verzorgt de
laag-niveau aansturing en sensoruitlezing, de Raspberry Pi 5 verzorgt de
hoog-niveau navigatie en mapping. 

Op de Pico draait de aansturing van de hardware. De klasse `Robot` bundelt
de fysieke onderdelen en delegeert taken aan de onderliggende klassen.
`Motor` regelt de aansturing van één motor, `Encoder` leest de wielsnelheid
uit, en `IMU` levert de oriëntatie (yaw) van de robot. `SensorHub` bundelt
de twee encoders en de IMU tot één toegangspunt voor sensordata. `Drive`
bevat de regeltechniek: het rekent een gewenste lineaire en hoeksnelheid om
naar motoraansturing, met aparte PID-regelaars per wiel en een PID voor de
koers (yaw). De communicatie tussen de twee processoren verloopt via UART,
afgehandeld door `PicoUARTHandler` aan de Pico-kant en `Pi5UARTHandler` aan
de Pi-kant.

Op de Pi 5 zijn er de klassen `Mapper` en `GridMap` welke samen een
occupancy-grid maken van de omgeving op basis van de LIDAR-scans (`LIDAR`).
`Localisation` schat de positie van de robot via odometrie en IMU, en
`ScanMatcher` corrigeert deze schatting. `Navigator` bepaalt het rijgedrag:
het volgt een pad, ontwijkt obstakels en stuurt aan op herplanning.
`PathPlanner` en `ExplorationPlanner` berekenen een route
naar een doel en een nieuw te verkennen doel (frontier-based exploration).
`DriveCommand` en `Position` zijn datadragende klassen die door het systeem
worden doorgegeven.


## Implementatie

De implementatie is geschreven in C++ en volgt het ontwerp één-op-één:
elke klasse uit het UML heeft een eigen `.h`- en `.cpp`-bestand. De
verantwoordelijkheden zijn gescheiden, zodat klassen onafhankelijk van
elkaar aangepast kunnen worden. Zo kent `Navigator`
geen details van de motoraansturing, en weet `Drive` niets van padplanning.

membervariabelen zijn `private` en alleen
via methodes benaderbaar. Read-only toegang verloopt via `const`-methodes
(bijvoorbeeld `GetCurrentYaw()` en `GetSpeedLeft()`), zodat de interne toestand
van een object niet ongewenst gewijzigd kan worden. Configuratiewaarden zoals
PID-parameters, pinnummers en drempelwaarden zijn als constanten wat het systeem aanpasbaar en leesbaar houdt.

De objectgeoriënteerde opzet maakte het mogelijk om onderdelen los te
ontwikkelen en te vervangen. Een voorbeeld is de aansturingslaag rond
`ExplorationPlanner`, die tijdens de ontwikkeling is geherstructureerd zonder
dat de aanroepende `Navigator`-logica overhoop hoefde.

## Testen

Het systeem is op verschillende niveaus getest. Losse onderdelen zijn eerst
afzonderlijk beproefd: de PID-regelaars van `Drive` zijn afgesteld door de
robot rechtuit en op de plek te laten draaien en het gedrag te observeren,
waarbij parameters als de maximale integraalwaarde en de feedforward zijn
afgesteld op stabiel rijden. De sensoruitlezing van `Encoder` en `IMU` is
gecontroleerd via uitgelezen waarden over de UART-verbinding.

Vervolgens is het samengestelde systeem getest in een afgesloten testomgeving,
waarbij de robot autonoom de ruimte in kaart bracht. Tijdens deze tests is het
gedrag van `Navigator` beoordeeld: het volgen van een gepland pad, het
ontwijken van obstakels en het opnieuw bepalen van een verkenningsdoel wanneer
een pad geblokkeerd raakte. De opgebouwde kaart is na afloop opgeslagen en
visueel gecontroleerd op volledigheid en juistheid.

Bevindingen uit de tests zijn teruggekoppeld naar het ontwerp en de
implementatie, waarna parameters en gedrag iteratief zijn bijgesteld.