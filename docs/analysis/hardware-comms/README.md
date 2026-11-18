# Area: hardware-comms (Hardware, comunicazioni e logging)

Wi-Fi e hotspot, Telegram, PWA e gateway, registrazione delle missioni e bag.

## 1. Stato attuale

> Telegram, PWA e hotspot funzionano su linus, nessuno provato sull'auto; il recorder perdeva l'inizio delle missioni.

- Recorder di missione: 17 bag vuoti su 58 nella prima campagna, primo messaggio mediano 8.2 s dopo l'avvio; ora registra da LOADED con pre-roll.
- Un `ros2 bag record -a` accanto al mission logger perde il 24 % di `/scan`; il bag del mission logger è completo a 40 Hz.
- Telegram via `f1tenth_ctl`: sequenza completa su sim linus; Whisper medium 2/3 comandi vocali.
- PWA e gateway: provati con gateway finto e rclpy su linus, 0 prove su hardware.
- Hotspot base su linus: AP 5 GHz canale 36 ok, l'adattatore USB cade dal bus 31 volte in 13 minuti.

## 2. Decisioni prese

- Registrare solo con il mission logger o con lista topic esplicita e QoS profondo ([2026-10-16_logger-check](2026-10-16_logger-check/report.md))
- Recorder pre-armato in pausa con ripresa al LOAD (proposta) ([2026-10-07_mission-recorder-late-start](2026-10-07_mission-recorder-late-start/report.md))
- Tailscale prima, hotspot 10.42.50.1 come fallback; `enable_comms` default false ([2026-10-21_comms-base-wifi](2026-10-21_comms-base-wifi/report.md))
- Voce Telegram con faster-whisper medium ([2026-10-20_telegram-control](2026-10-20_telegram-control/report.md))

## 3. Domande aperte / prossimi esperimenti

- Prova sull'auto di Telegram, PWA (checklist di 22 passi a ruote sollevate) e link_manager.
- Stabilità dell'adattatore Wi-Fi USB.
- CPU del recorder in pausa sul Jetson.

## 4. Esperimenti

Tabella generata da `python3 tools/analysis_index.py`: non modificarla a mano.

<!-- AUTO:START -->
| data | esperimento | status | risultato | moduli (versione) |
|---|---|---|---|---|
| 2026-10-21 | [Hotspot base, link_manager e copertura Wi-Fi: setup e piano di test](2026-10-21_comms-base-wifi/report.md) | open | Live test linus ~12 min: AP 5 GHz ch 36 ok, iPhone connesso, link -27/-36 dBm, 0 retry, uplink 244 controlli 0 falliti; 2.4 GHz senza beacon; adattatore USB cade dal bus 31 volte in 13 min. | f1tenth_comms 0.0.1, f1tenth_gateway 0.0.1, f1tenth_bringup 0.0.1, f1tenth_messages 0.0.1 |
| 2026-10-20 | [Controllo Telegram via f1tenth_ctl: ricognizione e test su sim](2026-10-20_telegram-control/report.md) | open | Sequenza connect->stack->READY->planner->logger->estop->killall passata; 117 unit test pass (49 bot test con python-telegram-bot 22.8); Whisper medium 2/3 comandi giusti (~0.27 s) vs small 1/3. | f1tenth_telegram 0.0.1, f1tenth_ctl 0.0.1 |
| 2026-10-20 | [PWA F1Tenth Lab Control e gateway (fasi 1-4)](2026-10-20_pwa-lab-control/report.md) | open | App, gateway e schermate provati su linus (gateway finto, rclpy reale su Humble, Playwright WebKit/Chromium); nulla provato su auto, Orin o iPhone vero. | f1tenth_gateway 0.0.1, f1tenth_ctl 0.0.1, f1tenth_telegram 0.0.1 |
| 2026-10-16 | [Il bag mente o lo stack rallenta? corridor10m_20261015T171753](2026-10-16_logger-check/report.md) | done | Verdetto (c) entrambi: nel bag /scan a 30.4 Hz contro 40.0 prodotti (24 % perso, 722 mancanti); wall tracker 155-178 ms per scan con CPU 95-97 % su 12 core, 4.2 osservazioni/s per 40 scan/s. | f1tenth_logger 0.0.1, f1tenth_perception 0.0.1 |
| 2026-10-07 | [Mission recorder: partenza tardiva e bag vuoti](2026-10-07_mission-recorder-late-start/report.md) | open | 58 run, 41 bag con dati, 17 vuoti (tutti i 10 di M03, 6 di M04, P002-R006). Primo messaggio mediana 8.2 s dopo mission_started (4.9-13.7 s); copertura mediana 42 % (5-76 %). | f1tenth_logger 0.0.1 |
<!-- AUTO:END -->
