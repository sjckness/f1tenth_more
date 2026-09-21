#!/usr/bin/env python3
"""
LLM Planner Node -- versione llama-server (Qwen2.5-3B-Instruct locale).

Traduce un comando in linguaggio naturale in un PIANO JSON (vocabolario
invariato: modes {straight, wall_turn}, guards {wall, turned, front_object,
distance}), poi lo traduce (plan_translate.phases_to_mission()) in una
missione del sistema REALE di f1tenth_behavior e la consegna tramite il
flusso di servizi gia' in produzione:

    LLM -> normalize_plan -> validate_plan -> phases_to_mission()
        -> file JSON -> /mission/abort_mission (se una missione era gia'
           in corso) -> /mission/load_mission -> /mission/start_mission

Uso (package reale: 'llm', non 'f110_autonomy'):
    ros2 run llm llm_planner_node "vai dritto, al muro gira a destra"
    ros2 run llm llm_planner_node            # interattivo (come prima)
    ros2 run llm llm_planner_node --dry-run "..."     # traduce, non carica
    ros2 run llm llm_planner_node --confirm "..."     # chiede conferma

Unica "interrogation" di questo package (interrogations.yaml) da quando
llm_mpc_tuner_node.py e' stato RIMOSSO del tutto, non solo deprioritizzato
-- vedi git history se serve recuperarlo. Il meccanismo default_model/
sentinel di llm.launch.py (vedi sotto) resta comunque generale, non
planner-specifico: pensato per una eventuale seconda interrogation futura
senza doverlo ricostruire.

Backend: llama-server's raw /completion endpoint -- lo STESSO pattern di
richiesta che llm_mpc_tuner_node.py (rimosso, vedi sopra) gia' usava per il
suo stesso backend, avviato da llm.launch.py via interrogations.yaml/
models.yaml (vedi get_plan_from_llm() piu' sotto per il pattern di
richiesta, all'epoca copiato dal ask_llm() di quel file: prompt grezzo via
`requests`, NESSUN templating ChatML, NESSUN vincolo grammar/json_schema --
solo prompting + temperature bassa). Sostituisce la versione precedente di
questo pass, che parlava con Ollama locale (Qwen 2.5 3B) via il client
`openai` OpenAI-compatibile -- cambiato deliberatamente per consolidare
sullo stesso backend/processo llama-server gia' in produzione per quel
nodo (all'epoca ancora presente), non come effetto collaterale di
qualcos'altro. Il modello servito e' lo Qwen2.5-3B-Instruct ufficiale (non
pruned), un proprio entry in models.yaml (`qwen25_3b_instruct`) selezionato
di default per questa interrogation via interrogations.yaml's
`default_model` (vedi anche llm.launch.py's model/interrogation pairing
fix, la stessa modifica che ha reso questo cambio sicuro: prima, scegliere
una interrogation diversa dal default senza passare anche model:=...
esplicitamente avrebbe servito quel nodo con il modello sbagliato, senza
errori).

get_plan_from_llm() e' l'unica funzione toccata in questo pass -- stessa
firma, stesso contratto di ritorno (lista di fasi o eccezione).
normalize_plan/validate_plan/SYSTEM_PROMPT/plan_translate.py e tutto cio'
che sta a valle del ritorno di get_plan_from_llm() sono INVARIATI.

--------------------------------------------------------------------------
Pass successivo (readiness/warm-up): due problemi erano in realta' lo
stesso -- il nodo non aveva alcun concetto reale di "il server e' pronto"
prima di usarlo.
  1. Un ConnectionError grezzo (urllib3 traceback) arrivava fino
     all'operatore dentro "traduzione fallita (...)", senza indicazione
     azionabile quando llama-server semplicemente non era in ascolto.
  2. Cold-start non gestito: la PRIMA chiamata /completion contro un
     llama-server appena avviato e' stata osservata dal vivo a ~58s contro
     un LLAMA_TIMEOUT di 60s -- un comando arrivato durante quella finestra
     avrebbe rischiato di andare in timeout o fallire come sopra.
Risolti insieme con _wait_for_llama_server(): un vero readiness+warm-up
check (una richiesta /completion reale, non un ping) chiamato in
LLMPlannerNode.__init__ PRIMA che qualsiasi comando (interattivo o
one-shot) possa raggiungere get_plan_from_llm(), piu' LLAMA_TIMEOUT alzato
a 90s come margine di sicurezza e un except dedicato per ConnectionError
dentro get_plan_from_llm() stessa (server riavviato/crashato a meta'
sessione, dopo che il warm-up era gia' passato).

--------------------------------------------------------------------------
Rispetto alla versione originaria di questo file (loose, non pacchettizzata,
pubblicava un array nudo di fasi su /corridor_cmd per un controller SLSQP
che non usiamo piu'):
  - rimossi l'import morto `from f110_llm_planner import plan_amend` e
    l'intera funzionalita' di EMENDAMENTO in-corsa (AMEND_PROMPT,
    get_amendment_from_llm, process_amendment, /plan_status,
    /corridor_update, --force-full) -- vedi il modulo docstring della
    classe piu' sotto per il motivo e il nuovo comportamento (abort +
    ricarica) che la sostituisce.
  - rimossi self.pub/corridor_cmd_qos/--topic (non pubblichiamo piu' su
    /corridor_cmd; consegniamo la missione via /mission/load_mission +
    /mission/start_mission, non un topic latched).

validate_plan blocca quattro piani che altrimenti romperebbero il robot a
valle (invariato dalla versione originaria):
  1. wall_turn senza "turn_sign": l'MPC fa .get("turn_sign", default) e gira
     nel verso sbagliato senza segnalare nulla.
  2. "thresh" non numerico (es. "3.0 metri"): guard_satisfied confronta float
     con str -> TypeError dentro control_loop -> il nodo MPC crasha.
  3. "stop_at"/"stop_at_distance" su una fase NON finale: quella fase impone
     vdes=0 e non avanza mai, il robot si pianta a meta' percorso.
  4. ultima fase senza "stop_at" ne' "stop_at_distance": il robot non ha
     nessuna condizione di arresto (qui solo warning, il piano viene
     tradotto e caricato lo stesso).
--------------------------------------------------------------------------
"""

import argparse
import json
import logging
import os
import sys
import threading
import time

import requests

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

from ament_index_python.packages import get_package_share_directory
from f1tenth_messages.srv import LoadMission
from std_msgs.msg import String
from std_srvs.srv import Trigger

from llm.plan_translate import (
    INTENT_PROMPT_FILENAME,
    EmptyPlanError,
    IntentRangeError,
    IntentSchemaError,
    PlanTranslationError,
    TranslatorOutputError,
    UnsupportedIntentModeError,
    intent_prompt_filename,
    load_intent_prompt,
    phases_to_mission,
    translate,
)

# =========================================================
# Config LLM (llama-server /completion) -- vedi get_plan_from_llm() per il
# pattern di richiesta, copiato da llm_mpc_tuner_node.ask_llm().
# =========================================================
# Matcha models.yaml's `qwen25_3b_instruct` entry's own default port (8083)
# -- corretto per un `ros2 run llm llm_planner_node "..."` diretto contro un
# llama-server avviato a mano su quella porta. Quando lanciato tramite
# llm.launch.py, il parametro ROS 'llm_url' (dichiarato in __init__, vedi
# sotto) sovrascrive questo default con la porta REALE risolta a quel
# momento (models.yaml puo' cambiare porta/modello senza toccare questo
# file) -- stesso meccanismo di override che llm_mpc_tuner_node.py's own
# mpc_url gia' usa. Modulo-level (non un attributo d'istanza) perche'
# get_plan_from_llm() resta una funzione libera con la stessa firma di
# prima (command_text -> list) -- LLMPlannerNode.__init__ riassegna questi
# due globals una volta, all'avvio, dal parametro ROS risolto (stesso
# pattern del client _client sotto: stato modulo-level che una funzione
# libera legge, gia' usato da questo file prima di questo pass).
LLAMA_URL = "http://127.0.0.1:8083/completion"
# Alzato da 60.0 a 90.0 (pass readiness/warm-up) -- margine di sicurezza per
# un server riavviato a meta' sessione (di nuovo freddo) anche se il warm-up
# di avvio (_wait_for_llama_server sotto) dovrebbe gia' rendere veloci
# (~1s, misurato) le chiamate per-comando nel caso normale. Senza timeout
# una chiamata appesa blocca tutto.
LLAMA_TIMEOUT = 90.0

# Vocabolario ESATTO del piano LLM -- INVARIATO (validate_plan lo applica,
# non ha nulla a che fare con lo schema missioni reale, vedi plan_translate.py).
VALID_MODES = ("straight", "wall_turn")
VALID_GUARDS = ("wall", "turned", "front_object", "distance")

# soglie plausibili, per intercettare allucinazioni numeriche -- INVARIATO
#
# NOTA su "turned" (verificata, non modificata in questo passaggio): il limite
# superiore e' 6.3 radianti, cioe' ~361 gradi, quindi era GIA' ben sopra pi
# greco e accettava 180 gradi (3.1416) e 270 gradi (4.712). Quello che prima
# non funzionava non era questa validazione ma l'ESECUZIONE: orientation_delta
# confrontava un delta yaw "wrapped", matematicamente limitato a (-180, 180]
# gradi, quindi 180 era al massimo sfiorabile e qualsiasi valore sopra era
# irraggiungibile per costruzione. Da quando CheckStopCondition accumula la
# rotazione in modo unwrapped (vedi condition_eval.turn_accum_deg) quel
# soffitto non esiste piu' e questo range e' finalmente realizzabile per intero.
THRESH_RANGE = {
    "wall": (0.2, 10.0),
    "front_object": (0.2, 10.0),
    "distance": (0.1, 50.0),
    "turned": (0.05, 6.3),      # radianti (~361 gradi -- vedi nota sopra)
}

# Timeout per ciascuna chiamata di servizio verso il sistema missioni
# (/mission/load_mission, /mission/start_mission, /mission/abort_mission) --
# distinto da LLAMA_TIMEOUT sopra (quello e' per la chiamata HTTP all'LLM).
MISSION_SERVICE_WAIT_SEC = 2.0
MISSION_SERVICE_CALL_TIMEOUT_SEC = 5.0

SYSTEM_PROMPT = """Sei un traduttore da linguaggio naturale a un piano di navigazione JSON per un robot.
Il robot esegue una SEQUENZA DI FASI. Ogni fase e' un oggetto JSON.

CAMPI OBBLIGATORI:
- "mode": uno tra ["straight", "wall_turn"]
    - "straight": va dritto mantenendo l'orientamento iniziale.
    - "wall_turn": gira (serve "turn_sign").
- "guard": condizione che fa AVANZARE alla fase successiva. Uno tra:
    - "wall": muro davanti piu' vicino di "thresh" metri.
    - "turned": ruotato di piu' di "thresh" radianti.
    - "front_object": oggetto davanti piu' vicino di "thresh" metri.
    - "distance": percorsi "thresh" metri in questa fase.
- "thresh": soglia NUMERICA (metri per wall/front_object/distance, radianti per turned).
  Deve essere un numero, non una stringa: 3.0 e non "3.0 metri".

CAMPI OPZIONALI:
- "turn_sign": OBBLIGATORIO se mode e' "wall_turn". -1.0 = DESTRA, +1.0 = SINISTRA.
- "stop_at": SOLO ULTIMA fase. Stop quando oggetto davanti <= valore (metri).
- "stop_at_distance": SOLO ULTIMA fase. Stop dopo N metri.

REGOLE:
- "vai dritto" -> "straight". "al muro" -> guard "wall" (thresh ~3.0).
- "gira a destra/sinistra" -> wall_turn, turn_sign -1.0/+1.0, di solito con guard "turned" thresh 1.3.
- "avanza N metri" (finale) -> guard "distance", thresh N, stop_at_distance N.
- "fermati all'oggetto/sedia" (finale) -> guard "front_object", thresh 1.0, stop_at 1.0.
- "supera / oltrepassa / passa accanto a X" NON e' una fermata: l'evitamento ostacoli
  e' gestito dal controllore. Usa "distance" per proseguire, NON "stop_at".
- Ogni fase DEVE avere un "guard". Se il comando dice solo "vai avanti" senza
  destinazione, usa guard "wall" (thresh 3.0) come default.

REGOLE STRUTTURALI (violarle blocca il robot):
1. Dopo una svolta (wall_turn), per proseguire NON usare "straight"
   (riallineerebbe all'orientamento iniziale): usa "wall_turn" con lo STESSO turn_sign.
2. "stop_at" e "stop_at_distance" possono comparire SOLO sull'ULTIMA fase.
   Su una fase intermedia il robot si ferma li' e non prosegue mai.
3. L'ULTIMA fase deve SEMPRE avere "stop_at" oppure "stop_at_distance",
   altrimenti il robot non si ferma mai.

Se il comando contiene azioni non esprimibili (saltare, volare, ecc.), IGNORALE
e produci comunque un piano valido con le primitive disponibili.

Rispondi ESCLUSIVAMENTE con {"plan":[...]}. NIENTE testo o markdown.

ESEMPIO 1:
comando: "vai dritto, al muro gira a destra e avanza 2 metri"
{"plan":[{"mode":"straight","guard":"wall","thresh":3.0},{"mode":"wall_turn","turn_sign":-1.0,"guard":"turned","thresh":1.3},{"mode":"wall_turn","turn_sign":-1.0,"guard":"distance","thresh":2.0,"stop_at_distance":2.0}]}

ESEMPIO 2 (ostacolo da superare, non da raggiungere):
comando: "vai dritto, supera l'ostacolo e avanza 3 metri"
{"plan":[{"mode":"straight","guard":"distance","thresh":3.0,"stop_at_distance":3.0}]}
"""


class LlamaServerUnreachableError(RuntimeError):
    """Sollevata da _wait_for_llama_server() quando llama-server non diventa
    raggiungibile (e scaldato) entro max_wait_s -- un messaggio chiaro e
    azionabile per l'operatore, non una ConnectionError/Timeout grezza di
    requests con dentro un traceback urllib3 illeggibile."""


def _wait_for_llama_server(url: str, max_wait_s: float = 90.0) -> float:
    """Attende che llama-server sia raggiungibile E scaldato prima di
    lasciar passare qualsiasi comando -- vedi il modulo docstring, sezione
    "Pass successivo (readiness/warm-up)", per il problema che questo
    risolve.

    Ogni tentativo invia una richiesta /completion MINIMA ma REALE (prompt
    banale, n_predict piccolo) -- non un semplice ping/socket-connect: e'
    proprio l'elaborazione di una richiesta reale a caricare il modello in
    memoria GPU la prima volta (il warm-up da ~20-30s, osservato dal vivo
    fino a 58s), quindi questo stesso check raddoppia da readiness-check a
    warm-up. Timeout per tentativo = LLAMA_TIMEOUT (letto dal valore
    corrente del modulo, non catturato all'import): un singolo tentativo
    puo' legittimamente restare appeso per tutta la finestra di warm-up
    prima di rispondere con successo.

    Retry con backoff (1s -> raddoppia -> tetto 5s tra un tentativo e il
    successivo) fino a max_wait_s totali. Si ritenta SOLO su
    requests.exceptions.ConnectionError (server non ancora in ascolto --
    atteso durante l'avvio del processo llama-server, prima che apra la
    porta). Qualsiasi altra eccezione (risposta HTTP di errore, JSON
    malformato, ecc.) si propaga immediatamente, senza ritentare alla
    cieca -- non e' detto che ritentare risolva un problema diverso da "non
    ancora in ascolto".

    Ritorna i secondi di attesa impiegati al successo. Solleva
    LlamaServerUnreachableError (non una ConnectionError grezza) se
    max_wait_s viene esaurito senza mai riuscire a connettersi.
    """
    t0 = time.time()
    delay = 1.0
    last_exc = None
    while True:
        try:
            response = requests.post(
                url,
                json={'prompt': 'ciao', 'n_predict': 8, 'temperature': 0.0},
                timeout=LLAMA_TIMEOUT,
            )
            response.raise_for_status()
            return time.time() - t0
        except requests.exceptions.ConnectionError as e:
            last_exc = e

        elapsed = time.time() - t0
        if elapsed >= max_wait_s:
            raise LlamaServerUnreachableError(
                f"Impossibile raggiungere llama-server su '{url}' dopo "
                f"{max_wait_s:.0f}s. Assicurati di averlo avviato: "
                f"`ros2 launch llm llm.launch.py` (o `supervisor_bringup."
                f"launch.py enable_intelligence:=true`)."
            ) from last_exc
        time.sleep(min(delay, max_wait_s - elapsed))
        delay = min(delay * 2, 5.0)


def _completion(system_prompt: str, command_text: str):
    """POST verso /completion e ritorna il PRIMO valore JSON valido nella risposta.

    Estratta da get_plan_from_llm() perche' il percorso v2 manda un prompt
    diverso e si aspetta un oggetto invece di una lista, ma il CONTRATTO DELLA
    RICHIESTA e' lo stesso e deve restarlo: prompt grezzo (nessun ruolo
    system/user, nessun templating ChatML anche se il modello ne porta uno),
    nessun vincolo grammar/json_schema, temperature 0.0, n_predict come tetto.
    Duplicarlo per il secondo percorso avrebbe significato due spelling della
    stessa richiesta che divergono al primo tuning.

    raw_decode() invece di json.loads(): parsa SOLO il primo valore JSON valido
    e ignora quello che il modello genera dopo -- bug riprodotto, il modello
    continua oltre la "}" di chiusura con altre coppie comando:/risposta:
    allucinate e json.loads() rifiutava l'intera risposta con "Extra data"
    anche quando il piano, fino a quel punto, era perfettamente valido.
    """
    prompt = system_prompt + f'\ncomando: "{command_text}"\nrisposta:'
    try:
        response = requests.post(
            LLAMA_URL,
            json={'prompt': prompt, 'n_predict': 512, 'temperature': 0.0},
            timeout=LLAMA_TIMEOUT,
        )
    except requests.exceptions.ConnectionError as e:
        raise RuntimeError(
            "Impossibile contattare llama-server (era raggiungibile "
            "all'avvio -- e' stato riavviato o e' crashato?)."
        ) from e
    response.raise_for_status()
    raw = response.json().get('content', '').strip()
    try:
        parsed, end_index = json.JSONDecoder().raw_decode(raw)
    except ValueError as e:
        # La STESSA eccezione, stesso tipo e stesso messaggio, ri-sollevata:
        # le si appende soltanto il testo che il modello ha davvero prodotto.
        # Senza questo, una risposta non parsabile e' l'unico fallimento di
        # cui non resta traccia di cosa sia stato detto -- il chiamante vede
        # "Expecting value: line 1 column 1" e nient'altro. Letto da
        # _emit_result() (campo llm_raw) e da nessun altro.
        e.llm_raw = raw
        raise
    trailing = raw[end_index:].strip()
    if trailing:
        logging.getLogger(__name__).debug(
            f'_completion: scartati {len(trailing)} caratteri dopo il '
            f'JSON valido: {trailing!r}')
    return parsed


def get_plan_from_llm(command_text: str) -> list:
    """Traduce un comando -> lista di fasi, via llama-server's /completion.

    STESSO pattern di richiesta di llm_mpc_tuner_node.ask_llm() (Step 1 di
    questo pass, letto direttamente da quel file prima di scrivere questa
    funzione, non assunto): prompt grezzo (nessun ruolo system/user, nessun
    templating ChatML anche se il modello ne porta uno incorporato -- vedi
    LLAMA_URL sopra), nessun vincolo grammar/json_schema, solo prompting +
    temperature bassa per la determinismo, `n_predict` a fare da tetto
    invece che nessun limite. SYSTEM_PROMPT gia' incorpora due esempi
    completi (few-shot), quindi appendere semplicemente il comando nello
    stesso identico formato di quegli esempi ("comando: ...\\nrisposta:") e'
    sufficiente perche' il modello prosegua il pattern -- stesso principio
    che rende ask_llm() affidabile senza ChatML sull'altro modello qwen2
    di questo stack.

    Ritorna una lista (array nudo di fasi) o solleva eccezione -- STESSO
    contratto di prima (Ollama/OpenAI-client), fallback-parsing invariato:
    accetta sia un array nudo sia {"plan": [...]} sia (difensivamente)
    qualsiasi altro campo del dict che risulti una lista.

    Parsing JSON via json.JSONDecoder().raw_decode(), non json.loads() --
    parsa SOLO il primo valore JSON valido nella risposta e ignora
    qualsiasi testo il modello generi dopo, invece di rifiutare l'intera
    risposta con "Extra data" quando il modello continua oltre la "}" di
    chiusura (es. altre coppie comando:/risposta: allucinate). Cambia SOLO
    come viene fatto il parsing -- accetta ancora esattamente le stesse
    forme di sopra, non aggiunge ne' rimuove alcuna forma accettata.

    ConnectionError intercettata a parte (pass readiness/warm-up, vedi
    modulo docstring): a differenza di _wait_for_llama_server() sopra
    (chiamato una volta all'avvio), qui il server ERA raggiungibile al
    momento dell'avvio -- una ConnectionError durante il funzionamento
    normale significa che e' stato riavviato o e' crashato a meta'
    sessione, un caso diverso da "non ancora partito" e merita un
    messaggio diverso. Qualsiasi altro errore (HTTP, JSON malformato, ecc.)
    e' invariato: si propaga cosi' com'e' fino al catch-all generico di
    process_command().
    """
    parsed = _completion(SYSTEM_PROMPT, command_text)

    # UNA RISPOSTA v2 NON DEVE MAI ENTRARE QUI. L'intent v2 e' un oggetto con
    # "plan" e "unsupported", e il ramo dict qui sotto lo accetterebbe
    # volentieri estraendone la lista "plan" -- producendo una missione
    # plausibile da un prompt con semantica diversa. I due percorsi non si
    # mescolano: vedi PLANNER_PATHS, dove l'accoppiamento prompt/traduttore e'
    # dichiarato una volta sola.
    if isinstance(parsed, dict) and 'plan' in parsed and 'unsupported' in parsed:
        raise ValueError(
            'risposta in formato intent v2 ricevuta sul percorso legacy: '
            'prompt e traduttore non sono accoppiati (vedi PLANNER_PATHS)')

    if isinstance(parsed, list):
        return parsed
    if isinstance(parsed, dict):
        if isinstance(parsed.get("plan"), list):      # chiave attesa, prima di tutto
            return parsed["plan"]
        for v in parsed.values():
            if isinstance(v, list):
                return v
        raise ValueError(f"risposta senza lista di fasi: {parsed!r}")
    raise ValueError(f"formato inatteso: {parsed!r}")


def get_intent_from_llm(command_text: str, system_prompt: str, feedback=None) -> dict:
    """Comando -> intent v1 (schemas/intent_v1.json), percorso v2.

    Ritorna l'oggetto COSI' COM'E', senza estrarne la lista "plan": la
    validazione contro lo schema e la traduzione sono compito di
    plan_translate.translate(), che rifiuta qualsiasi forma inattesa per nome.
    Nessuna normalizzazione dei numeri come stringa (normalize_plan): lo schema
    richiede `type: number` e un modello che manda "3.0" deve essere corretto
    con un retry, non silenziosamente aggiustato.

    `feedback` e' il testo d'errore del validatore del tentativo precedente,
    appeso al comando per il retry (vedi LLMPlannerNode._plan_v2).
    """
    text = command_text
    if feedback:
        text = (f'{command_text}\n(il tentativo precedente e\' stato rifiutato: '
                f'{feedback}. Correggi e rispondi di nuovo solo con il JSON.)')
    parsed = _completion(system_prompt, text)
    if not isinstance(parsed, dict):
        raise ValueError(f'intent atteso come oggetto JSON, ricevuto {type(parsed).__name__}')
    return parsed


def normalize_plan(plan):
    """Converte i numeri arrivati come stringa. Fatto PRIMA della validazione:
    l'LLM a volte scrive "3.0" invece di 3.0, ed e' recuperabile."""
    if not isinstance(plan, list):
        return plan, []
    fixes = []
    out = []
    for i, ph in enumerate(plan):
        if not isinstance(ph, dict):
            out.append(ph)
            continue
        ph = dict(ph)
        for key in ("thresh", "turn_sign", "stop_at", "stop_at_distance"):
            if key in ph and isinstance(ph[key], str):
                try:
                    ph[key] = float(ph[key].strip().split()[0].replace(",", "."))
                    fixes.append(f"fase {i}: '{key}' era stringa -> {ph[key]}")
                except (ValueError, IndexError):
                    pass
        out.append(ph)
    return out, fixes


def validate_plan(plan):
    """Valida il piano contro il vocabolario ESATTO dell'LLM planner.
    Ritorna (ok, motivo_errore, lista_warning)."""
    warns = []

    if not isinstance(plan, list) or len(plan) == 0:
        return False, "il piano non e' un array di fasi non vuoto", warns

    for i, ph in enumerate(plan):
        if not isinstance(ph, dict):
            return False, f"fase {i} non e' un oggetto", warns

        if ph.get("mode") not in VALID_MODES:
            return False, (f"fase {i}: mode '{ph.get('mode')}' non valido "
                           f"(ammessi {list(VALID_MODES)})"), warns

        guard = ph.get("guard")
        if guard not in VALID_GUARDS:
            return False, (f"fase {i}: guard '{guard}' non valido "
                           f"(ammessi {list(VALID_GUARDS)})"), warns

        # --- thresh deve essere NUMERICO
        if "thresh" not in ph:
            return False, f"fase {i}: manca 'thresh'", warns
        if isinstance(ph["thresh"], bool) or not isinstance(ph["thresh"], (int, float)):
            return False, (f"fase {i}: 'thresh' deve essere un numero, "
                           f"ricevuto {ph['thresh']!r}"), warns

        lo, hi = THRESH_RANGE[guard]
        if not (lo <= float(ph["thresh"]) <= hi):
            warns.append(f"fase {i}: thresh={ph['thresh']} fuori dal range "
                         f"plausibile per '{guard}' ({lo}-{hi})")

        # --- wall_turn SENZA turn_sign
        if ph["mode"] == "wall_turn":
            if "turn_sign" not in ph:
                return False, (f"fase {i}: 'wall_turn' senza 'turn_sign'; girerebbe "
                               f"a caso"), warns
            if float(ph["turn_sign"]) not in (-1.0, 1.0):
                return False, (f"fase {i}: turn_sign deve essere -1.0 (destra) "
                               f"o +1.0 (sinistra), ricevuto {ph['turn_sign']!r}"), warns

    # --- stop_at solo sull'ultima fase
    for i, ph in enumerate(plan[:-1]):
        for key in ("stop_at", "stop_at_distance"):
            if key in ph:
                return False, (f"fase {i}: '{key}' e' ammesso solo sull'ultima "
                               f"fase; su una fase intermedia il robot si ferma "
                               f"li' e non prosegue"), warns

    # --- l'ultima fase DEVE avere una condizione di arresto
    #
    # ERRORE, non piu' warning. Era un warning quando ogni fase veniva
    # comunque tradotta in un goal_distance con un tetto finito (50 m): il
    # piano restava eseguibile, solo con una fine arbitraria. Con lo schema
    # 3.0 ogni fase diventa un drive OPEN-ENDED -- mpc_corr non termina mai da
    # solo, per progetto -- quindi un'ultima fase senza stop_at produce
    # letteralmente una missione che non si ferma mai. E' esattamente la
    # REGOLA STRUTTURALE 3 del SYSTEM_PROMPT ("altrimenti il robot non si
    # ferma mai"), che ora ha una conseguenza reale e va fatta rispettare.
    last = plan[-1]
    if "stop_at" not in last and "stop_at_distance" not in last:
        return False, ("l'ultima fase non ha ne' 'stop_at' ne' 'stop_at_distance': "
                       "con i moti 'drive' dello schema 3.0 il controllore non "
                       "termina mai da solo, quindi il robot non si fermerebbe "
                       "mai"), warns

    return True, "", warns


# describe(plan) -- RIMOSSA in questo pass, non sostituita da un alias.
#
# Rendeva le FASI DELL'LLM, e aveva una mappatura sua che era rimasta
# indietro: sul comando "vai dritto, al muro gira a destra e fermati a due
# metri dal muro" mostrava "gira a destra" per l'ultima fase mentre il JSON
# emetteva mode "straight", perche' il traduttore mappa deliberatamente una
# continuazione post-svolta su "straight" (MPC_corr ri-ancora
# psi_init_corridor a ogni move). Due layer con due mappature, uno dei quali
# stantio.
#
# describe_mission() sotto rende la MISSIONE TRADOTTA -- lo stesso documento
# che va alla macchina -- quindi il disaccordo non e' corretto, e' impossibile.
# Tenere in giro la vecchia funzione con la mappatura sbagliata avrebbe solo
# invitato a richiamarla.


# L'ACCOPPIAMENTO PROMPT <-> TRADUTTORE, DICHIARATO UNA VOLTA SOLA.
#
# Le due meta' non si mescolano mai, e non possono: ogni voce qui nomina il
# prompt E il traduttore, e LLMPlannerNode sceglie la voce una volta sola
# all'avvio (self._planner_path) invece di passare un prompt a un traduttore
# scelto altrove. Le due combinazioni incrociate falliscono comunque, per
# nome, prima di produrre qualcosa di plausibile:
#
#   risposta v2 -> phases_to_mission : get_plan_from_llm() rifiuta un oggetto
#       con "plan"+"unsupported" invece di estrarne la lista (l'avrebbe
#       accettata, producendo una missione da un prompt con altra semantica)
#   risposta legacy -> translate()   : una lista nuda non e' `type: object`,
#       IntentSchemaError
#
# 'v2' e' il default: e' il percorso in cui il modello non puo' inventare
# velocita', timeout, magnitudine di svolta o identificativi.
PLANNER_PATHS = ('legacy', 'v2')
DEFAULT_PLANNER_PATH = 'v2'
PLANNER_PATH_SPEC = {
    'legacy': {'prompt': 'SYSTEM_PROMPT (inline, questo file)',
               'validation': 'validate_plan()',
               'translator': 'phases_to_mission()'},
    'v2': {'prompt': f'prompts/{INTENT_PROMPT_FILENAME}',
           'validation': 'schemas/intent_v1.json',
           'translator': 'translate()'},
}

# Quanti tentativi EXTRA concedere al modello quando l'intent non passa la
# validazione, ri-alimentando il testo d'errore del validatore. Due: un
# modello che sbaglia la forma tre volte di fila non la azzecchera' alla
# quarta, e ogni tentativo costa una generazione intera.
MAX_INTENT_RETRIES = 2


def _describe_stop_condition(stop: dict) -> str:
    """Una stop_condition v3.0 -> testo leggibile, con valore E unita'."""
    t = stop.get('type')
    if t == 'front_clearance':
        # Il nome del tipo e' un residuo: legge /perception/front_distance,
        # la distanza dal MURO con gli oggetti rilevati esclusi. Dirlo qui,
        # perche' "clearance" a schermo farebbe pensare all'opposto.
        text = f'muro entro {stop["distance"]} m'
        if stop.get('debounce_ticks', 1) > 1:
            text += f' per {stop["debounce_ticks"]} tick consecutivi'
        return text
    if t == 'obstacle_distance_below':
        dove = 'davanti' if stop.get('forward_only') else 'in qualsiasi direzione'
        return f'oggetto {dove} entro {stop["distance"]} m'
    if t == 'distance_reached':
        return f'percorsi {stop["distance"]} m'
    if t == 'orientation_delta':
        return f'ruotato di {stop["value"]} gradi'
    if t == 'time_elapsed':
        return f'trascorsi {stop.get("duration_sec")} s'
    if t == 'object_reached':
        return 'oggetto raggiunto alla distanza indicata sopra'
    return f'{t} {json.dumps({k: v for k, v in stop.items() if k != "type"})}'


def describe_mission(mission: dict) -> str:
    """Riassunto leggibile della MISSIONE TRADOTTA, non del piano dell'LLM.

    PERCHE' DAL DOCUMENTO TRADOTTO E NON DALL'INTENT. Il vecchio describe()
    rendeva le fasi dell'LLM, e le due rappresentazioni potevano dissentire:
    sul comando "vai dritto, al muro gira a destra e fermati a due metri dal
    muro" il display mostrava "gira a destra" per l'ultima fase mentre il JSON
    emetteva mode "straight" -- perche' il traduttore mappa deliberatamente
    una fase di continuazione post-svolta su "straight" (MPC_corr ri-ancora
    psi_init_corridor a ogni move, vedi plan_translate). Il display aveva una
    mappatura sua, ferma a una versione precedente.

    Derivare il riassunto dallo STESSO documento che va alla macchina elimina
    la possibilita' del disaccordo, invece di correggerla una volta.

    Mostra per ogni move: modo, velocita', condizione d'arresto con valore e
    unita', timeout e terminale. Velocita' e timeout mancavano del tutto dal
    vecchio display ed erano esattamente cio' che il percorso legacy ometteva
    in silenzio.
    """
    lines = []
    for i, move in enumerate(mission.get('moves', [])):
        drive = move.get('drive', {})
        mode = drive.get('mode')
        target = move.get('go_to_object')
        if target is not None:
            # Not a drive step at all: without this branch it rendered as
            # "vai dritto" with the controller's default speed.
            azione = (f'vai verso "{target["target_class"]}" (il piu\' vicino), '
                      f'fermati a {target["gap_m"]} m tra muso e bordo')
            drive = {'speed': target.get('speed', 0.0)}
        elif mode == 'wall_turn' and float(drive.get('turn_mag_deg', 0.0)) > 0.0:
            verso = 'sinistra' if float(drive.get('turn_sign', 0.0)) > 0 else 'destra'
            azione = f'gira a {verso} di {drive["turn_mag_deg"]} gradi'
        elif mode == 'wall_turn':
            azione = 'mantieni la direzione (wall_turn, magnitudine 0)'
        else:
            azione = 'vai dritto'

        speed = float(drive.get('speed', 0.0))
        vel = f'{speed} m/s' if speed > 0.0 else 'default del controllore'

        extra = []
        if drive.get('approach_d_safe') is not None:
            extra.append(f'standoff {drive["approach_d_safe"]} m')
        if move.get('terminal'):
            extra.append('TERMINALE')

        lines.append(
            f'  {i}. {azione}\n'
            f'       id       {move.get("id")}\n'
            f'       velocita {vel}\n'
            f'       fino a   {_describe_stop_condition(move.get("stop_condition", {}))}\n'
            f'       timeout  {move.get("timeout_sec")} s -> {move.get("on_timeout")}'
            + (f'\n       note     {", ".join(extra)}' if extra else ''))
    return '\n'.join(lines)


# Cosa dire che E' supportato, sotto il banner RICHIESTA NON SUPPORTATA. Il
# testo senza go_to e' quello di prima, identico; con go_to si aggiunge solo
# la frase sull'oggetto piu' vicino di una classe.
SUPPORTED_HINT_NO_GO_TO = (
    '\nSupportato: andare dritto e fermarsi al muro, a una '
    'distanza percorsa, o prima di cio\' che si trova davanti '
    'senza nominarlo. Esempio: "vai dritto e fermati prima '
    'dell\'ostacolo".')
SUPPORTED_HINT_GO_TO = SUPPORTED_HINT_NO_GO_TO + (
    '\nSupportato anche: andare verso l\'oggetto PIU\' VICINO di una classe '
    'riconosciuta (persona, sedia, bottiglia...), non una persona per nome ne\' '
    'un esemplare scelto. Esempio: "vai dalla persona".')


def supported_hint(go_to_enabled: bool) -> str:
    """Return the "what IS supported" line printed under a refusal."""
    return SUPPORTED_HINT_GO_TO if go_to_enabled else SUPPORTED_HINT_NO_GO_TO


def go_to_enabled_default() -> bool:
    """stack_params.yaml's go_to_enabled, imported lazily (ament index)."""
    from f1tenth_params.param_defaults import get_value
    return bool(get_value('go_to_enabled'))


# Prefisso della riga di esito macchina-leggibile (vedi _emit_result).
RESULT_PREFIX = 'RESULT '

# Consumatori AGGIUNTIVI dell'esito, registrati da chi ne ha uno (il nodo
# registra il publisher /test/plan_result). La riga stampata resta identica:
# questo e' un secondo destinatario, non una sostituzione, e un sink che
# solleva non puo' impedire l'emissione ne' rompere il comando.
_RESULT_SINKS = []


def _emit_result(outcome: dict) -> None:
    """Stampa UNA riga `RESULT {json}` per comando, sempre l'ultima.

    PURAMENTE ADDITIVO: nessuna print, nessun log e nessun ritorno esistente
    cambia: questa riga si aggiunge in fondo e basta. Serve a chi osserva il
    nodo DA FUORI (la campagna di test --dry-run) per dire un fallimento LLM
    da un fallimento del traduttore senza dover fare regex su prosa italiana
    e senza mai vedere il TIPO dell'eccezione, che fino a qui non usciva.

    I campi sono FATTI che il nodo conosce con certezza -- da quale stadio
    l'esito e' uscito e quale eccezione e' stata sollevata -- non una
    tassonomia: mappare error_type su categorie proprie e' compito di chi
    consuma la riga, non di questo file.

    `status` e' uno tra: ok, llm_error, translator_error, refused, node_error.
    `refused` NON e' un errore -- e' EmptyPlanError, cioe' la risposta giusta
    a un comando ambiguo o non eseguibile (vedi plan_translate.EmptyPlanError).
    """
    print(RESULT_PREFIX + json.dumps(outcome, ensure_ascii=False, default=str),
          flush=True)
    for sink in tuple(_RESULT_SINKS):
        try:
            sink(outcome)
        except Exception as e:  # noqa: BLE001 -- un sink rotto non e' un errore
            print(f'{RESULT_PREFIX}sink fallito: {type(e).__name__}: {e}',
                  file=sys.stderr, flush=True)


def _llm_failure_outcome(exc: Exception, attempts: int) -> dict:
    """Esito per un'eccezione arrivata dalla chiamata all'LLM.

    `llm_raw` c'e' solo quando il modello ha risposto ma il testo non era
    parsabile (vedi _completion): e' esattamente il caso in cui serve.
    """
    return {
        'status': 'llm_error',
        'stage': 'llm',
        'error_type': type(exc).__name__,
        'error_message': str(exc),
        'attempts': attempts,
        'llm_raw': getattr(exc, 'llm_raw', None),
    }


class LLMPlannerNode(Node):
    """Vedi il modulo docstring per il flusso completo.

    Consegna via SERVIZI (load_mission/start_mission/abort_mission), non piu'
    via un publisher latched -- nessuna ragione quindi per tenere il processo
    vivo dopo un comando singolo non interattivo (era necessario prima solo
    per il latching TRANSIENT_LOCAL di /corridor_cmd). In modalita'
    interattiva il loop di input gira sul thread di background come prima
    (self._thread); main() pero' non chiama piu' rclpy.spin(node) sul thread
    principale in parallelo -- chiamare rclpy.spin_until_future_complete()
    per ogni servizio dal thread di background MENTRE il thread principale
    spinna lo stesso nodo sarebbe due executor concorrenti sullo stesso nodo
    (comportamento non definito in rclpy); main() invece fa
    node._thread.join() e lascia che sia process_command(), sul thread di
    background, a spinnare se stesso per ogni chiamata di servizio.

    Emendamento in-corsa (process_amendment/AMEND_PROMPT/--force-full della
    versione originaria) RIMOSSO in questo pass, non solo disattivato --
    inutile mantenerlo dietro un flag dato che il nuovo flusso a servizi non
    condivide nulla con la pubblicazione latched su /corridor_update che
    quel percorso usava. Comportamento di default ora: ogni comando e'
    trattato come una missione nuova (abort dell'eventuale missione in corso
    -> translate -> load -> start) -- vedi process_command().
    """

    def __init__(self, opts):
        super().__init__('llm_planner_node')
        self.opts = opts
        # Esito dell'ultimo comando, riempito dai percorsi di pianificazione e
        # stampato una volta da process_command() -- vedi _emit_result.
        self._outcome = None
        # Finestra REALE della chiamata all'LLM, in tempo ROS: marcata attorno
        # alla chiamata stessa (vedi _note_llm_sent/_note_llm_received), non
        # attorno a process_command(), che fa anche traduzione, scrittura file
        # e chiamate di servizio. Con i retry, 'sent' resta quello del PRIMO
        # tentativo e 'received' quello dell'ultimo: e' l'attesa che l'utente
        # ha davvero pagato per questo comando.
        self._llm_t_sent = None
        self._llm_t_received = None
        self._command_text = ''

        # llm_url/llm_timeout_sec: gli unici parametri ROS di questo nodo.
        # Default sui moduli-level LLAMA_URL/LLAMA_TIMEOUT gia' definiti in
        # cima al file (per un `ros2 run` diretto senza launch); llm.launch.py
        # sovrascrive llm_url con la porta REALE risolta per l'interrogation
        # 'planner' (vedi models.yaml/interrogations.yaml e llm.launch.py's
        # own model/interrogation pairing fix). Riassegnati qui, una volta,
        # sui globals stessi -- non su self -- perche' get_plan_from_llm()
        # resta una funzione libera con la stessa firma di prima (vedi il
        # commento di LLAMA_URL sopra per il perche'). `global` deve stare
        # PRIMA di ogni uso del nome in questa funzione (SyntaxError altrimenti
        # -- self.declare_parameter('llm_url', LLAMA_URL) sotto legge gia'
        # LLAMA_URL), quindi viene prima del declare_parameter, non dopo.
        global LLAMA_URL, LLAMA_TIMEOUT
        self.declare_parameter('llm_url', LLAMA_URL)
        self.declare_parameter('llm_timeout_sec', LLAMA_TIMEOUT)
        self.declare_parameter('planner_path', DEFAULT_PLANNER_PATH)
        # Topic dell'esito per la campagna di test (f1tenth_logger/test_campaign/).
        self.declare_parameter('test_plan_result_topic', '/test/plan_result')
        # go_to_enabled: stack_params.yaml's value, overridable per run.
        self.declare_parameter('go_to_enabled', go_to_enabled_default())
        LLAMA_URL = str(self.get_parameter('llm_url').value)
        LLAMA_TIMEOUT = float(self.get_parameter('llm_timeout_sec').value)

        # planner_path sceglie prompt + validazione + traduttore COME UN
        # BLOCCO (vedi PLANNER_PATHS). Risolto qui, una volta, prima di
        # qualsiasi richiesta.
        self._planner_path = str(self.get_parameter('planner_path').value)
        if self._planner_path not in PLANNER_PATHS:
            raise ValueError(
                f'planner_path={self._planner_path!r} non valido: '
                f'attesi {list(PLANNER_PATHS)}')

        # Il prompt viene dal FILE, non da una stringa inline, e viene letto
        # ADESSO -- prima di _wait_for_llama_server() sotto, che altrimenti
        # spenderebbe fino a 90s ad aspettare il server per poi fallire sul
        # primo comando per un file mancante. Un errore di packaging deve
        # fermare la costruzione, non la prima richiesta.
        # go_to_enabled sceglie anche il PROMPT, non solo il traduttore: con
        # false il modello riceve il prompt di prima, che non conosce go_to,
        # invece di imparare go_to e vederselo rifiutare a ogni tentativo.
        self._go_to_enabled = bool(self.get_parameter('go_to_enabled').value)
        prompt_file = intent_prompt_filename(self._go_to_enabled)
        if self._planner_path == 'v2':
            try:
                self._system_prompt = load_intent_prompt(prompt_file)
            except OSError as e:
                raise RuntimeError(
                    f'planner_path=v2 ma il prompt non e\' leggibile: {e}. '
                    f'Atteso prompts/{prompt_file} nel sorgente o '
                    'nella share directory installata del pacchetto llm.') from e
        else:
            self._system_prompt = SYSTEM_PROMPT

        spec = PLANNER_PATH_SPEC[self._planner_path]
        prompt_desc = (f'prompts/{prompt_file}' if self._planner_path == 'v2'
                       else spec['prompt'])
        self.get_logger().info(
            f'planner_path={self._planner_path} -- prompt {prompt_desc}, '
            f'validazione {spec["validation"]}, traduttore {spec["translator"]}, '
            f'go_to_enabled={self._go_to_enabled}')

        # Readiness + warm-up check (pass readiness/warm-up, vedi modulo
        # docstring) -- PRIMA di qualsiasi altra cosa in questo __init__,
        # cosi' che ne' il ramo interattivo (self._thread, sotto) ne' un
        # comando one-shot (chiamato da main() DOPO che il costruttore
        # ritorna, quindi dopo questo punto per costruzione) possano mai
        # raggiungere get_plan_from_llm() prima che il server sia
        # verificato raggiungibile e scaldato. Solleva
        # LlamaServerUnreachableError (mai propagata oltre main(), vedi
        # quella funzione) se il server non risponde entro max_wait_s --
        # NIENTE viene costruito dopo questo punto in quel caso (nodo
        # inutilizzabile senza backend).
        elapsed = _wait_for_llama_server(LLAMA_URL)
        self.get_logger().info(f'Server raggiunto e scaldato in {elapsed:.1f}s.')

        self.load_client = self.create_client(LoadMission, '/mission/load_mission')
        self.start_client = self.create_client(Trigger, '/mission/start_mission')
        self.abort_client = self.create_client(Trigger, '/mission/abort_mission')

        # /test/plan_result: un messaggio per comando, riuscito o fallito,
        # per il logger della campagna (f1tenth_logger/test_campaign/). Registrato come
        # SINK di _emit_result invece di essere chiamato dai percorsi di
        # pianificazione, cosi' che valga per ogni esito senza dover elencare
        # i punti di uscita -- esattamente il motivo per cui la riga RESULT
        # sta li' e non in process_command().
        self.test_plan_pub = self.create_publisher(
            String, str(self.get_parameter('test_plan_result_topic').value), 10)
        _RESULT_SINKS.append(self._publish_plan_result)

        self.get_logger().info(
            f'LLM Planner (llama-server @ {LLAMA_URL}) pronto -- consegna via '
            '/mission/load_mission + /mission/start_mission.'
        )

        self._stop = False
        self._thread = None
        if opts.interactive:
            self.get_logger().info('Scrivi un comando e premi invio (Ctrl-D per uscire).')
            self._thread = threading.Thread(target=self._input_loop, daemon=True)
            self._thread.start()

    def _ros_now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _note_llm_sent(self):
        """Marca l'invio del prompt. Solo il PRIMO tentativo di un comando."""
        if self._llm_t_sent is None:
            self._llm_t_sent = self._ros_now()

    def _note_llm_received(self):
        """Marca la risposta. Ogni tentativo: vince l'ultimo."""
        self._llm_t_received = self._ros_now()

    def _publish_plan_result(self, outcome: dict):
        """Esito -> /test/plan_result (sezione 16a del brief test-logging).

        `status` e' ok/error soltanto: la distinzione fine (llm_error,
        translator_error, refused, node_error) resta in `planner_status`, che
        il logger ignora ma un'analisi successiva no.

        `plan_id` e' il mission_id, cioe' la STESSA stringa che finisce nel
        file missione consegnato a /mission/load_mission: il loader la
        ripubblica come plan_id nei suoi /test/mission_event, ed e' cosi' che
        il logger verifica che quegli eventi appartengano al test aperto.
        """
        status = 'ok' if outcome.get('status') == 'ok' else 'error'
        error = ''
        if status != 'ok':
            error = ': '.join(str(x) for x in (
                outcome.get('error_type'), outcome.get('error_message')) if x)
        sent = self._llm_t_sent
        received = self._llm_t_received
        if sent is None:
            # Nessuna chiamata all'LLM e' mai partita (es. errore di nodo
            # prima): la finestra e' vuota, non inventata.
            sent = received = self._ros_now()
        elif received is None:
            received = self._ros_now()
        payload = {
            'prompt_num': self.opts.prompt_num,
            'prompt_text': self._command_text,
            'kind': self.opts.kind,
            't_prompt_sent': sent,
            't_response_received': received,
            'latency_ms': (received - sent) * 1000.0,
            'status': status,
            'error': error,
            'plan_id': outcome.get('mission_id') or '',
            'plan': outcome.get('mission'),
            'planner_status': outcome.get('status'),
            'attempts': outcome.get('attempts'),
        }
        self.test_plan_pub.publish(
            String(data=json.dumps(payload, ensure_ascii=False, default=str)))

    def _input_loop(self):
        while not self._stop:
            try:
                command = input('comando> ').strip()
            except (EOFError, KeyboardInterrupt):
                break
            if not command:
                continue
            self.run_command(command)

    # ── Servizi verso il sistema missioni reale ─────────────────────────
    def _call_trigger_service(self, client, service_name: str):
        """Chiama un servizio std_srvs/Trigger in modo sincrono (bloccante
        sul thread chiamante). Ritorna (success, message); (False, motivo)
        se il servizio non e' disponibile o non risponde entro il timeout --
        MAI un'eccezione non gestita verso process_command()."""
        if not client.wait_for_service(timeout_sec=MISSION_SERVICE_WAIT_SEC):
            return False, f'{service_name} non disponibile (nessun server attivo?)'
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(
            self, future, timeout_sec=MISSION_SERVICE_CALL_TIMEOUT_SEC)
        if not future.done() or future.result() is None:
            return False, (f'{service_name}: nessuna risposta entro '
                           f'{MISSION_SERVICE_CALL_TIMEOUT_SEC}s')
        resp = future.result()
        return bool(resp.success), str(resp.message)

    def _call_abort_mission(self):
        """Aborta l'eventuale missione in corso PRIMA di caricarne una nuova
        (vedi process_command). success=False qui e' l'esito ATTESO e
        benigno quando non c'era nulla da abortire (stato gia'
        IDLE/COMPLETE/ABORTED -- vedi /mission/abort_mission in
        mission/loader.py), quindi loggato a livello info, non error."""
        ok, message = self._call_trigger_service(self.abort_client, '/mission/abort_mission')
        if ok:
            self.get_logger().info(f'Missione precedente abortita: {message}')
        else:
            self.get_logger().info(f'Nessuna missione da abortire (o servizio assente): {message}')

    def _call_load_mission(self, path: str):
        if not self.load_client.wait_for_service(timeout_sec=MISSION_SERVICE_WAIT_SEC):
            return False, '/mission/load_mission non disponibile (nessun server attivo?)'
        req = LoadMission.Request()
        req.path = path
        future = self.load_client.call_async(req)
        rclpy.spin_until_future_complete(
            self, future, timeout_sec=MISSION_SERVICE_CALL_TIMEOUT_SEC)
        if not future.done() or future.result() is None:
            return False, ('/mission/load_mission: nessuna risposta entro '
                           f'{MISSION_SERVICE_CALL_TIMEOUT_SEC}s')
        resp = future.result()
        return bool(resp.success), str(resp.message)

    def _call_start_mission(self):
        return self._call_trigger_service(self.start_client, '/mission/start_mission')

    # ── Scrittura del file missione ─────────────────────────────────────
    def _write_mission_file(self, mission: dict) -> str:
        """Scrive la missione tradotta come file JSON sotto la cartella
        installata di f1tenth_behavior (missions/llm_generated/), la stessa
        radice da cui MissionLoader risolve mission_file_name -- in una
        sottocartella dedicata cosi' da non mescolarsi/confondersi con le
        missioni scritte a mano nella stessa directory (nessuna convenzione
        "PACED-NAV-xxx.json" esiste in questo repo -- verificato, vedi il
        riepilogo finale). Nome file = mission_id (gia' timestampato di
        default, vedi plan_translate.phases_to_mission)."""
        out_dir = os.path.join(
            get_package_share_directory('f1tenth_behavior'), 'missions', 'llm_generated')
        os.makedirs(out_dir, exist_ok=True)
        path = os.path.join(out_dir, f'{mission["mission_id"]}.json')
        payload = json.dumps(mission, indent=2, ensure_ascii=False) + '\n'

        # MAI sovrascrivere con un contenuto DIVERSO.
        #
        # Con gli id deterministici del percorso v2 (sha1 dell'intent, non piu'
        # un timestamp) lo stesso comando produce lo STESSO nome di file, quindi
        # ri-eseguirlo e' il caso normale, non una collisione: se il contenuto
        # coincide si riusa il file e basta. Contenuto diverso sullo stesso id
        # significherebbe invece che due intent distinti hanno prodotto lo stesso
        # sha1, o che qualcosa ha scritto qui sotto senza passare di qui -- in
        # entrambi i casi perdere il file precedente in silenzio e' il male
        # peggiore. Questa directory contiene SOLO output generato: le missioni
        # scritte a mano stanno in missions/, una cartella sopra, e nulla in
        # questo repo scrive li' (verificato con un grep su tutto src/).
        if os.path.exists(path):
            with open(path, encoding='utf-8') as f:
                existing = f.read()
            if existing == payload:
                self.get_logger().info(
                    f'missione identica gia\' presente, riusata: {path}')
                return path
            raise FileExistsError(
                f'{path} esiste gia\' con un contenuto DIVERSO; mi rifiuto di '
                'sovrascriverlo. Stesso mission_id per due missioni diverse: '
                'controlla il file prima di rimuoverlo.')

        with open(path, 'w', encoding='utf-8') as f:
            f.write(payload)
        return path

    def _plan_legacy(self, command_text: str):
        """Percorso legacy: SYSTEM_PROMPT -> validate_plan -> phases_to_mission.

        INVARIATO rispetto a prima del wire-in, deliberatamente: e' il
        fallback se il percorso v2 si comporta male alla sessione alimentata,
        e un fallback che e' stato "sistemato mentre c'ero" non e' un
        fallback. Ritorna (mission, unsupported, secondi, tentativi) o None.
        """
        t0 = time.time()
        self._note_llm_sent()
        try:
            plan = get_plan_from_llm(command_text)
        except Exception as e:
            self._note_llm_received()
            self.get_logger().error(
                f'LLM: traduzione fallita ({type(e).__name__}: {e}). Nulla caricato.')
            self._outcome = _llm_failure_outcome(e, attempts=1)
            return None
        self._note_llm_received()
        dt = time.time() - t0

        plan, fixes = normalize_plan(plan)
        for f in fixes:
            self.get_logger().warn(f'corretto -> {f}')

        ok, reason, warns = validate_plan(plan)
        if not ok:
            self.get_logger().error(f'Piano invalido: {reason}. Nulla caricato.')
            self.get_logger().error(f'  (piano ricevuto: {json.dumps(plan, ensure_ascii=False)})')
            self._outcome = {
                'status': 'translator_error', 'stage': 'translator',
                'error_type': 'InvalidPlan', 'error_message': reason,
                'attempts': 1, 'llm_raw': json.dumps(plan, ensure_ascii=False),
            }
            return None
        for w in warns:
            self.get_logger().warn(w)

        try:
            mission = phases_to_mission(plan)
        except PlanTranslationError as e:
            self.get_logger().error(f'Traduzione in missione fallita: {e}. Nulla caricato.')
            self._outcome = {
                'status': 'translator_error', 'stage': 'translator',
                'error_type': type(e).__name__, 'error_message': str(e),
                'attempts': 1, 'llm_raw': json.dumps(plan, ensure_ascii=False),
            }
            return None
        return mission, (), dt, 1, ()

    def _plan_v2(self, command_text: str):
        """Percorso v2: prompt file -> intent_v1.json -> translate().

        Un intent che non passa la validazione vale fino a MAX_INTENT_RETRIES
        tentativi EXTRA, ri-alimentando al modello il testo d'errore del
        validatore. Ogni tentativo e' loggato.

        NESSUN FALLBACK SUL PERCORSO LEGACY, mai. Un downgrade silenzioso
        produrrebbe una missione da un prompt con semantica diversa e nessuno
        se ne accorgerebbe: il display direbbe comunque qualcosa di
        plausibile. Se il percorso v2 fallisce, non esce nessuna missione.
        """
        feedback = None
        unsupported_request = command_text
        t0 = time.time()
        rejections = []          # un record per tentativo rifiutato (vedi _emit_result)
        for attempt in range(1, MAX_INTENT_RETRIES + 2):
            self._note_llm_sent()
            try:
                intent = get_intent_from_llm(command_text, self._system_prompt, feedback)
            except Exception as e:
                self._note_llm_received()
                self.get_logger().error(
                    f'LLM: generazione fallita ({type(e).__name__}: {e}). Nulla caricato.')
                self._outcome = _llm_failure_outcome(e, attempts=attempt)
                self._outcome['rejections'] = rejections
                return None
            self._note_llm_received()

            try:
                result = translate(intent, go_to_enabled=self._go_to_enabled)
            except UnsupportedIntentModeError as e:
                # B3: separable from an ordinary schema rejection on purpose.
                # Getting here means the model authored a mode the prompt
                # never taught it -- a prompt-compliance failure, not a
                # malformed plan. A request the model correctly routed to
                # "unsupported" never reaches the translator at all, so the
                # two outcomes are distinguishable by grepping this tag.
                feedback = str(e)
                unsupported_request = command_text
                self.get_logger().warn(
                    f'[prompt-non-rispettato] il modello ha prodotto un mode '
                    f'non supportato (tentativo {attempt}/{MAX_INTENT_RETRIES + 1}): {e}')
                self.get_logger().warn(
                    f'  (intent ricevuto: {json.dumps(intent, ensure_ascii=False)})')
                rejections.append({'attempt': attempt, 'error_type': type(e).__name__,
                                   'error_message': str(e), 'intent': intent})
                continue
            except (IntentSchemaError, IntentRangeError) as e:
                feedback = str(e)
                self.get_logger().warn(
                    f'intent rifiutato (tentativo {attempt}/{MAX_INTENT_RETRIES + 1}): {e}')
                self.get_logger().warn(
                    f'  (intent ricevuto: {json.dumps(intent, ensure_ascii=False)})')
                rejections.append({'attempt': attempt, 'error_type': type(e).__name__,
                                   'error_message': str(e), 'intent': intent})
                continue
            except EmptyPlanError as e:
                # NON un errore del modello: e' la risposta giusta a un comando
                # ambiguo o non supportato. Un retry qui insisterebbe perche'
                # indovini.
                #
                # I due casi vanno detti in modo DIVERSO. Il prompt marca
                # l'ambiguita' vera col prefisso "ambiguo:" ("gira e vai avanti
                # un po'" -- manca la direzione); tutto il resto e' una
                # richiesta che il robot non sa fare (andare verso un oggetto
                # nominato). Stamparli entrambi come "comando ambiguo" fa
                # sembrare guasto il planner proprio quando ha fatto la cosa
                # giusta, ed e' il primo messaggio che l'operatore legge.
                items = list(intent.get('unsupported', ()))
                ambiguous = [i for i in items if i.strip().lower().startswith('ambiguo')]
                # `refused`, non un errore: vedi _emit_result. I due sotto-casi
                # (ambiguo / non eseguibile) restano distinti come nelle print.
                self._outcome = {
                    'status': 'refused', 'stage': 'translator',
                    'error_type': type(e).__name__, 'error_message': str(e),
                    'attempts': attempt, 'ambiguous': bool(ambiguous),
                    'unsupported': items, 'rejections': rejections,
                    'llm_raw': json.dumps(intent, ensure_ascii=False),
                }
                if items and not ambiguous:
                    print('\nRICHIESTA NON SUPPORTATA -- il robot non sa farlo, '
                          'e non e\' un errore del planner:')
                    for item in items:
                        print(f'  - {item}')
                    print(supported_hint(self._go_to_enabled))
                    self.get_logger().error(
                        'richiesta non supportata; nessuna missione emessa.')
                    return None
                print('\nnessun piano eseguibile -- il comando e\' ambiguo o '
                      'interamente non esprimibile:')
                for item in items:
                    print(f'  - {item}')
                self.get_logger().error('Nessuna missione emessa.')
                return None
            except TranslatorOutputError as e:
                # Bug NOSTRO, non dell'utente: il documento va loggato intero.
                self.get_logger().error(f'BUG DEL TRADUTTORE: {e}')
                self.get_logger().error(
                    'documento rifiutato:\n'
                    + json.dumps(e.mission, indent=2, ensure_ascii=False))
                self._outcome = {
                    'status': 'translator_error', 'stage': 'translator',
                    'error_type': type(e).__name__, 'error_message': str(e),
                    'attempts': attempt, 'rejections': rejections,
                    'llm_raw': json.dumps(intent, ensure_ascii=False),
                }
                return None

            self._outcome = {
                'status': 'ok', 'stage': 'translator',
                'error_type': None, 'error_message': None,
                'attempts': attempt, 'rejections': rejections,
                'llm_raw': json.dumps(intent, ensure_ascii=False),
            }
            return (result.mission, result.unsupported, time.time() - t0, attempt,
                    result.notes)

        # B1: the retry budget terminates in an explicit UNSUPPORTED outcome
        # carrying the operator's original words, not in a bare "planning
        # failed". There is deliberately no fallback to the last intent that
        # validated: for a request the robot cannot perform, the nearest
        # validating plan is precisely the wrong-object substitution the
        # system prompt spends a section forbidding, and running it would
        # report success at whatever happened to be in front.
        print('\n' + '!' * 72)
        print('RICHIESTA NON SUPPORTATA -- nessuna missione emessa.')
        print(f'  richiesta: {unsupported_request}')
        print(f'  motivo:    {feedback}')
        print('!' * 72 + '\n')
        self.get_logger().error(
            f'richiesta non supportata dopo {MAX_INTENT_RETRIES + 1} tentativi: '
            f'{unsupported_request!r} (ultimo motivo: {feedback}). '
            'Nulla caricato, nessun ripiego.')
        self._outcome = {
            'status': 'translator_error', 'stage': 'translator',
            'error_type': 'RetriesExhausted', 'error_message': feedback,
            'attempts': MAX_INTENT_RETRIES + 1, 'rejections': rejections,
            'llm_raw': json.dumps(intent, ensure_ascii=False),
        }
        return None

    def run_command(self, command_text: str) -> bool:
        """process_command() piu' UNA riga RESULT finale (vedi _emit_result).

        Involucro PURAMENTE ADDITIVO: process_command() qui sotto e' esattamente
        quella di prima -- stesso nome, stesso corpo, stesso valore di ritorno,
        stesse print -- cosi' che chi la chiama direttamente (i test lo fanno,
        anche non-bound su uno stub) non veda alcuna differenza. I due punti di
        ingresso REALI, main() e _input_loop(), passano invece di qui.

        L'emissione e' in `finally` cosi' che anche un'eccezione che nessun
        except cattura (un bug del traduttore) lasci comunque una riga di esito
        prima di risalire come traceback.
        """
        self._outcome = None
        self._command_text = command_text
        self._llm_t_sent = None
        self._llm_t_received = None
        try:
            return self.process_command(command_text)
        finally:
            _emit_result(self._outcome or {
                'status': 'node_error', 'stage': 'node',
                'error_type': 'Unhandled',
                'error_message': 'nessun esito registrato (vedi stderr)',
                'attempts': None, 'llm_raw': None,
            })

    def process_command(self, command_text: str) -> bool:
        planned = (self._plan_v2(command_text) if self._planner_path == 'v2'
                   else self._plan_legacy(command_text))
        if planned is None:
            return False
        mission, unsupported, dt, attempts, notes = planned

        # Completa l'esito 'ok' gia' registrato dal pianificatore. `status`
        # resta 'ok' anche piu' sotto se load/start falliscono: dice che una
        # missione VALIDA e' stata prodotta, non che sia stata consegnata --
        # la consegna ha i suoi log ed e' fuori dallo scopo di questa riga.
        if isinstance(self._outcome, dict):
            self._outcome.update({
                'mission_id': mission.get('mission_id'),
                'schema_version': mission.get('schema_version'),
                'n_moves': len(mission.get('moves', ())),
                'plan_seconds': round(dt, 3),
                'unsupported': list(unsupported),
                'notes': list(notes),
                'mission': mission,
            })

        tentativi = f', {attempts} tentativi' if attempts > 1 else ''
        print(f'\nmissione generata in {dt:.2f}s ({self._planner_path}{tentativi}), '
              f'{len(mission["moves"])} move:')
        print(describe_mission(mission))
        print(f'\nJSON ({mission["mission_id"]}):')
        print(json.dumps(mission, indent=2, ensure_ascii=False))

        # Cio' che il traduttore ha cambiato rispetto alla richiesta (oggi: una
        # distanza da un oggetto alzata al minimo raggiungibile). Non blocca.
        for note in notes:
            print(f'\nNOTA: {note}')

        if unsupported:
            print('\n' + '!' * 72)
            print('PARTE DEL COMANDO NON E\' STATA TRADOTTA -- la missione qui sopra')
            print('e\' solo il resto. Non verra\' avviata senza una conferma esplicita.')
            for item in unsupported:
                print(f'  NON ESEGUITO: {item}')
            print('!' * 72)

        # 5) scrittura su file -- fatta ANCHE in --dry-run, per ispezione
        try:
            mission_path = self._write_mission_file(mission)
        except OSError as e:
            self.get_logger().error(f'scrittura della missione fallita: {e}. Nulla caricato.')
            # Ne' LLM ne' traduttore: la missione era valida, e' il nodo che non
            # e' riuscito a scriverla su disco.
            self._outcome = {
                'status': 'node_error', 'stage': 'node',
                'error_type': type(e).__name__, 'error_message': str(e),
                'attempts': attempts, 'mission_id': mission.get('mission_id'),
                'llm_raw': None,
            }
            return False
        print(f'\nscritta in: {mission_path}\n')
        if isinstance(self._outcome, dict):
            self._outcome['mission_path'] = mission_path

        if self.opts.dry_run:
            self.get_logger().warn('--dry-run: NON caricata sul sistema missioni')
            return False

        # Un `unsupported` non vuoto richiede SEMPRE una conferma esplicita,
        # anche senza --confirm: e' esattamente il caso in cui l'operatore
        # crede di aver chiesto qualcos'altro, e far partire il resto in
        # automatico esegue una missione che nessuno ha approvato. Senza un
        # terminale su cui chiedere (EOFError), la risposta e' no.
        if unsupported or self.opts.confirm:
            prompt_text = ('CONFERMI di voler avviare solo la parte tradotta? [s/N] '
                           if unsupported else 'caricare ed avviare? [s/N] ')
            try:
                if input(prompt_text).strip().lower() not in ('s', 'si', 'y'):
                    self.get_logger().info('annullato')
                    return False
            except EOFError:
                self.get_logger().error(
                    'conferma richiesta ma nessun terminale disponibile: nulla avviato.'
                    if unsupported else 'annullato')
                return False

        # 6) abort dell'eventuale missione in corso, poi load + start
        self._call_abort_mission()

        load_ok, load_message = self._call_load_mission(mission_path)
        if not load_ok:
            self.get_logger().error(
                f'load_mission FALLITO: {load_message}. start_mission NON chiamato.')
            return False
        self.get_logger().info(f'load_mission OK: {load_message}')

        start_ok, start_message = self._call_start_mission()
        if not start_ok:
            self.get_logger().error(f'start_mission FALLITO: {start_message}')
            return False
        self.get_logger().info(f'start_mission OK: {start_message}')
        return True

    def destroy_node(self):
        self._stop = True
        super().destroy_node()


def build_parser():
    p = argparse.ArgumentParser(
        prog='llm_planner_node',
        description='Traduce un comando in linguaggio naturale in una missione '
                    'f1tenth_behavior e la carica/avvia via /mission/load_mission '
                    '+ /mission/start_mission.')
    p.add_argument('command', nargs='*',
                   help='comando in linguaggio naturale. Se assente, modo interattivo.')
    p.add_argument('--dry-run', action='store_true',
                   help='traduce e scrive il file missione, ma non chiama i servizi')
    p.add_argument('--confirm', action='store_true',
                   help='chiede conferma prima di chiamare load_mission/start_mission')
    # Campagna di test (f1tenth_logger/test_campaign/): finiscono in /test/plan_result.
    p.add_argument('--prompt-num', type=int, default=-1, dest='prompt_num',
                   help='numero del prompt in prompts.yaml della campagna. '
                        '-1 (default) lascia che il logger risolva il test dal '
                        'TESTO esatto del comando.')
    p.add_argument('--kind', choices=('initial', 'replan'), default='initial',
                   help="'replan' se questo comando ri-pianifica una missione "
                        'gia\' in corso: il logger lo aggiunge al test aperto '
                        'invece di aprirne uno nuovo.')
    return p


def main(args=None):
    ros_free = remove_ros_args(sys.argv)[1:]
    opts = build_parser().parse_args(ros_free)
    opts.interactive = len(opts.command) == 0

    rclpy.init(args=args)
    try:
        node = LLMPlannerNode(opts)
    except LlamaServerUnreachableError as e:
        # Nessun traceback grezzo verso il terminale (pass readiness/warm-up,
        # vedi modulo docstring) -- messaggio chiaro e azionabile, uscita
        # pulita con codice non-zero. Il costruttore non e' mai arrivato a
        # costruire un nodo funzionante (fallito prima ancora di creare i
        # client dei servizi missione), quindi qui non c'e' nessun
        # node.destroy_node() da chiamare -- solo rclpy.shutdown().
        print(f'ERRORE: {e}', file=sys.stderr)
        # Stesso formato di esito degli altri percorsi (vedi _emit_result):
        # qui il nodo non esiste nemmeno, quindi la riga la stampa main().
        _emit_result({
            'status': 'llm_error', 'stage': 'llm',
            'error_type': type(e).__name__, 'error_message': str(e),
            'attempts': 0, 'llm_raw': None,
        })
        rclpy.shutdown()
        sys.exit(1)

    try:
        if opts.interactive:
            # Nessuno spin sul thread principale qui, deliberatamente: vedi
            # il docstring della classe qua sopra per perche' -- il thread
            # di background (self._thread) spinna gia' se stesso per ogni
            # chiamata di servizio dentro process_command(). join() blocca
            # solo il thread principale finche' quello di input non esce
            # (Ctrl-D/Ctrl-C).
            node._thread.join()
        else:
            node.run_command(' '.join(opts.command))
            # Nessun topic latched da tenere vivo (load_mission/start_mission
            # sono servizi sincroni, non piu' una publish fire-and-forget su
            # /corridor_cmd) -- a differenza della versione originaria, non
            # serve restare vivi dopo che process_command() e' tornato.
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
