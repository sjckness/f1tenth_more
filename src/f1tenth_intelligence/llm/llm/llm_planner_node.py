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
from std_srvs.srv import Trigger

from llm.plan_translate import PlanTranslationError, phases_to_mission

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
THRESH_RANGE = {
    "wall": (0.2, 10.0),
    "front_object": (0.2, 10.0),
    "distance": (0.1, 50.0),
    "turned": (0.05, 6.3),      # radianti
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
    prompt = SYSTEM_PROMPT + f'\ncomando: "{command_text}"\nrisposta:'
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
    # raw_decode() invece di json.loads(): parsa SOLO il primo valore JSON
    # valido nella stringa e ignora qualsiasi cosa il modello generi dopo --
    # bug riprodotto: il modello continua oltre la "}" di chiusura con altre
    # coppie comando:/risposta: allucinate, e json.loads() rifiutava l'intera
    # risposta con "Extra data" anche quando il piano vero e proprio, fino a
    # quel punto, era perfettamente valido. Un fallimento GENUINO (JSON non
    # valido fin dall'inizio, non solo garbage in coda) solleva comunque
    # json.JSONDecodeError qui sotto -- nessun try/except aggiunto, si
    # propaga esattamente come json.loads() avrebbe fatto prima.
    parsed, end_index = json.JSONDecoder().raw_decode(raw)
    trailing = raw[end_index:].strip()
    if trailing:
        # Solo debug: non blocca nulla, ma utile per accorgersi se il modello
        # sta "andando fuori giri" (allucinando oltre il piano) piu' spesso
        # del previsto.
        logging.getLogger(__name__).debug(
            f'get_plan_from_llm: scartati {len(trailing)} caratteri dopo il '
            f'JSON valido: {trailing!r}')

    if isinstance(parsed, list):
        return parsed
    if isinstance(parsed, dict):
        if isinstance(parsed.get("plan"), list):      # chiave attesa, prima di tutto
            return parsed["plan"]
        for v in parsed.values():
            if isinstance(v, list):
                return v
        raise ValueError(f"risposta senza lista di fasi: {raw}")
    raise ValueError(f"formato inatteso: {raw}")


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

    # --- l'ultima fase deve avere una condizione di arresto
    last = plan[-1]
    if "stop_at" not in last and "stop_at_distance" not in last:
        warns.append("l'ultima fase non ha ne' 'stop_at' ne' 'stop_at_distance': "
                     "il robot non si fermera' da solo")

    return True, "", warns


def describe(plan) -> str:
    """Riassunto leggibile, per controllare a occhio prima di far partire il robot."""
    import math
    lines = []
    for i, ph in enumerate(plan):
        if ph["mode"] == "wall_turn":
            verso = "sinistra" if float(ph["turn_sign"]) > 0 else "destra"
            azione = f"gira a {verso}"
        else:
            azione = "vai dritto"

        g, th = ph["guard"], ph["thresh"]
        cond = {
            "wall": f"muro entro {th} m",
            "turned": f"ruotato di {th} rad ({math.degrees(float(th)):.0f} gradi)",
            "front_object": f"oggetto davanti entro {th} m",
            "distance": f"percorsi {th} m",
        }[g]

        extra = []
        if "stop_at" in ph:
            extra.append(f"STOP a {ph['stop_at']} m dall'oggetto")
        if "stop_at_distance" in ph:
            extra.append(f"STOP dopo {ph['stop_at_distance']} m")

        lines.append(f"  {i}. {azione:<18} fino a: {cond}"
                     + (f"   [{', '.join(extra)}]" if extra else ""))
    return "\n".join(lines)


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
        LLAMA_URL = str(self.get_parameter('llm_url').value)
        LLAMA_TIMEOUT = float(self.get_parameter('llm_timeout_sec').value)

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

    def _input_loop(self):
        while not self._stop:
            try:
                command = input('comando> ').strip()
            except (EOFError, KeyboardInterrupt):
                break
            if not command:
                continue
            self.process_command(command)

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
        with open(path, 'w', encoding='utf-8') as f:
            json.dump(mission, f, indent=2, ensure_ascii=False)
            f.write('\n')
        return path

    def process_command(self, command_text: str) -> bool:
        # 1) traduzione LLM -- INVARIATA
        t0 = time.time()
        try:
            plan = get_plan_from_llm(command_text)
        except Exception as e:
            self.get_logger().error(f'LLM: traduzione fallita ({e}). Nulla caricato.')
            return False
        dt = time.time() - t0

        # 2) normalizzazione -- INVARIATA
        plan, fixes = normalize_plan(plan)
        for f in fixes:
            self.get_logger().warn(f'corretto -> {f}')

        # 3) validazione -- INVARIATA (contratto verso il traduttore, vedi
        # plan_translate._sanity_check per perche' questo resta comunque la
        # PRIMA linea di difesa, non l'unica)
        ok, reason, warns = validate_plan(plan)
        if not ok:
            self.get_logger().error(f'Piano invalido: {reason}. Nulla caricato.')
            self.get_logger().error(f'  (piano ricevuto: {json.dumps(plan, ensure_ascii=False)})')
            return False
        for w in warns:
            self.get_logger().warn(w)

        print(f'\npiano generato in {dt:.2f}s, {len(plan)} fasi:')
        print(describe(plan))

        # 4) traduzione verso lo schema missioni reale
        try:
            mission = phases_to_mission(plan)
        except PlanTranslationError as e:
            self.get_logger().error(f'Traduzione in missione fallita: {e}. Nulla caricato.')
            return False

        print(f'\nmissione tradotta ({len(mission["moves"])} move):')
        print(json.dumps(mission, indent=2, ensure_ascii=False))

        # 5) scrittura su file -- fatta ANCHE in --dry-run, per ispezione
        mission_path = self._write_mission_file(mission)
        print(f'\nscritta in: {mission_path}\n')

        if self.opts.dry_run:
            self.get_logger().warn('--dry-run: NON caricata sul sistema missioni')
            return False

        if self.opts.confirm:
            try:
                if input('caricare ed avviare? [s/N] ').strip().lower() not in ('s', 'si', 'y'):
                    self.get_logger().info('annullato')
                    return False
            except EOFError:
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
            node.process_command(' '.join(opts.command))
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
