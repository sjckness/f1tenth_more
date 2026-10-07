#!/bin/bash
# G4 (LLM): the host's llama-server (CUDA 12.6 build, mounted read-only at the
# same path) runs on the image's CUDA 12.9 + compat libraries, offloads to the
# GPU and answers /completion - with the stack's arguments (models.yaml:
# -ngl 999, port 8083, cwd = the llama.cpp root).
set -u
BIN=${LLAMA_SERVER:-/scratch/fabiocar/llama.cpp/build/bin/llama-server}
MODEL=${LLAMA_MODEL:-/home/fabiocar/projects/llm/models/qwen2.5-3b-instruct-q5_k_m.gguf}
PORT=8083
echo "== ldd (CUDA libs as resolved in the image)"
ldd "$BIN" | grep -E 'not found|cuda|cublas|libggml|libllama' | sed 's/ (0x[0-9a-f]*)//'
cd "${LLAMA_CWD:-$(dirname "$BIN")/../..}"
T0=$(date +%s.%N)
"$BIN" -m "$MODEL" --port $PORT -ngl 999 > /tmp/llama.log 2>&1 &
pid=$!
for i in $(seq 1 120); do
  curl -sf http://127.0.0.1:$PORT/health >/dev/null 2>&1 && break
  kill -0 $pid 2>/dev/null || break
  sleep 1
done
echo "== health after ${i}s ($(echo "$(date +%s.%N) - $T0" | bc) s): $(curl -s http://127.0.0.1:$PORT/health)"
T1=$(date +%s.%N)
echo "== /completion"
curl -s http://127.0.0.1:$PORT/completion -H 'Content-Type: application/json' \
  -d '{"prompt":"Q: What is the capital of Italy? Answer in one word.\nA:","n_predict":8,"temperature":0,"seed":1}' \
  | python3 -c 'import json,sys; r=json.load(sys.stdin); print(json.dumps({k:r.get(k) for k in ("content","tokens_predicted","timings")}, indent=1))'
echo "== first /completion took $(echo "$(date +%s.%N) - $T1" | bc) s"
echo "== /completion again x2 (kernels JIT-compiled by now: steady state)"
for n in 2 3; do
curl -s http://127.0.0.1:$PORT/completion -H 'Content-Type: application/json' \
  -d '{"prompt":"Q: What is the capital of France? Answer in one word.\nA:","n_predict":64,"temperature":0,"seed":1,"cache_prompt":false}' \
  | python3 -c 'import json,sys; r=json.load(sys.stdin); t=r["timings"]; print(json.dumps({"content":r["content"][:40],"prompt_per_second":round(t["prompt_per_second"],1),"predicted_per_second":round(t["predicted_per_second"],1)}))'
done
kill $pid; wait $pid 2>/dev/null
echo "== server log (device / offload lines)"
grep -a -i -E 'cuda|offload|device|error|ggml_cuda|compute capability|VRAM|failed' /tmp/llama.log | head -25
