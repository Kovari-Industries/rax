# DeepSeek serving

The simulation branches include `harness/model_serving`, an inference lifecycle
independent of the simulator and agent backend. It prepares a pinned checkpoint
and vLLM image, checks the GPU allocation, starts the service and validates the
Messages protocol. Claude Code connects through `agent_provider: custom`.

## Model and hardware

| Setting | Value |
| --- | --- |
| Model | `deepseek-ai/DeepSeek-V4.1-Flash-1017` |
| Revision | `1bae0e1088993b9b82de08f59de5be5a34e42170` |
| Image | `docker.io/vllm/vllm-openai@sha256:c1c9f6fd5c109ba7f0546a59f5b2f15fb87f64c77782e90a27b648b42a8e67c3` |
| Allocation | One node, 4 H200 GPUs, 48 CPU cores, 1 TB host memory |
| Tensor parallelism | 4 |
| Context | 1,048,576 tokens |
| Scheduling | 64 sequences, 4,096 batched tokens, 0.90 GPU memory fraction |
| Thinking | Enabled, reasoning effort 100; requests use maximum effort |
| Engram | CPU offload |
| CPU dispatch threads | 1 |

The default profile is `configs/serving/deepseek-v41-flash-1017.json`.
The checkpoint and image identities are checked before prepared artifacts are
reused. Host-memory capacity is needed for offload as well as GPU memory for the
model and KV cache.

## Prepare, serve and validate

Run these commands from a simulation branch with the host Python environment,
Hugging Face download dependencies and Singularity installed. Use a fast storage
root and a private, nonempty service key file with mode `0600`.

```bash
export RAX_RUNTIME_ROOT=/fast-disk/rax/runtime
export RAX_ARTIFACT_ROOT="$RAX_RUNTIME_ROOT"
PY=runtime/envs/host/bin/python
CFG=configs/serving/deepseek-v41-flash-1017.json
ROOT="$RAX_RUNTIME_ROOT/serving/deepseek"
KEY=/private/deepseek-service.key

$PY -m harness.model_serving prepare --config "$CFG" --root "$ROOT"
# Run inside the four-GPU allocation; keep this process alive.
$PY -m harness.model_serving serve --config "$CFG" --root "$ROOT" --key-file "$KEY"
```

Preparation downloads the pinned weights with resumable Hugging Face transfers,
converts the pinned OCI image to a SIF and writes `prepared.json` after both finish.
The serving process runs an NCCL all-reduce and pinned host-memory transfer test
on the allocated GPUs before loading vLLM. It polls `/v1/models` until the expected
served model is available, then writes `endpoint.json` with status `ready`.

From a second process, validate the ready service with an existing robot JPEG:

```bash
$PY -m harness.model_serving test --config "$CFG" --root "$ROOT" \
    --key-file "$KEY" --image /data/robot-scene.jpg
```

The test waits for readiness and checks a tool call, its returned result, image
input, thinking blocks and complete streaming responses. Results go to `smoke.json`.
The endpoint supports the Anthropic Messages protocol used by Claude Code; the
agent continues to manage its own tool loop.

For Slurm, copy `configs/serving/slurm.example.json` to a local site profile and
set the account, regular GPU partitions, GPU resource names and storage group.
Set `experiment_args` to the target task, for example `["--task", "click_mouse"]`;
benchmark-specific demonstration and evaluation defaults come from that branch.
`scripts/submit_deepseek_pipeline.py` renders preparation, serving, validation and
experiment jobs; `--submit` submits them. Scripts use `--no-requeue`. Validation
runs after serving starts, and the experiment depends on successful validation.
The service and simulator use separate GPU allocations.

## CUDA Graphs and DSpark

The three profiles under `configs/serving/h200-speed/` use the same model,
revision, container, four-GPU allocation, context and reasoning strength:

| Profile | Execution | Intended workload |
| --- | --- | --- |
| `baseline.json` | Eager, four CPU threads | Comparison reference |
| `graphs.json` | `FULL_AND_PIECEWISE` CUDA Graphs, four CPU threads | Concurrent requests |
| `graphs-dspark.json` | CUDA Graphs and five-token DSpark blocks | Serial agent requests |

DSpark uses probabilistic drafting and block rejection sampling with adaptive
verification disabled. The default serving profile uses this latency configuration.
CUDA Graph capture sizes are recorded in the JSON profiles. Compiled artifacts
are cached on fast storage and reused across variants.

## Measure and record performance

`scripts/tune_deepseek_serving.py --plan /private/tuning-plan.json` compares
variants sequentially within one allocation. A plan names prepared artifacts,
configuration files, an image, a private key file and an output root:

```json
{
  "root": "/fast-disk/rax/tuning",
  "prepared": "/fast-disk/rax/runtime/serving/deepseek/prepared.json",
  "key_file": "/private/deepseek-service.key",
  "image": "/data/robot-scene.jpg",
  "repeats": 2,
  "concurrencies": [1, 2, 4, 8],
  "variants": [
    {"name": "baseline", "config": "configs/serving/h200-speed/baseline.json"},
    {"name": "graphs", "config": "configs/serving/h200-speed/graphs.json"},
    {"name": "graphs-dspark", "config": "configs/serving/h200-speed/graphs-dspark.json"}
  ]
}
```

Each variant passes protocol checks, warms up, then measures identical short-code,
long-context image/code and concurrent-code requests. Sampling uses temperature
1, top-p 0.95 and maximum thinking. The recorder measures time to first token,
end-to-end throughput, approximate throughput after the first token and aggregate
throughput. Token counts come from server usage, including speculative chunks.

Per-variant records include the complete serving configuration, payload hashes,
input/output token counts, timings, protocol results and before/after vLLM cache
and speculative-decoding counters. `results.json` collects all variants;
`scripts/summarize_deepseek_tuning.py --root /fast-disk/rax/tuning` writes
`comparison.json` after checking paired workloads and fixed configuration.
Keep raw logs and manifests in the runtime storage; publish aggregate measurements.

A completed four-H200 comparison measured:

| Measurement | Eager | CUDA Graphs | CUDA Graphs + DSpark |
| --- | ---: | ---: | ---: |
| Short request, 512 output tokens (tokens/s) | 14.53 | 135.86–136.01 | 273.04–295.36 |
| Eight concurrent 256-token requests, aggregate tokens/s | — | 788.38 | 343.19 |

For a 71,341-input-token image/code request, first-token latency changed from
41.07 to 6.63 seconds cold and from 36.54 to 1.84 seconds warm between eager and
the latency profile. These measurements describe fixed serving requests; choose
the graph-only profile when aggregate concurrent throughput is the priority.

## Service cleanup

The lifecycle owns its GPU-check and vLLM process groups. SIGTERM or SIGINT is
forwarded to the active child; final cleanup waits for shutdown and escalates to
SIGKILL after its grace period. It records status `stopped` in `endpoint.json`.
Stop the owning serving process, or cancel its Slurm job, to release the GPUs.

The Slurm pipeline adds cleanup jobs that cancel its serving job after a failed
protocol test or when its experiment finishes. Tuning stops each variant before
starting the next, also on exceptions or termination. Prepared weights, images,
caches and benchmark records remain available for the next run.
