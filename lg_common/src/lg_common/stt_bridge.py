#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Loads config from a toml file and starts a TopicBridge with a custom handler.

Usage:
  python3 -m lg_common.stt_bridge -c lg_common/scripts/stt_bridge.toml
"""
from __future__ import annotations
import argparse, sys
from pathlib import Path

from visionport.node import VPNode

try:
    import tomllib  # py311+
except ModuleNotFoundError:
    import tomli as tomllib  # pip install tomli

from lg_common.topicbridge import OllamaClient, LLMAggregator, LLM_KML, TopicBridge


DEFAULT_CONFIG_PRIMARY = Path("/media/dockers/stt_bridge.toml")
PACKAGE_CONFIG_FALLBACK = Path(__file__).resolve().parents[2] / "scripts" / "stt_bridge.toml"


def default_config_path() -> str:
    if DEFAULT_CONFIG_PRIMARY.exists():
        return str(DEFAULT_CONFIG_PRIMARY)
    return str(PACKAGE_CONFIG_FALLBACK)


def load_toml(path: str) -> dict:
    try:
        with open(path, "rb") as f:
            return tomllib.load(f)
    except OSError:
        if Path(path) == DEFAULT_CONFIG_PRIMARY and PACKAGE_CONFIG_FALLBACK.exists():
            with open(PACKAGE_CONFIG_FALLBACK, "rb") as f:
                return tomllib.load(f)
        raise
    except tomllib.TOMLDecodeError:
        if Path(path) == DEFAULT_CONFIG_PRIMARY and PACKAGE_CONFIG_FALLBACK.exists():
            with open(PACKAGE_CONFIG_FALLBACK, "rb") as f:
                return tomllib.load(f)
        raise

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-c", "--config", default=default_config_path())
    ap.add_argument("args", nargs=argparse.REMAINDER)

    args = ap.parse_args()
    vpnode_name = None
    print(args.args)
    for raw in args.args:  ## pick up ros created name
        if raw.startswith("__name:="):
            vpnode_name = raw.split(":=", 1)[1]
            print("VPnode name:", vpnode_name)

    cfg = load_toml(args.config)

    # Bridge
    b = cfg.get("bridge", {})
    name         = vpnode_name or b.get("name", "vp_topic_bridge")
    input_topic  = b.get("input_topic", "/stt")
    output_topic = b.get("output_topic", "/llm/response")
    queue_max    = int(b.get("queue_max", 16))

    # Ollama
    o = cfg.get("ollama", {})
    base_url        = o.get("base_url", "http://127.0.0.1:11434")
    system_prompt   = o.get("system")
    connect_timeout = float(o.get("connect_timeout", 5.0))
    read_timeout    = float(o.get("read_timeout", 120.0))
    candidate_base  = o.get("candidate_base", "/llm")
    keep_alive      = o.get("keep_alive", "2h")
    debug_client    = bool(o.get("debug", False))

    # Handler
    h = cfg.get("handler", {})
    handler_type   = (h.get("type", "LLM_KML") or "").strip().lower()
    models         = list(h.get("models", ["granite4"]))
    agg_model      = h.get("aggregator_model")
    agg_system     = h.get("aggregator_system")
    max_workers    = h.get("max_workers", None)
    max_chars      = h.get("max_transcript_chars", None)
    debug_handler  = bool(h.get("debug", False))
    cand_opts      = h.get("candidate_options", {})
    agg_opts       = h.get("agg_options", {})

    # Prompts
    p = cfg.get("prompts", {})
    prompt_tmpl     = p.get("prompt_template")  # must contain {transcript}
    agg_synth_tmpl  = p.get("aggregate_template")  # must contain {transcript} and {candidates}

    #LLM_KML
    k = cfg.get("LLM_KML", {})
    quick_model = k.get("quick_model", "granite4:micro-h")
    json_model = k.get("json_model", "granite4:latest")
    json_system_prompt = k.get("json_system_prompt", "You are a structured planner that outputs valid JSON. Return this one for testing: {'orbit': 'NYC', 'altitude': 2000}")
    quick_template = k.get("quick_template",  "User said:\n{transcript}\n\nRespond briefly to acknowledge receipt.")
    json_template = k.get("json_template", "Input:\n{transcript}")

    # Validate placeholders early (fail fast)
    if not prompt_tmpl or "{transcript}" not in prompt_tmpl:
        raise ValueError("prompts.prompt_template must contain {transcript}")
    if agg_synth_tmpl and ("{transcript}" not in agg_synth_tmpl or "{candidates}" not in agg_synth_tmpl):
        raise ValueError("prompts.aggregate_template must contain {transcript} AND {candidates}")

    # Init
    node = VPNode(name)

    client = OllamaClient(
        base_url=base_url,
        default_system=system_prompt,
        connect_timeout=connect_timeout,
        read_timeout=read_timeout,
        node=node,
        candidate_base=candidate_base,
        keep_alive=keep_alive,
        debug=debug_client,
    )
    KMLclient = OllamaClient(
        base_url=base_url,
        default_system=json_system_prompt,
        connect_timeout=connect_timeout,
        read_timeout=2*read_timeout,
        node=node,
        candidate_base=candidate_base,
        keep_alive=keep_alive,
        debug=debug_client,
    )

    if handler_type == "llm_agg":
        handler = LLMAggregator(
            client=client,
            models=models,
            prompt_template=prompt_tmpl,
            agg_model=agg_model,
            synth_template=agg_synth_tmpl,
            aggregator_system=agg_system,
            candidate_options=cand_opts,
            agg_options=agg_opts,
            max_workers=max_workers,
            max_transcript_chars=max_chars,
            debug=debug_handler,
        )

    elif handler_type == "llm_kml":
        handler = LLM_KML(
            client=KMLclient,
            quick_model=quick_model,
            json_model=json_model,
            quick_template=quick_template,
            json_template=json_template,
            json_callback=None,
            max_workers=max_workers or 2,
            debug=debug_handler,
        )

    elif handler_type in ("echo", ""):
        handler = lambda s: s
    else:
        raise ValueError(f"Unknown handler.type: {handler_type}")

    # fail fast if models missing
#    need = list(models) + ([agg_model] if agg_model else [])
#    client.ensure_models(need)

    bridge = TopicBridge(
        name=name,
        node=node,
        input_topic=input_topic,
        output_topic=output_topic,
        queue_max=queue_max,
        handler=handler,
    )
    bridge.start()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
