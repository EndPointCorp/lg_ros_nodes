#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
TopicBridge: one input topic -> handler(text) -> one output topic.

- If no handler is provided, the bridge echoes input to output.

ENV (used by main()):
  OLLAMA_URL=http://127.0.0.1:11434
  OLLAMA_SYSTEM=
  OLLAMA_MODELS=llama2         # comma list for fan-out
  AGG_MODEL=                    # optional, for synthesis
  INPUT_TOPIC=/stt
  OUTPUT_TOPIC=/llm/response
  CANDIDATE_BASE=/llm          # /llm/<model>
  PROMPT_TMPL='[SYSTEM]...\n[TRANSCRIPT START]\n{transcript}\n[TRANSCRIPT END]...'  (must have {transcript})
  AGG_SYNTH_TMPL='...{transcript}...{candidates}...'  (must have both)
  BRIDGE_NAME=vp_topic_bridge
  BRIDGE_QUEUE=16

Back-compat:
- If PROMPT_TMPL is unset but PROMPT_PREFIX/ PROMPT_SUFFIX are set, we build a template: f"{PREFIX}{{transcript}}{SUFFIX}"
"""

from __future__ import annotations
import os
import sys
import queue
import threading
import concurrent.futures
from typing import Callable, Dict, List, Optional

import requests

from visionport.node import VPNode
from visionport.vpros.models.std_msgs.msg import String


class OllamaClient:
    """
    If a VPNode is provided, each successful generate(model=...) will publish
    the candidate text to <candidate_base>/<model> (default: /llm/<model>).
    """
    def __init__(
        self,
        *,
        base_url: str = "http://127.0.0.1:11434",
        connect_timeout: float = 5.0,
        read_timeout: float = 120.0,
        default_system: Optional[str] = None,
        node: Optional[VPNode] = None,
        candidate_base: str = "/llm",
    ):
        self.base_url = base_url.rstrip("/")
        self.cto = connect_timeout
        self.rto = read_timeout
        self.default_system = (default_system or "").strip() or None

        self._node = node
        self._candidate_base = candidate_base.rstrip("/")
        self._candidate_pubs: Dict[str, Callable[[String], None]] = {}
        self._session = requests.Session()  # keep-alive, faster than new TCP per call


    @staticmethod
    def wrap_template(text: Optional[str], template: str) -> str:
        t = (text or "").strip()
        if "{transcript}" not in template:
            raise ValueError("wrap_template: template must contain {transcript}")
        return template.format(transcript=t)

    @staticmethod
    def build_template_from_prefix_suffix(prefix: str, suffix: str) -> str:
        return f"{prefix}{{transcript}}{suffix}"

    def _pub_for_model(self, model: str) -> Optional[Callable[[String], None]]:
        if not self._node:
            return None
        if model not in self._candidate_pubs:
            topic = f"{self._candidate_base}/{model}"
            self._candidate_pubs[model] = self._node.get_pub(topic, String)
        return self._candidate_pubs[model]

    def generate(
        self,
        *,
        prompt: str,
        model: str,
        system: Optional[str] = None,
        options: Optional[Dict] = None,
    ) -> str:
        payload = {"model": model, "prompt": prompt, "stream": False, "keep_alive": "2h"}
        if system or self.default_system:
            payload["system"] = system or self.default_system
        if options:
            payload["options"] = options

        r = self._session.post(f"{self.base_url}/api/generate", json=payload, timeout=(self.cto, self.rto))
        r.raise_for_status()
        text = (r.json().get("response") or "").strip()

        pub = self._pub_for_model(model)
        if pub and text:
            try:
                pub(text)
            except Exception as e:
                sys.stderr.write(f"[OllamaClient] WARN publish {model}: {e}\n")

        return text


class TopicBridge:
    """
    Single-input -> handler(text) -> single-output.
    If handler is None, echo input to output.
    """

    def __init__(
        self,
        *,
        name: str,
        input_topic: str,
        output_topic: str,
        queue_max: int = 16,
        handler: Optional[Callable[[str], str]] = None,
        node: Optional[VPNode] = None,
    ):
        self.name = name
        self.node = node or VPNode(name)
        self.input_topic = input_topic
        self.output_topic = output_topic
        self._q: "queue.Queue[str]" = queue.Queue(maxsize=queue_max)
        self._pub_out = self.node.get_pub(self.output_topic, String)
        self._handler: Optional[Callable[[str], str]] = None
        self.set_handler(handler or (lambda s: s))

        self.node.sub(self.input_topic, String, self._on_msg)

    def set_handler(self, handler: Callable[[str], str]) -> None:
        self._handler = handler

    def start(self) -> None:
        threading.Thread(target=self._worker, daemon=True).start()
        sys.stderr.write(f"[TopicBridge] up name={self.name} in={self.input_topic} out={self.output_topic}\n")
        threading.Event().wait()  # keep alive; VPNode runs MQTT loop

    def _on_msg(self, msg: String) -> None:
        try:
            self._q.put_nowait(msg.data)
        except queue.Full:
            sys.stderr.write("[TopicBridge] WARNING: dropped message (queue full)\n")

    def _worker(self) -> None:
        while True:
            text = self._q.get()
            try:
                out = self._handler(text) if self._handler else text
                if out:
                    self._pub_out(out)
                else:
                    self._pub_out(text)  # echo fallback
                    sys.stderr.write("[TopicBridge] handler returned empty; echoed input\n")
            except Exception as e:
                sys.stderr.write(f"[TopicBridge] ERROR: {e}\n")
            finally:
                self._q.task_done()


class LLMAggregator:
    """
    Callable handler:
      - wraps transcript with PROMPT_TMPL (must include {transcript}),
      - fans out to multiple models (parallel),
      - optionally synthesizes a final answer with an aggregator model using AGG_SYNTH_TMPL.
    """
    def __init__(self, *,
        client: OllamaClient,
        models: List[str],
        prompt_template: str,
        agg_model: Optional[str] = None,
        synth_template: Optional[str] = None,   # must contain {transcript} and {candidates}
        aggregator_system: Optional[str] = None,
        candidate_options: Optional[Dict] = None,
        agg_options: Optional[Dict] = None,
        max_workers: Optional[int] = None
        ):

        if not models:
            raise ValueError("LLMAggregator: models must be non-empty")
        if "{transcript}" not in prompt_template:
            raise ValueError("LLMAggregator: prompt_template must contain {transcript}")
        if synth_template and ("{transcript}" not in synth_template or "{candidates}" not in synth_template):
            raise ValueError("LLMAggregator: synth_template must contain {transcript} and {candidates}")

        self.client = client
        self.models = models
        self.prompt_template = prompt_template
        self.agg_model = (agg_model or "").strip() or None
        self.synth_template = synth_template or (
            "[SYSTEM]\nSynthesize the best single answer from the candidates for the transcript below.\n\n"
            "[TRANSCRIPT]\n{transcript}\n\n"
            "[CANDIDATES]\n{candidates}\n\n"
            "[INSTRUCTIONS]\nBe concise and actionable.\n"
        )
        self.aggregator_system = aggregator_system
        self.candidate_options = candidate_options or {}
        self.agg_options = agg_options or {}
        self.max_workers = max_workers or len(self.models)

    def __call__(self, text: str) -> str:
        prompt = self.client.wrap_template(text, self.prompt_template)

        # Fan-out in parallel
        with concurrent.futures.ThreadPoolExecutor(max_workers=self.max_workers) as pool:
            futs = {m: pool.submit(self.client.generate, prompt=prompt, model=m, options=self.candidate_options) for m in self.models}
            candidates: Dict[str, str] = {}
            for m, f in futs.items():
                try:
                    candidates[m] = f.result()
                except Exception as e:
                    sys.stderr.write(f"[LLMAggregator] model={m} ERROR: {e}\n")
                    candidates[m] = ""

        if self.agg_model:
            cand_text = "".join(f"[{m}]\n{(candidates.get(m) or '').strip()}\n" for m in self.models)
            synth_prompt = self.synth_template.format(transcript=text, candidates=cand_text)
            try:
                return self.client.generate(
                    prompt=synth_prompt,
                    model=self.agg_model,
                    system=self.aggregator_system,
                    options=self.agg_options
                )
            except Exception as e:
                sys.stderr.write(f"[LLMAggregator] aggregate ERROR: {e}\n")

        # Fallback: first non-empty candidate in declared order
        for m in self.models:
            if candidates.get(m):
                return candidates[m]
        return ""


# ----------------------------- CLI runner -----------------------------

def _env_prompt_template() -> str:
    tmpl = os.getenv("PROMPT_TMPL")
    if tmpl:
        return tmpl
    prefix = os.getenv("PROMPT_PREFIX", "[SYSTEM]\nYou are concise and precise.\n\n[TRANSCRIPT START]\n")
    suffix = os.getenv("PROMPT_SUFFIX", "\n[TRANSCRIPT END]\nReturn one short, actionable paragraph.\n")
    return OllamaClient.build_template_from_prefix_suffix(prefix, suffix)

def main():
    # Bridge config
    INPUT_TOPIC   = os.getenv("INPUT_TOPIC", "/stt")
    OUTPUT_TOPIC  = os.getenv("OUTPUT_TOPIC", "/llm/response")
    NAME          = os.getenv("BRIDGE_NAME", "vp_topic_bridge")
    QMAX          = int(os.getenv("BRIDGE_QUEUE", "16"))

    # LLM config
    OLLAMA_URL    = os.getenv("OLLAMA_URL", "http://127.0.0.1:11434")
    OLLAMA_SYSTEM = os.getenv("OLLAMA_SYSTEM", None)
    MODELS        = [m.strip() for m in os.getenv("OLLAMA_MODELS", "llama2").split(",") if m.strip()]
    AGG_MODEL     = os.getenv("AGG_MODEL") or None
    CAND_BASE     = os.getenv("CANDIDATE_BASE", "/llm")   # /llm/<model>

    # Templates
    PROMPT_TMPL   = _env_prompt_template()
    AGG_SYNTH_TMPL = os.getenv(
        "AGG_SYNTH_TMPL",
        "[SYSTEM]\nSynthesize the best single answer from the candidates for the transcript below.\n\n"
        "[TRANSCRIPT]\n{transcript}\n\n"
        "[CANDIDATES]\n{candidates}\n\n"
        "[INSTRUCTIONS]\nBe concise and actionable.\n"
    )
    AGG_SYSTEM    = os.getenv("AGG_SYSTEM", None)

    node = VPNode(NAME)

    # Client with node so it auto-publishes /llm/<model>
    client = OllamaClient(
        base_url=OLLAMA_URL,
        default_system=OLLAMA_SYSTEM,
        node=node,
        candidate_base=CAND_BASE,
    )

    if MODELS:
        handler = LLMAggregator(
            client=client,
            models=MODELS,
            prompt_template=PROMPT_TMPL,
            agg_model=AGG_MODEL,
            synth_template=AGG_SYNTH_TMPL,
            aggregator_system=AGG_SYSTEM,
        )

    # Bridge using the same node, can start its own if not passed
    bridge = TopicBridge(
        name=NAME,
        node=node,
        input_topic=INPUT_TOPIC,
        output_topic=OUTPUT_TOPIC,
        queue_max=QMAX,
        handler=handler,
    )
    bridge.start()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass

