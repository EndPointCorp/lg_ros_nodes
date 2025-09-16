#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
TopicBridge: one input topic -> handler(text) -> one output topic.

- Single VPNode; subscribes & publishes directly.
- If no handler is provided, echo input to output.
- Handler may be:
    - a simple Callable[[str], str], or
    - a Callable[[str], tuple[str, dict[str,str]]]
      returning (final_text, candidates_by_model).

If the handler has attribute `candidate_topics: dict[str,str]` (model->topic),
TopicBridge will create publishers for those topics once and publish each
candidate after handler invocation.

ENV defaults (used by main()):
  OLLAMA_URL=http://127.0.0.1:11434
  OLLAMA_SYSTEM=
  OLLAMA_MODELS=llama2
  AGG_MODEL=
  AGG_TOPIC=/llm/candidates
  INPUT_TOPIC=/stt
  OUTPUT_TOPIC=/llm/response
  PROMPT_PREFIX='[SYSTEM]... [TRANSCRIPT START]\n'
  PROMPT_SUFFIX='\n[TRANSCRIPT END]...'
  AGG_SYNTH_TMPL='...{transcript}...{candidates}...'
  BRIDGE_NAME=vp_topic_bridge
  BRIDGE_QUEUE=16
"""

from __future__ import annotations
import os
import sys
import queue
import threading
import concurrent.futures
from typing import Callable, Dict, List, Optional, Tuple, Union

import requests

from visionport.node import VPNode
from visionport.vpros.models.std_msgs.msg import String


# ----------------------------- LLM client ------------------------------

class OllamaClient:
    def __init__(
        self,
        base_url: str = "http://127.0.0.1:11434",
        connect_timeout: float = 5.0,
        read_timeout: float = 120.0,
        default_system: Optional[str] = None,
    ):
        self.base_url = base_url.rstrip("/")
        self.cto = connect_timeout
        self.rto = read_timeout
        self.default_system = (default_system or "").strip() or None

    def wrap(self, text: Optional[str], prefix: str, suffix: str) -> str:
        t = (text or "").strip()
        return f"{prefix}{t}{suffix}"

    def generate(
        self,
        *,
        prompt: str,
        model: str,
        system: Optional[str] = None,
        options: Optional[Dict] = None,
    ) -> str:
        payload = {"model": model, "prompt": prompt, "stream": False}
        if system or self.default_system:
            payload["system"] = system or self.default_system
        if options:
            payload["options"] = options
        r = requests.post(f"{self.base_url}/api/generate", json=payload, timeout=(self.cto, self.rto))
        r.raise_for_status()
        return (r.json().get("response") or "").strip()


# ----------------------------- Bridge core -----------------------------

HandlerReturn = Union[str, Tuple[str, Dict[str, str]]]

class TopicBridge:
    """
    Generic single-input -> handler(text) -> single-output bridge.

    - If handler is None: echo input to output.
    - If handler returns a tuple (final, candidates), and if handler has
      `candidate_topics` (model->topic), the bridge publishes candidates to
      those topics.
    """

    def __init__(
        self,
        *,
        name: str = "topic_bridge",
        input_topic: str,
        output_topic: str,
        handler: Optional[Callable[[str], HandlerReturn]] = None,
        queue_max: int = 16,
    ):
        self.name = name
        self.node = VPNode(name)
        self.input_topic = input_topic
        self.output_topic = output_topic
        self.handler: Callable[[str], HandlerReturn] = handler or (lambda text: text)
        self._q: "queue.Queue[str]" = queue.Queue(maxsize=queue_max)



        # Prepare candidate publishers if the handler declares them ## TODO remove, let OllamaClient use VPNode and publish there
        self._candidate_pubs: Dict[str, Callable[[str], None]] = {}
        topics = getattr(self.handler, "candidate_topics", None)
        if isinstance(topics, dict):
            for model, topic in topics.items():
                pub_fn = self.node.get_pub(topic, String)
                self._candidate_pubs[model] = pub_fn

        self._pub_out = self.node.get_pub(self.output_topic, String)
        self.node.sub(self.input_topic, String, self._on_msg)

    def start(self) -> None:
        threading.Thread(target=self._worker, daemon=True).start()
        sys.stderr.write(
            f"[TopicBridge] up name={self.name} in={self.input_topic} out={self.output_topic}\n"
        )
        threading.Event().wait()  # keep alive; VPNode runs MQTT loop

    def _on_msg(self, msg: String) -> None:
        try:
            self._q.put_nowait(msg.data)
        except queue.Full:
            sys.stderr.write("[TopicBridge] WARNING: dropped message (queue full)\n")

    def _publish_candidates(self, candidates: Dict[str, str]) -> None:
        if not self._candidate_pubs:
            return
        for model, text in candidates.items():
            if not text:
                continue
            pub = self._candidate_pubs.get(model)
            if pub:
                try:
                    pub(text)
                except Exception as e:
                    sys.stderr.write(f"[TopicBridge] WARN candidate publish {model}: {e}\n")

    def _worker(self) -> None:
        while True:
            text = self._q.get()
            try:
                out = self.handler(text)
                if isinstance(out, tuple) and len(out) == 2:
                    final, candidates = out  # type: ignore
                    self._publish_candidates(candidates or {})
                    if final:
                        self._pub_out(final)
                    else:
                        self._pub_out(text)  # echo fallback
                        sys.stderr.write("[TopicBridge] handler returned empty final; echoed input\n")
                else:
                    final = out if isinstance(out, str) else ""
                    if final:
                        self._pub_out(final)
                    else:
                        self._pub_out(text)  # echo fallback
                        sys.stderr.write("[TopicBridge] handler returned empty; echoed input\n")
            except Exception as e:
                sys.stderr.write(f"[TopicBridge] ERROR: {e}\n")
            finally:
                self._q.task_done()


# ----------------------------- Aggregator ------------------------------

class LLMAggregator:
    """
    Callable handler that:
      - wraps transcript,
      - calls multiple models in parallel,
      - returns (final_text, candidates_by_model),
      - declares `candidate_topics` so TopicBridge can publish them.

    ctor args:
      ollama         : OllamaClient
      models         : list[str]
      agg_model      : str | None
      wrap_prefix    : str
      wrap_suffix    : str
      agg_topic_base : str (base topic for per-LLM candidates, default '/llm/candidates')
      synth_template : str (has {transcript} and {candidates})
    """
    def __init__(
        self,
        *,
        ollama: OllamaClient,
        models: List[str],
        agg_model: Optional[str],
        wrap_prefix: str,
        wrap_suffix: str,
        agg_topic_base: str = "/llm/candidates",
        synth_template: Optional[str] = None,
    ):
        if not models:
            raise ValueError("LLMAggregator: models must be non-empty")
        self.ollama = ollama
        self.models = models
        self.agg_model = (agg_model or "").strip() or None
        self.wrap_prefix = wrap_prefix
        self.wrap_suffix = wrap_suffix
        self.agg_topic_base = agg_topic_base.rstrip("/")
        self.synth_template = synth_template or (
            "[SYSTEM]\nSynthesize the best single answer from the candidates for the transcript below.\n\n"
            "[TRANSCRIPT]\n{transcript}\n\n"
            "[CANDIDATES]\n{candidates}\n\n"
            "[INSTRUCTIONS]\nBe concise and actionable.\n"
        )
        # Tell the bridge where to publish each candidate
        self.candidate_topics: Dict[str, str] = {
            m: f"{self.agg_topic_base}/{m}" for m in self.models
        }

    def __call__(self, text: str) -> Tuple[str, Dict[str, str]]:
        prompt = self.ollama.wrap(text, self.wrap_prefix, self.wrap_suffix)

        # Fan-out
        with concurrent.futures.ThreadPoolExecutor(max_workers=len(self.models)) as pool:
            futs = {m: pool.submit(self.ollama.generate, prompt=prompt, model=m) for m in self.models}
            candidates: Dict[str, str] = {}
            for m, f in futs.items():
                try:
                    candidates[m] = f.result()
                except Exception as e:
                    sys.stderr.write(f"[LLMAggregator] model={m} ERROR: {e}\n")
                    candidates[m] = ""

        # Aggregate
        final = ""
        if self.agg_model:
            cand_text = "".join(f"[{m}]\n{(candidates.get(m) or '').strip()}\n" for m in self.models)
            synth_prompt = self.synth_template.format(transcript=text, candidates=cand_text)
            try:
                final = self.ollama.generate(prompt=synth_prompt, model=self.agg_model)
            except Exception as e:
                sys.stderr.write(f"[LLMAggregator] aggregate ERROR: {e}\n")

        # Fallback: first non-empty candidate
        if not final:
            for m in self.models:
                if candidates.get(m):
                    final = candidates[m]
                    break

        return final, candidates


# ----------------------------- CLI runner -----------------------------

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
    AGG_TOPIC     = os.getenv("AGG_TOPIC", "/llm/candidates")

    # Wrapper for per-model calls
    WRAP_PREFIX   = os.getenv("PROMPT_PREFIX", "[SYSTEM]\nYou are concise and precise.\n\n[TRANSCRIPT START]\n")
    WRAP_SUFFIX   = os.getenv("PROMPT_SUFFIX", "\n[TRANSCRIPT END]\nReturn one short, actionable paragraph.\n")
    # Aggregation template (env-controlled)
    AGG_SYNTH_TMPL = os.getenv(
        "AGG_SYNTH_TMPL",
        "[SYSTEM]\nSynthesize the best single answer from the candidates for the transcript below.\n\n"
        "[TRANSCRIPT]\n{transcript}\n\n"
        "[CANDIDATES]\n{candidates}\n\n"
        "[INSTRUCTIONS]\nBe concise and actionable.\n"
    )

    ollama = OllamaClient(base_url=OLLAMA_URL, default_system=OLLAMA_SYSTEM)

    handler: Optional[Callable[[str], HandlerReturn]] = None
    if MODELS:
        handler = LLMAggregator(
            ollama=ollama,
            models=MODELS,
            agg_model=AGG_MODEL,
            wrap_prefix=WRAP_PREFIX,
            wrap_suffix=WRAP_SUFFIX,
            agg_topic_base=AGG_TOPIC,
            synth_template=AGG_SYNTH_TMPL,
        )

    bridge = TopicBridge(
        name=NAME,
        input_topic=INPUT_TOPIC,
        output_topic=OUTPUT_TOPIC,
        handler=handler,   # if None -> echo
        queue_max=QMAX,
    )
    bridge.start()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass

