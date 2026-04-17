#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
TopicBridge: one input topic -> handler(text) -> one output topic.
"""
from __future__ import annotations

import concurrent.futures
import json
import queue
import requests
import sys
import threading
import time
from typing import Callable, Dict, List, Optional

from visionport.node import VPNode
from visionport.vpros.models.std_msgs.msg import String


def _log(msg: str) -> None:
    print(msg, file=sys.stderr, flush=True)


class OllamaClient:
    def __init__(
        self,
        *,
        base_url: str = "http://127.0.0.1:11434",
        connect_timeout: float = 5.0,
        read_timeout: float = 120.0,
        default_system: Optional[str] = None,
        node: Optional[VPNode] = None,
        candidate_base: str = "/llm",
        keep_alive: Optional[str] = None,
        debug: bool = False,
    ):
        self.base_url = base_url.rstrip("/")
        self.cto = connect_timeout
        self.rto = read_timeout
        self.default_system = (default_system or "").strip() or None
        self.keep_alive = keep_alive
        self.debug = debug
        self._node = node
        self._candidate_base = candidate_base.rstrip("/")
        self._candidate_pubs: Dict[str, Callable[[String], None]] = {}
        self._session = requests.Session()

    @staticmethod
    def wrap_template(text: Optional[str], template: str) -> str:
        transcript = (text or "").strip()
        if "{transcript}" not in template:
            raise ValueError("wrap_template: template must contain {transcript}")
        return template.format(transcript=transcript)

    @staticmethod
    def build_template_from_prefix_suffix(prefix: str, suffix: str) -> str:
        return f"{prefix}{{transcript}}{suffix}"

    def list_models(self) -> set[str]:
        response = self._session.get(f"{self.base_url}/api/tags", timeout=(self.cto, 10))
        response.raise_for_status()
        return {m.get("model") for m in response.json().get("models", []) if m.get("model")}

    def ensure_models(self, names: List[str]) -> None:
        have = self.list_models()
        missing = [name for name in names if name and name not in have]
        if missing:
            raise RuntimeError(
                f"Ollama missing models: {missing}. Pull them: `ollama pull {' '.join(missing)}`"
            )

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
        payload = {"model": model, "prompt": prompt, "stream": False}
        if self.keep_alive:
            payload["keep_alive"] = self.keep_alive
        if system or self.default_system:
            payload["system"] = system or self.default_system
        if options:
            payload["options"] = options

        if self.debug:
            _log(
                f"[OllamaClient] generate model={model} keep_alive={self.keep_alive} "
                f"opts={list(options.keys()) if options else []} prompt_len={len(prompt)}"
            )

        response = self._session.post(
            f"{self.base_url}/api/generate", json=payload, timeout=(self.cto, self.rto)
        )
        response.raise_for_status()
        text = (response.json().get("response") or "").strip()

        publisher = self._pub_for_model(model)
        if publisher and text:
            try:
                publisher(text)
            except Exception as exc:
                _log(f"[OllamaClient] WARN publish {model}: {exc}")

        return text


class TopicBridge:
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
        self._handler: Callable[[str], str] = handler or (lambda text: text)

        self.node.sub(self.input_topic, String, self._on_msg)
        _log(f"[TopicBridge] up name={self.name} in={self.input_topic} out={self.output_topic}")

    def set_handler(self, handler: Callable[[str], str]) -> None:
        self._handler = handler

    def start(self) -> None:
        threading.Thread(target=self._worker, daemon=True).start()
        threading.Event().wait()

    def _on_msg(self, msg: String) -> None:
        try:
            self._q.put_nowait(msg.data)
        except queue.Full:
            _log("[TopicBridge] WARNING: dropped message (queue full)")

    def _worker(self) -> None:
        while True:
            text = self._q.get()
            try:
                out = self._handler(text) if self._handler else text
                self._pub_out(out or text)
            except Exception as exc:
                _log(f"[TopicBridge] ERROR: {exc}")
            finally:
                self._q.task_done()


class LLM_KML:
    def __init__(
        self,
        *,
        client: OllamaClient,
        quick_model: str,
        json_model: str,
        quick_template: str = "User said:\n{transcript}\n\nRespond briefly to acknowledge receipt.",
        json_template: str = "Input:\n{transcript}",
        max_workers: int = 2,
        json_callback: Optional[Callable[[Dict], None]] = None,
        debug: bool = False,
    ):
        if "{transcript}" not in quick_template:
            raise ValueError("quick_template must contain {transcript}")
        if "{transcript}" not in json_template:
            raise ValueError("json_template must contain {transcript}")

        self.client = client
        self.quick_template = quick_template
        self.json_template = json_template
        self.quick_model = quick_model
        self.json_model = json_model
        self.json_callback = json_callback or self.handle_json_toolcalls
        self.max_workers = max_workers
        self.debug = debug

    @staticmethod
    def handle_json_toolcalls(data: Dict) -> None:
        _log(f"[LLM_KML] JSON toolcalls received: {data}")

    def __call__(self, transcript: str) -> str:
        def run_quick() -> str:
            return self.client.generate(
                model=self.quick_model,
                prompt=self.quick_template.format(transcript=transcript),
            )

        def run_json() -> str:
            return self.client.generate(
                model=self.json_model,
                prompt=self.json_template.format(transcript=transcript),
            )

        with concurrent.futures.ThreadPoolExecutor(max_workers=self.max_workers) as executor:
            quick_future = executor.submit(run_quick)
            json_future = executor.submit(run_json)
            quick_reply = quick_future.result()
            json_raw = json_future.result()

        if self.debug:
            _log(f"[LLM_KML] Quick reply: {quick_reply}")
            _log(f"[LLM_KML] JSON raw output: {json_raw}")

        try:
            parsed = json.loads(json_raw)
            self.json_callback(parsed)
        except Exception as exc:
            _log(f"[LLM_KML] JSON parsing failed: {exc}")

        return quick_reply


class LLMAggregator:
    def __init__(
        self,
        *,
        client: OllamaClient,
        models: List[str],
        prompt_template: str,
        agg_model: Optional[str] = None,
        synth_template: Optional[str] = None,
        aggregator_system: Optional[str] = None,
        candidate_options: Optional[Dict] = None,
        agg_options: Optional[Dict] = None,
        max_workers: Optional[int] = None,
        max_transcript_chars: Optional[int] = None,
        debug: bool = False,
    ):
        if not models:
            raise ValueError("LLMAggregator: models must be non-empty")
        if "{transcript}" not in prompt_template:
            raise ValueError("LLMAggregator: prompt_template must contain {transcript}")
        if synth_template and (
            "{transcript}" not in synth_template or "{candidates}" not in synth_template
        ):
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
        self.max_workers = max_workers or max(1, len(self.models))
        self.max_transcript_chars = max_transcript_chars
        self.debug = debug

    def __call__(self, text: str) -> str:
        started = time.perf_counter()

        if self.max_transcript_chars and len(text) > self.max_transcript_chars:
            text = text[-self.max_transcript_chars:]

        prompt = self.client.wrap_template(text, self.prompt_template)

        with concurrent.futures.ThreadPoolExecutor(max_workers=self.max_workers) as pool:
            futures = {
                model: pool.submit(
                    self.client.generate,
                    prompt=prompt,
                    model=model,
                    options=self.candidate_options,
                )
                for model in self.models
            }
            candidates: Dict[str, str] = {}
            for model, future in futures.items():
                try:
                    candidates[model] = future.result()
                except Exception as exc:
                    _log(f"[LLMAggregator] model={model} ERROR: {exc}")
                    candidates[model] = ""

        final = ""
        if self.agg_model:
            candidate_text = "".join(
                f"[{model}]\n{(candidates.get(model) or '').strip()}\n" for model in self.models
            )
            synth_prompt = self.synth_template.format(transcript=text, candidates=candidate_text)
            try:
                final = self.client.generate(
                    prompt=synth_prompt,
                    model=self.agg_model,
                    system=self.aggregator_system,
                    options=self.agg_options,
                )
            except Exception as exc:
                _log(f"[LLMAggregator] aggregate ERROR: {exc}")

        if not final:
            for model in self.models:
                if candidates.get(model):
                    final = candidates[model]
                    break

        if self.debug:
            elapsed = time.perf_counter() - started
            lengths = {model: len(candidates.get(model, "")) for model in self.models}
            _log(
                f"[LLMAggregator] prompt_len={len(prompt)} cand_lens={lengths} total={elapsed:.2f}s "
                f"mw={self.max_workers} opts_cand={list(self.candidate_options.keys())} "
                f"opts_agg={list(self.agg_options.keys())}"
            )

        return final
