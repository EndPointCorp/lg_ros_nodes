import json
import os
from pathlib import Path
from http.server import HTTPServer, BaseHTTPRequestHandler
import threading
import inspect
import html as _html
import logging as _logging

from kml_tools import get_logger, configure_logging, env, FUNCTION_TABLE

"""
kml_tools_worker.py — Runner / Publisher / Minimal HTTP server

Depends on: kml_tools_backend.py
  from kml_tools_backend import get_logger, configure_logging, env, FUNCTION_TABLE

What this module does:
- Execute tool-call plans (list of {fn: args} OR legacy {"fn": "...", "args": {...}})
- Accumulate <gx:FlyTo> steps and feature snippets into a full KML document
- Save full KML to file, with robust parent dir creation and logging
- Publish KML documents or per-step snippets to a directory for use with a KML NetworkLink
- Tiny HTTP server that accepts JSON with {"plan":[...]} or {"calls":[...]} and replies with {"kml":..., "snippets":[...]}

Environment (optional):
- KML_TOOLS_LOG_LEVEL = DEBUG|INFO|WARNING|ERROR   (default INFO)

Design notes:
- Shared logging: uses backend.get_logger("worker") so log lines include the module source.
- We only call functions exposed by backend.FUNCTION_TABLE and auto-inject `inherit_from`
  when the backend function accepts it. Otherwise we leave args untouched.
- Runner includes a <gx:Tour> wrapper by default (include_tour=True). Some clients auto-play
  the first tour; disable with include_tour=False if you don't want that behavior.
"""

log = get_logger("worker")


# --------------------------------------------------------------------------------------
# Runner
# --------------------------------------------------------------------------------------
class Runner:
    """
    Execute tool-call plans and accumulate Tour steps.

    Args:
        title (str): KML Document <name>
        include_tour (bool): Wrap steps in <gx:Tour>. Some KML clients auto-play the first tour.
        out_path (str|Path|None): If provided and auto_write=True, export_kml(out_path) after run_plan
        auto_write (bool): Write after run_plan if out_path is set
        log_path (str|Path|None): Optional path for a file handler (in addition to any root handlers)
        log_to_stdout (bool): Attach a stdout handler if none (default True)
        log_level (str|int|None): Override logging level for this run (e.g., "DEBUG")

    Returns:
        Runner
    """
    def __init__(self, title="KML Tools Run", include_tour=True,
                 out_path="/home/VP_AI.kml", auto_write=True,
                 log_path=None, log_to_stdout=True, log_level=None):
        self.title = title
        self.include_tour = bool(include_tour)
        self.flytos = []     # list of <gx:FlyTo> strings
        self.features = []   # list of Placemark/Style/etc snippets
        self.styles = []     # reserved if backend supplies styles
        self.last_result = None
        self.errors = []
        self.out_path = Path(out_path).resolve() if out_path else None
        self.auto_write = bool(auto_write)

        # Logging configuration (shared root; this just adds handlers as needed)
        if log_level is not None:
            try:
                lvl = getattr(_logging, str(log_level).upper(), _logging.INFO) if isinstance(log_level, str) else int(log_level)
                get_logger().setLevel(lvl)
            except Exception:
                pass
        if log_to_stdout:
            _ensure_stdout_handler()
        if log_path:
            _add_file_handler(Path(log_path))

        log.info("Runner init: title='%s' out_path=%s auto_write=%s", self.title, str(self.out_path) if self.out_path else None, self.auto_write)

    def add(self, result):
        """Add a tool result to the run."""
        if not isinstance(result, dict):
            return
        self.last_result = result
        self.flytos.extend(result.get("flytos") or [])
        kml = result.get("kml_text")
        if kml:
            self.features.append(kml)

    def call(self, fn, args=None):
        """
        Call a function by name with args via backend.FUNCTION_TABLE.

        Behavior:
            - If the backend function has a parameter named 'inherit_from',
              we automatically pass the previous result unless the caller
              already set it.

        Returns:
            dict: backend envelope (or an error envelope)
        """
        f = FUNCTION_TABLE.get(fn)
        if not callable(f):
            err = {"errors": [f"unknown fn '{fn}'"], "kml_text": "", "flytos": []}
            log.error("worker.call failed: %s (not callable)", fn)
            self.add(err)
            return err

        args = dict(args or {})
        try:
            sig = inspect.signature(f)
            if self.last_result is not None and "inherit_from" in sig.parameters and "inherit_from" not in args:
                args["inherit_from"] = self.last_result
        except Exception:
            pass

        try:
            log.info("worker.step %s label=%s", fn, args.get("label"))
            out = f(**args)
            if out.get("errors"):
                log.warning("worker.step %s errors: %s", fn, out["errors"])
            self.add(out)
            log.debug("worker.step %s flytos+=%d features+=%s", fn, len(out.get("flytos", [])), bool(out.get("kml_text")))
            return out
        except Exception as e:
            log.exception("worker.step %s exception: %s", fn, e)
            err = {"errors": [str(e)], "kml_text": "", "flytos": []}
            self.add(err)
            return err

    def run_plan(self, plan):
        """
        Execute a plan.

        Supports two formats:
          - Legacy dicts: {"fn":"flyto_lookat","args":{...}}
          - Canonical one-key dicts: [{"flyto_lookat": {...}}, {"kml_point": {...}}]

        Returns:
            dict: last step's result (or {} if empty)
        """
        log.info("worker.plan steps=%d", len(plan or []))
        out = None
        for step in plan or []:
            if isinstance(step, dict) and "fn" in step:
                out = self.call(step.get("fn", ""), step.get("args", {}))
            elif isinstance(step, dict) and len(step) == 1:
                fn, args = next(iter(step.items()))
                out = self.call(fn, args)
            else:
                self.errors.append(f"invalid plan step: {step}")
                log.error("worker.plan invalid step: %s", step)

        if self.auto_write and self.out_path:
            self.export_kml(self.out_path)

        return out or {}

    def _assemble_document(self):
        """Build the full KML document string."""
        steps_concat = "".join(self.flytos)
        tour = f"<gx:Tour><name>Auto Tour</name><gx:Playlist>{steps_concat}</gx:Playlist></gx:Tour>" if (self.include_tour and steps_concat) else ""
        features_xml = "".join(self.features)
        return f"""<?xml version='1.0' encoding='UTF-8'?>
<kml xmlns='http://www.opengis.net/kml/2.2' xmlns:gx='http://www.google.com/kml/ext/2.2'>
  <Document><name>{_esc(self.title)}</name>{tour}{features_xml}</Document>
</kml>"""

    def export_kml(self, path=None):
        """
        Return the assembled KML document string and optionally write it to disk.

        Args:
            path (str|Path|None): destination file

        Returns:
            str: full KML document text
        """
        doc = self._assemble_document()
        if path:
            p = Path(path)
            p.parent.mkdir(parents=True, exist_ok=True)
            p.write_text(doc, encoding="utf-8")
            log.info("worker.export wrote %s (%d bytes)", str(p), len(doc.encode("utf-8")))
        return doc


# --------------------------------------------------------------------------------------
# NetworkLink publisher
# --------------------------------------------------------------------------------------
class NetworkLinkPublisher:
    """
    Write KML docs or per-step snippets to a directory intended for a served KML NetworkLink.

    Args:
        out_dir (str|Path): directory to write into
        maintain_latest (bool): create/refresh a 'latest.kml' symlink to the newest full doc
    """
    def __init__(self, out_dir, maintain_latest=True):
        self.out_dir = Path(out_dir)
        self.out_dir.mkdir(parents=True, exist_ok=True)
        self.maintain_latest = bool(maintain_latest)

    def write_full(self, kml_text, name="tour"):
        """
        Write a full KML document.

        Returns:
            str: path to the written file
        """
        p = self.out_dir / f"{name}.kml"
        p.write_text(kml_text, encoding="utf-8")
        log.info("worker.publish full %s", p)
        if self.maintain_latest:
            try:
                latest = self.out_dir / "latest.kml"
                if latest.exists() or latest.is_symlink():
                    latest.unlink()
                latest.symlink_to(p.name)
                log.info("worker.publish updated symlink %s -> %s", latest, p.name)
            except Exception as e:
                log.warning("worker.publish symlink failed: %s", e)
        return str(p)

    def write_snippet(self, snippet_text, name=None):
        """
        Write a snippet (single Placemark or FlyTo list) to its own file.

        Returns:
            str: path to the written file
        """
        name = name or "snippet"
        base = self._unique_basename(name, suffix=".kml")
        p = self.out_dir / base
        p.write_text(snippet_text, encoding="utf-8")
        log.info("worker.publish snippet %s", p)
        return str(p)

    def write_snippets_from_runner(self, runner, prefix="step"):
        """
        Write every FlyTo step from a Runner as its own snippet file.

        Returns:
            list[str]: paths written
        """
        written = []
        width = max(3, len(str(len(runner.flytos))))
        for idx, ft in enumerate(runner.flytos, 1):
            name = f"{prefix}-{idx:0{width}d}"
            written.append(self.write_snippet(ft, name=name))
        return written

    def _unique_basename(self, stem, suffix=".kml"):
        """Generate a unique filename in out_dir using an incrementing counter."""
        i = 1
        while True:
            candidate = f"{stem}-{i}{suffix}" if i > 1 else f"{stem}{suffix}"
            if not (self.out_dir / candidate).exists():
                return candidate
            i += 1


# --------------------------------------------------------------------------------------
# File helpers
# --------------------------------------------------------------------------------------
def _esc(x):
    """Escape text for XML."""
    return "" if x is None else _html.escape(str(x), quote=False)


def _ensure_stdout_handler():
    """Attach a stdout handler to the shared logger if none exists yet."""
    root = get_logger()  # root "kml_tools"
    if any(isinstance(h, _logging.StreamHandler) and not isinstance(h, _logging.FileHandler) for h in root.handlers):
        return
    h = _logging.StreamHandler()
    h.setLevel(root.level)
    fmt = _logging.Formatter("[%(asctime)s] %(levelname)s %(name)s: %(message)s")
    h.setFormatter(fmt)
    root.addHandler(h)


def _add_file_handler(path):
    """Add a file handler to the shared logger."""
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    fh = _logging.FileHandler(p, encoding="utf-8")
    fh.setLevel(get_logger().level)
    fmt = _logging.Formatter("[%(asctime)s] %(levelname)s %(name)s: %(message)s")
    fh.setFormatter(fmt)
    get_logger().addHandler(fh)
    log.info("worker logging to %s", str(p))


def load_plan_file(path):
    """
    Load a plan JSON or JSONL file.

    Behavior:
        - .json     => returns [plan] (single list of steps)
        - .jsonl    => returns [plan1, plan2, ...] (one JSON per line)

    Returns:
        list[list]: list of plans
    """
    p = Path(path)
    txt = p.read_text(encoding="utf-8")
    if p.suffix.lower() == ".jsonl":
        return [json.loads(line) for line in txt.splitlines() if line.strip()]
    return [json.loads(txt)]


def run_plans_to_files(plan_paths, out_dir, include_tour=True):
    """
    Run one or many plan files and write full KML documents for each plan.

    Returns:
        list[str]: paths to the written files
    """
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    written = []
    for plan_path in plan_paths:
        plans = load_plan_file(plan_path)
        for idx, plan in enumerate(plans, 1):
            r = Runner(title=Path(plan_path).stem, include_tour=include_tour)
            r.run_plan(plan)
            out_path = out_dir / f"{Path(plan_path).stem}-{idx}.kml"
            r.export_kml(out_path)
            written.append(str(out_path))
    return written


# --------------------------------------------------------------------------------------
# Minimal HTTP server
# --------------------------------------------------------------------------------------
class ToolCallHandler(BaseHTTPRequestHandler):
    """
    Accept JSON POST:
      - {"title": "...", "include_tour": true|false, "plan":[{...}, ...]}
      - {"title": "...", "include_tour": true|false, "calls":[{"fn":"...","args":{...}}, ...]}
    Responds with:
      - 200: {"kml": "<full doc>", "snippets": ["<gx:FlyTo>..", ...], "features": ["<Placemark>..", ...]}
      - 400: {"error": "..."} for invalid payload
      - 500: {"error": "..."} on server error
    """

    server_version = "kml_tools_worker/1.0"

    def _send(self, code, payload):
        data = json.dumps(payload).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_POST(self):
        try:
            ln = int(self.headers.get("Content-Length", "0"))
            body = self.rfile.read(ln)
            js = json.loads(body.decode("utf-8"))
            include_tour = bool(js.get("include_tour", True))
            r = Runner(title=js.get("title", "KML Tools Server"), include_tour=include_tour)

            if "plan" in js and isinstance(js["plan"], list):
                r.run_plan(js["plan"])
                kml = r.export_kml()  # not writing, just assembling
                self._send(200, {"kml": kml, "snippets": r.flytos, "features": r.features})
                log.info("worker.server handled plan with %d steps", len(js["plan"]))
                return

            if "calls" in js and isinstance(js["calls"], list):
                for call in js["calls"]:
                    if isinstance(call, dict) and "fn" in call:
                        r.call(call.get("fn", ""), call.get("args", {}))
                    elif isinstance(call, dict) and len(call) == 1:
                        fn, args = next(iter(call.items()))
                        r.call(fn, args)
                self._send(200, {"kml": r._assemble_document(), "snippets": r.flytos, "features": r.features})
                log.info("worker.server handled %d calls", len(js["calls"]))
                return

            self._send(400, {"error": "invalid payload (expected 'plan' or 'calls')"})
        except Exception as e:
            log.exception("worker.server error: %s", e)
            self._send(500, {"error": str(e)})


def serve(host="127.0.0.1", port=8000, include_tour=True):
    """
    Start the server (blocking).

    Args:
        host (str): interface to bind
        port (int): port to listen on
        include_tour (bool): default tour wrapping for new requests
    """
    # include_tour is carried by each request; kept here for parity with Runner defaults
    log.info("worker.server starting on http://%s:%d", host, port)
    HTTPServer((host, port), ToolCallHandler).serve_forever()


# --------------------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------------------
if __name__ == "__main__":
    import argparse

    ap = argparse.ArgumentParser(description="kml_tools_worker: run plans, export/publish KML, or serve an HTTP endpoint")
    ap.add_argument("--log-level", default=os.getenv("KML_TOOLS_LOG_LEVEL", "INFO"), help="root log level (default from env)")
    ap.add_argument("--log-file", default=None, help="optional log file path")

    sub = ap.add_subparsers(dest="cmd")

    runp = sub.add_parser("run", help="Run plan JSON/JSONL and write full KML docs")
    runp.add_argument("--out", required=True, help="Output directory for KML files")
    runp.add_argument("--no-tour", action="store_true", help="Do not wrap steps in <gx:Tour>")
    runp.add_argument("plans", nargs="+", help="Plan file(s): .json or .jsonl")

    pub = sub.add_parser("publish", help="Run a single plan and publish full doc + per-step snippets")
    pub.add_argument("--out", required=True, help="Directory for NetworkLink files")
    pub.add_argument("--name", default="tour", help="Base filename for the full KML (default: tour)")
    pub.add_argument("--no-tour", action="store_true", help="Do not wrap steps in <gx:Tour>")
    pub.add_argument("plan", help="Plan file (.json or .jsonl with one line; first plan is used)")

    srv = sub.add_parser("serve", help="Start a tiny HTTP server that accepts tool_call lists")
    srv.add_argument("--host", default="127.0.0.1")
    srv.add_argument("--port", type=int, default=8000)

    args = ap.parse_args()

    # Configure shared logging once for the root "kml_tools" logger
    configure_logging(level=args.log_level, log_path=args.log_file)

    if args.cmd == "run":
        paths = run_plans_to_files(args.plans, args.out, include_tour=not args.no_tour)
        print(json.dumps({"written": paths}, indent=2))

    elif args.cmd == "publish":
        plans = load_plan_file(args.plan)
        if not plans:
            raise SystemExit("no plan found")
        plan = plans[0]
        r = Runner(title=Path(args.plan).stem, include_tour=not args.no_tour)
        r.run_plan(plan)
        nl = NetworkLinkPublisher(args.out)
        full_path = nl.write_full(r._assemble_document(), name=args.name)
        snippets = nl.write_snippets_from_runner(r, prefix=f"{args.name}-step")
        print(json.dumps({"full": full_path, "snippets": snippets}, indent=2))

    elif args.cmd == "serve":
        serve(host=args.host, port=args.port)

    else:
        ap.print_help()

