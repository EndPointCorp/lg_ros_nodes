from __future__ import annotations
import os, math, time, json, html, uuid, logging
from pathlib import Path
import requests
import hashlib as _h
from typing import Any, Dict, Optional, List, Tuple, Union, Literal

"""
kml_tools_backend.py — core functions & declarations (no CLI, no server)

Environment:
- KML_TOOLS_LOG_LEVEL: DEBUG|INFO|WARNING|ERROR (default INFO)
- KML_TOOLS_OFFLINE: "1" to use cache-only (no live HTTP)
- GOOGLE_MAPS_API_KEY: optional for Google geocoding/directions
- KML_TOOLS_CACHE: cache directory (default ~/.cache/kml_tools)

Return envelope (for every function):
{
  "kml_text": str,        # Styles + KML snippets joined (optional)
  "geometries": [str],    # Placemark/Folder/etc (optional)
  "flytos": [str],        # <gx:FlyTo> steps for a Tour (optional)
  "params": {...},        # echo of important params / derived values
  "context": {...},       # arbitrary context
  "styles": [str],        # any <Style> definitions
  "ids": {...},           # id mapping if needed
  "errors": [str],        # non-fatal warnings
  "label": "..."          # human label for tracing
  "last_cam": {...}       # full camera/LookAt context when relevant
}
"""

# Logging utilities (shared across modules)
_ROOT_LOGGER_NAME = "kml_tools"
_DEFAULT_LEVEL = os.getenv("KML_TOOLS_LOG_LEVEL", "INFO").upper()

def get_logger(source: Optional[str] = None) -> logging.Logger:
    """Return a module-specific logger that shares the root handlers.

    Args:
        source: Name suffix for the logger (e.g., "backend", "worker", "tools").
    """
    name = _ROOT_LOGGER_NAME if not source else f"{_ROOT_LOGGER_NAME}.{source}"
    logger = logging.getLogger(name)
    root = logging.getLogger(_ROOT_LOGGER_NAME)
    if not root.handlers:  # Configure root once if empty
        lvl = getattr(logging, _DEFAULT_LEVEL, logging.INFO)
        root.setLevel(lvl)
        fmt = logging.Formatter("[%(asctime)s] %(levelname)s %(name)s: %(message)s")
        sh = logging.StreamHandler()
        sh.setLevel(lvl)
        sh.setFormatter(fmt)
        root.addHandler(sh)
    return logger

def configure_logging(level: Optional[Union[str,int]] = None, log_path: Optional[str] = None) -> None:
    """Configure global logging handlers and levels.

    Args:
        level: e.g., "DEBUG" or logging.DEBUG; if None, uses env.
        log_path: Optional file path for a rotating/file handler.
    """
    root = logging.getLogger(_ROOT_LOGGER_NAME)
    if level is not None:
        if isinstance(level, str):
            root.setLevel(getattr(logging, level.upper(), logging.INFO))
        else:
            root.setLevel(level)
        for h in root.handlers:
            h.setLevel(root.level)
    if log_path:
        fh = logging.FileHandler(log_path, encoding="utf-8")
        fmt = logging.Formatter("[%(asctime)s] %(levelname)s %(name)s: %(message)s")
        fh.setFormatter(fmt)
        fh.setLevel(root.level)
        root.addHandler(fh)
        root.info("File logging enabled: %s", log_path)

log = get_logger("backend")

# Config & cache
OFFLINE = os.getenv("KML_TOOLS_OFFLINE", "0") == "1"
CACHE_DIR = Path(os.getenv("KML_TOOLS_CACHE", str(Path.home()/".cache"/"kml_tools")))
CACHE_DIR.mkdir(parents=True, exist_ok=True)
GOOGLE_KEY = os.getenv("GOOGLE_MAPS_API_KEY", "").strip()

TTL_GEOCODE_S = int(os.getenv("KML_CACHE_TTL_GEOCODE_S", str(30*24*3600)))
TTL_ROUTE_S   = int(os.getenv("KML_CACHE_TTL_ROUTE_S",   str( 7*24*3600)))
TTL_OSRM_S    = int(os.getenv("KML_CACHE_TTL_OSRM_S",    str( 3*24*3600)))

HEADERS = {"User-Agent": "kml-tools/3.0"}
NOMINATIM_BASE = "https://nominatim.openstreetmap.org"
OSRM_BASE      = "https://router.project-osrm.org"

# ENV and helpers 
R = 6371000.0

def env(*,
        flytos: Optional[List[str]] = None,
        geometries: Optional[List[str]] = None,
        params: Optional[Dict[str, Any]] = None,
        context: Optional[Dict[str, Any]] = None,
        styles: Optional[List[str]] = None,
        ids: Optional[Dict[str, Any]] = None,
        errors: Optional[List[str]] = None,
        label: Optional[str] = None,
        last_cam: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    """Standard envelope dict for tool results.

    Args:
        flytos (Optional[List[str]]): Zero or more ``<gx:FlyTo>`` XML strings.
        geometries: Zero or more raw KML snippets such as ``<Placemark>``, ``<Style>``, or ``<Folder>``.
        label: Human label for tracing/logging.
        last_cam: Last camera/lookat context.

    Returns:
        Dict[str, Any]: ``{"flytos": [...], "geometries": [...], "label": label, "last_cam": {...}}``.
    """
    kml_text = "".join([f for f in [styles, geometries] if f])
    return {
        "flytos": flytos or [],
        "geometries": geometries or [],
        "params": params or {},
        "context": context or {},
        "styles": styles or [],
        "ids": ids or {},
        "errors": errors or [],
        "label": label,
        "last_cam": last_cam,
        "kml_text": kml_text,
    }

def _uuid(prefix: str = "id") -> str:
    """Return a short deterministic-ish id string.

    Args:
        prefix: Prefix label.

    Returns:
        str: e.g., "id-abc123..."
    """
    return f"{prefix}-{uuid.uuid4().hex[:10]}"

def esc(x: Any) -> str:
    """Escape text for XML.

    Args:
        x (Any)

    Returns:
        str
    """
    return "" if x is None else html.escape(str(x), quote=False)

def clamp(v: float, lo: float, hi: float) -> float:
    """Clamp value into [lo, hi]."""
    return max(lo, min(hi, v))

def norm_heading(h: float) -> float:
    """Normalize heading to [0..360)."""
    return (float(h) % 360.0 + 360.0) % 360.0

def wrap_lon(x: float) -> float:
    return (float(x) + 180.0) % 360.0 - 180.0

def angle_delta(a: float, b: float) -> float:
    """Shortest signed delta from a→b (degrees in [-180,180))."""
    return (float(b) - float(a) + 540.0) % 360.0 - 180.0

def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Great-circle distance in meters between two WGS84 points."""
    p1 = math.radians(lat1)
    p2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dphi/2)**2 + math.cos(p1)*math.cos(p2)*math.sin(dl/2)**2
    return 2*R*math.asin(math.sqrt(a))

# HTTP & cache helpers
def _cache_key(url: str, params: Optional[Dict[str, Any]]) -> str:
    items = sorted((k, str(v)) for k, v in (params or {}).items())
    raw = url + "?" + "&".join(f"{k}={v}" for k, v in items)
    return _h.sha256(raw.encode("utf-8")).hexdigest()

def _cache_path(key: str) -> Path:
    """Cache file path for key."""
    return CACHE_DIR / f"{key}.json"

def _cache_load(key: str, max_age_s: int) -> Optional[Union[dict, list]]:
    """Load cached JSON if not older than max_age_s. If max_age_s<0, allow stale."""
    p = _cache_path(key)
    if not p.exists():
        return None
    try:
        age = time.time() - p.stat().st_mtime
        if max_age_s >= 0 and age > max_age_s:
            return None
        return json.loads(p.read_text("utf-8"))
    except Exception as e:
        log.warning("backend.cache_load failed: %s", e)
        return None

def _cache_save(key: str, obj: Union[dict, list]) -> None:
    """Save JSON to cache, ignore failures."""
    p = _cache_path(key)
    try:
        p.write_text(json.dumps(obj), encoding="utf-8")
    except Exception as e:
        log.warning("backend.cache_save failed: %s", e)

def _http_get_json_cached(url: str, params: Dict[str, Any], ttl_s: int, allow_stale_on_error: bool = True) -> Optional[Union[dict, list]]:
    """GET JSON with caching. Offline returns stale if available."""
    key = _cache_key(url, params)
    data = _cache_load(key, max_age_s=ttl_s)
    if data is not None:
        log.debug("backend.http cache hit %s", url)
        return data
    if OFFLINE:
        log.info("backend.http offline -> stale %s", url)
        return _cache_load(key, max_age_s=-1)
    try:
        log.debug("backend.http GET %s params=%s", url, params)
        r = requests.get(url, params=params, headers=HEADERS, timeout=30)
        if r.status_code == 200:
            js = r.json()
            _cache_save(key, js)
            return js
        log.warning("backend.http status %s for %s", r.status_code, url)
        return _cache_load(key, max_age_s=-1) if allow_stale_on_error else None
    except Exception as e:
        log.error("backend.http error %s: %s", url, e)
        return _cache_load(key, max_age_s=-1) if allow_stale_on_error else None

# Geocoding and directions
def _google_geocode(name: str, region: Optional[str]) -> Optional[Tuple[float, float, str]]:
    """Google Geocoding.
    Returns:
        (lat, lon, display_name) | None
    """
    if not GOOGLE_KEY:
        return None
    js = _http_get_json_cached("https://maps.googleapis.com/maps/api/geocode/json",
                               {"address": name, "region": region or "", "key": GOOGLE_KEY},
                               ttl_s=TTL_GEOCODE_S)
    if js and isinstance(js, dict) and js.get("results"):
        r0 = js["results"][0]
        loc = r0["geometry"]["location"]
        disp = r0.get("formatted_address", name)
        lat, lon = float(loc["lat"]), float(loc["lng"])
        log.info("backend.geocode[google] '%s' => (%.6f, %.6f)", name, lat, lon)
        return (lat, lon, disp)
    return None

def _nominatim_geocode(name: str, country_bias: Optional[str]) -> Optional[Tuple[float, float, str]]:
    """Nominatim free geocoder.
    Returns:
        (lat, lon, display_name) | None
    """
    p = {"q": name, "format":"jsonv2", "limit": 1}
    if country_bias:
        p["countrycodes"] = str(country_bias).lower()
    js = _http_get_json_cached(f"{NOMINATIM_BASE}/search", p, ttl_s=TTL_GEOCODE_S)
    if isinstance(js, list) and js:
        it = js[0]
        lat, lon = float(it["lat"]), float(it["lon"])
        disp = it.get("display_name", name)
        log.info("backend.geocode[nominatim] '%s' => (%.6f, %.6f)", name, lat, lon)
        return (lat, lon, disp)
    return None

def _resolve_place(place: Dict[str, Any]) -> Tuple[float, float, str]:
    """Resolve place dict (name or query) through google or nominatim.
    Returns:
        tuple: (lat, lon, display_name)
    """
    if place is None:
        raise ValueError("place is None")
    if "lat" in place and "lon" in place:
        return float(place["lat"]), float(place["lon"]), place.get("name") or "Location"
    q = place.get("name") or place.get("query")
    if not q:
        raise ValueError("place requires {lat,lon} or {name/query}")
    res = _google_geocode(q, place.get("country_bias")) or _nominatim_geocode(q, place.get("country_bias"))
    if not res:
        raise ValueError(f"geocoding failed for '{q}'")
    return res

def coords_from(lon: Optional[float] = None, lat: Optional[float] = None, place: Optional[Dict[str, Any]] = None) -> Tuple[float,float,str]:
    """    Resolve input into (lat,lon,display_name).

    Args:
        lon,lat (float|None)
        place (dict|None)

    Returns:
        (lat, lon, display)
    """
    if lat is not None and lon is not None:
        return float(lat), float(lon), place.get("name", "Location") if place else "Location"
    return _resolve_place(place or {})

def _google_directions(a_lat: float, a_lon: float, b_lat: float, b_lon: float, mode: str) -> Optional[List[Tuple[float,float]]]:
    """Google Directions (overview polyline).
    Returns:
        list[(lat,lon)] or None.
    """
    if not GOOGLE_KEY:
        return None
    js = _http_get_json_cached("https://maps.googleapis.com/maps/api/directions/json",
                               {"origin": f"{a_lat:.8f},{a_lon:.8f}",
                                "destination": f"{b_lat:.8f},{b_lon:.8f}",
                                "mode": mode, "key": GOOGLE_KEY}, ttl_s=TTL_ROUTE_S)
    if js and isinstance(js, dict) and js.get("routes"):
        try:
            enc = js["routes"][0]["overview_polyline"]["points"]
            index, lat, lng, coords = 0, 0, 0, []
            while index < len(enc):
                for k in ("lat","lng"):
                    shift = result = 0
                    while True:
                        b = ord(enc[index]) - 63
                        index += 1
                        result |= (b & 0x1f) << shift
                        shift += 5
                        if b < 0x20:
                            break
                    delta = ~(result >> 1) if (result & 1) else (result >> 1)
                    if k == "lat": lat += delta
                    else: lng += delta
                coords.append((lat/1e5, lng/1e5))
            log.info("backend.route[google] %d pts, mode=%s", len(coords), mode)
            return coords
        except Exception as e:
            log.error("backend.route[google] decode failed: %s", e)
    return None

def _osrm_route(a_lat: float, a_lon: float, b_lat: float, b_lon: float, mode: str) -> Optional[List[Tuple[float,float]]]:
    """OSRM route (overview polyline).
    Returns:
        list[(lat,lon)] or None.
    """
    prof = {"driving": "driving", "walking": "foot", "cycling": "bike"}.get(mode, "foot")
    js = _http_get_json_cached(f"{OSRM_BASE}/route/v1/{prof}/{a_lon:.8f},{a_lat:.8f};{b_lon:.8f},{b_lat:.8f}",
                               {"overview": "full", "geometries": "geojson"}, ttl_s=TTL_OSRM_S)
    if js and isinstance(js, dict) and js.get("routes"):
        coords = js["routes"][0]["geometry"]["coordinates"]
        pts = [(lat, lon) for lon, lat in coords]
        log.info("backend.route[osrm] %d pts, mode=%s", len(pts), mode)
        return pts
    return None

def provider_route(a_lat: float, a_lon: float, b_lat: float, b_lon: float, mode: str) -> Optional[List[Tuple[float,float]]]:
    pts = _google_directions(a_lat, a_lon, b_lat, b_lon, mode) or _osrm_route(a_lat, a_lon, b_lat, b_lon, mode)
    if pts is None:
        log.warning("backend.route no provider -> straight segment")
    return pts

# Bearing API
def _bearing_math(lat1: float, lon1: float, lat2: float, lon2: float, *,
                  method: Literal["rhumb", "great_circle"] = "rhumb",
                  shortest_antimeridian: bool = True) -> float:
    """
    Compute initial bearing from (lat1,lon1) to (lat2,lon2).
    Returns degrees ∈ [0, 360).
    """
    φ1, φ2 = math.radians(lat1), math.radians(lat2)
    Δλ = math.radians(lon2 - lon1)

    if method == "great_circle":
        x = math.sin(Δλ) * math.cos(φ2)
        y = math.cos(φ1)*math.sin(φ2) - math.sin(φ1)*math.cos(φ2)*math.cos(Δλ)
        θ = math.degrees(math.atan2(x, y))
        return (θ + 360.0) % 360.0

    # rhumb (loxodrome)
    if shortest_antimeridian and abs(Δλ) > math.pi:
        Δλ = -(2*math.pi - Δλ) if Δλ > 0 else (2*math.pi + Δλ)
    dψ = math.log(math.tan(math.pi/4 + φ2/2) / math.tan(math.pi/4 + φ1/2))
    q  = dψ if abs(dψ) > 1e-12 else math.cos(φ1)
    θ  = math.degrees(math.atan2(Δλ, q))
    return (θ + 360.0) % 360.0

def bearing_between(*,
                    origin: Optional[Dict[str, Any]] = None,
                    destination: Optional[Dict[str, Any]] = None,
                    from_lat: Optional[float] = None,
                    from_lon: Optional[float] = None,
                    to_lat: Optional[float] = None,
                    to_lon: Optional[float] = None,
                    method: Literal["rhumb","great_circle"] = "rhumb",
                    shortest_antimeridian: bool = True,
                    label: Optional[str] = None,
                    last_cam: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    """
    TOOL: Resolve inputs and compute bearing.
    Returns env(..., params={"bearing_deg": float, "heading_deg": float, "method": str})
    """
    if origin is None and (from_lat is not None and from_lon is not None):
        origin = {"lat": float(from_lat), "lon": float(from_lon)}
    if destination is None and (to_lat is not None and to_lon is not None):
        destination = {"lat": float(to_lat), "lon": float(to_lon)}
    if origin is None or destination is None:
        return env(errors=["Provide origin & destination as places or explicit coordinates"], label=label, last_cam=last_cam)

    o_lat, o_lon, o_disp = _resolve_place(origin)
    d_lat, d_lon, d_disp = _resolve_place(destination)
    brg = _bearing_math(o_lat, o_lon, d_lat, d_lon, method=method, shortest_antimeridian=shortest_antimeridian)
    log.info("backend.bearing (%s) %s → %s: %.2f°", method, o_disp, d_disp, brg)
    return env(
        label=label,
        params={"bearing_deg": brg, "heading_deg": brg, "method": method},
        context={"from": {"lat": o_lat, "lon": o_lon, "name": o_disp},
                 "to":   {"lat": d_lat, "lon": d_lon, "name": d_disp}},
        last_cam=last_cam
    )

# -------------------------------------------------------------------------------------
# KML 
# -------------------------------------------------------------------------------------
def _line_style_xml(style_id: str, *, color: Optional[str] = None, width: Optional[float] = None) -> str:
    aabbggrr = color or "ff00ffff"
    w = width if width is not None else 3.0
    return f"""
<Style id="{esc(style_id)}">
  <LineStyle>
    <color>{esc(aabbggrr)}</color>
    <width>{w:.1f}</width>
  </LineStyle>
</Style>""".strip()

def _linestring_xml(name: str, coords_ll: List[Tuple[float,float]], *, altitudeMode: str = "clampToGround",
                    extrude: bool = False, tessellate: bool = True, style_url: Optional[str] = None) -> str:
    coords_txt = " ".join(f"{lon:.8f},{lat:.8f},0" for lat, lon in coords_ll)
    style_part = f"<styleUrl>#{esc(style_url)}</styleUrl>" if style_url else ""
    return f"""
<Placemark>
  <name>{esc(name)}</name>
  {style_part}
  <LineString>
    <tessellate>{1 if tessellate else 0}</tessellate>
    <extrude>{1 if extrude else 0}</extrude>
    <altitudeMode>{esc(altitudeMode)}</altitudeMode>
    <coordinates>{coords_txt}</coordinates>
  </LineString>
</Placemark>""".strip()

def kml_polyline(name: str, points: Optional[List[Tuple[float,float]]] = None, places: Optional[List[Dict[str,Any]]] = None, *,
                 altitudeMode: str = "clampToGround",
                 extrude: bool = False,
                 tessellate: bool = True,
                 style_color: Optional[str] = None,
                 style_width: Optional[float] = None,
                 style_id: Optional[str] = None,
                 label: Optional[str] = None) -> Dict[str, Any]:
    """TOOL: Render a LineString from a path."""
    pts: List[Tuple[float,float]] = []
    if points:
        pts = [(float(lat), float(lon)) for (lat, lon) in points]
    elif places:
        for pl in places:
            la, lo, _ = _resolve_place(pl)
            pts.append((la, lo))
    else:
        return env(errors=["Provide points or places"], label=label)
    if len(pts) < 2:
        return env(errors=["Need at least two points for a polyline"], label=label)
    sid = style_id or _uuid("line")
    style_xml = _line_style_xml(sid, color=style_color, width=style_width)
    line_xml  = _linestring_xml(name, pts, altitudeMode=altitudeMode, extrude=extrude, tessellate=tessellate, style_url=sid)
    log.info("backend.kml_polyline %s: %d pts", name, len(pts))
    return env(geometries=[line_xml], styles=[style_xml], label=label or f"polyline {name}")

def kml_circle(name: str,
               lat: float,
               lon: float,
               radius_m: float,
               *,
               steps: int = 96,
               altitudeMode: str = "clampToGround",
               extrude: bool = False,
               tessellate: bool = True,
               style_color: Optional[str] = None,
               style_width: Optional[float] = None,
               style_id: Optional[str] = None,
               label: Optional[str] = None) -> Dict[str, Any]:
    """TOOL: Render a Circle LineString from a path."""
    steps = max(12, int(steps))
    latr = math.radians(lat)
    c = max(1e-9, math.cos(latr))
    dlat = (radius_m / R) * (180.0 / math.pi)
    dlon = (radius_m / (R * c)) * (180.0 / math.pi)
    pts: List[Tuple[float,float]] = []
    for i in range(steps+1):
        ang = 2*math.pi * (i/steps)
        la = lat + dlat * math.sin(ang)
        lo = ((lon + dlon * math.cos(ang) + 180.0) % 360.0) - 180.0
        pts.append((la, lo))
    return kml_polyline(name, points=pts, altitudeMode=altitudeMode, extrude=extrude,
                        tessellate=tessellate, style_color=style_color, style_width=style_width,
                        style_id=style_id, label=label or f"circle {name}")

def kml_point(name: str,
              lon: Optional[float] = None,
              lat: Optional[float] = None,
              place: Optional[Dict[str, Any]] = None,
              label: Optional[str] = None,
              last_cam: Optional[Dict[str,Any]] = None) -> Dict[str, Any]:
    """Create a Point ``<Placemark>`` geometry and return it in the envelope.

    Args:
        name: Placemark name.
        lon: Longitude in degrees.
        lat: Latitude in degrees.
        place: Alternative place descriptor used if ``lat/lon`` not given.
        label: Optional label for logs.

    Returns:
        Envelope with a KML Placemark in ``geometries``.
    """
    if lat is None or lon is None:
        lat, lon, _ = coords_from(place=place or {"name": name})
    xml = (
        "<Placemark>"
        f"<name>{esc(name)}</name>"
        "<Point>"
        f"<coordinates>{lon:.8f},{lat:.8f},0</coordinates>"
        "</Point>"
        "</Placemark>"
    )
    lab = label or f"point {name} @ ({lat:.6f},{lon:.6f})"
    log.info("backend.kml_point %s (%.6f, %.6f)", name, lat, lon)
    return env(geometries=[xml], label=lab, last_cam=last_cam)

def tour_wait(duration_s: float = 0.25, *, label: Optional[str] = None) -> Dict[str, Any]:
    """
    TOOL: insert a blocking pause into the tour timeline.
    """
    step = f"<gx:Wait><gx:duration>{max(float(duration_s), 0.01):.2f}</gx:duration></gx:Wait>"
    return env(flytos=[step], geometries=[], label=label or f"wait {duration_s:.2f}s")

def lookat_to_camera(lat: float, lon: float, alt: float, heading: float, tilt: float, range_m: float) -> Dict[str, float]:
    """Convert a KML LookAt target and viewing geometry into an approximate KML Camera eye position
       using a local ENU (East–North–Up) small-distance approximation.
    This approximation assumes a locally flat Earth over short distances and does
    not query terrain; the returned altitude is computed as ``alt + range_m * cos(tilt)``.

    Returns:
        Dict[str, float]: A mapping containing the approximate Camera eye position: {"lat":..., "lon":..., "alt":...}
    """

    hdg = math.radians(norm_heading(heading))
    th = math.radians(tilt)

    lat_rad = math.radians(lat)
    L = range_m * math.sin(th)   # horizontal component (meters)
    up = range_m * math.cos(th)  # vertical component (meters)
    # meters -> degrees (small-angle)
    cos_lat = max(1e-12, math.cos(lat_rad))
    dLat_deg = ((-L * math.cos(hdg)) / R) * (180.0 / math.pi)
    dLon_deg = ((-L * math.sin(hdg)) / (R * cos_lat)) * (180.0 / math.pi)
    return {"lat": lat + dLat_deg,
            "lon": wrap_lon(lon + dLon_deg),
            "alt": alt + up}

# Feature visibility controls (gx:AnimatedUpdate)
def _animated_visibility(*, feature_id: str, target_type: str = "Folder", visible: bool = True,
                         animate_s: float = 0.2) -> str:
    """Build a gx:AnimatedUpdate that toggles <visibility> for a target element.
    target_type must match the real top-level type (e.g., 'Folder' or 'Placemark').
    """
    v = 1 if visible else 0
    t = esc(target_type)
    fid = esc(feature_id)
    return f"""
<gx:AnimatedUpdate>
  <gx:duration>{max(float(animate_s), 0.0):.2f}</gx:duration>
  <Update>
    <targetHref/>
    <Change>
      <{t} targetId="{fid}">
        <visibility>{v}</visibility>
      </{t}>
    </Change>
  </Update>
</gx:AnimatedUpdate>""".strip()

def feature_visibility_on(*, feature_id: str, target_type: str = "Folder", animate_s: float = 0.2) -> str:
    """Turn a feature ON (visible=1). Place this before the steps where it should be shown."""
    return _animated_visibility(feature_id=feature_id, target_type=target_type, visible=True, animate_s=animate_s)

def feature_visibility_off(*, feature_id: str, target_type: str = "Folder", animate_s: float = 0.2) -> str:
    """Turn a feature OFF (visible=0). Place this after the steps where it should be shown."""
    return _animated_visibility(feature_id=feature_id, target_type=target_type, visible=False, animate_s=animate_s)

def feature_show_for(*, feature_id: str, target_type: str = "Folder",
                     show_animate_s: float = 0.2, hold_s: float = 2.0, hide_animate_s: float = 0.2) -> str:
    """Blocking helper: show a feature, wait for hold_s, then hide it again.
    NOTE: This pauses the tour during hold_s. Prefer on/off around movement for continuous play.
    """
    parts = [feature_visibility_on(feature_id=feature_id, target_type=target_type, animate_s=show_animate_s),
             f"<gx:Wait><gx:duration>{max(float(hold_s), 0.0):.2f}</gx:duration></gx:Wait>",
             feature_visibility_off(feature_id=feature_id, target_type=target_type, animate_s=hide_animate_s)]
    return "".join(parts)

def flyto(
    mode: Literal["lookat","camera"] = "lookat",
    lat: Optional[float] = None,
    lon: Optional[float] = None,
    place: Optional[Dict[str, Any]] = None,
    alt: Optional[float] = None,
    heading: Optional[float] = None,
    tilt: Optional[float] = None,
    range_m: Optional[float] = None,
    duration_s: float = 1.6,
    flytomode: Optional[Literal["smooth","bounce"]] = None,
    altitudeMode: Optional[Literal["clampToGround","relativeToGround","absolute"]] = None,
    last_cam: Optional[Dict[str, Any]] = None,
    label: Optional[str] = None,
) -> Dict[str, Any]:
    """Emit a single KML tour step as a ``<gx:FlyTo>`` containing either a ``<LookAt>`` or ``<Camera>`` view.
       Composes the XML for the requested view mode
       Returns:
           Envelope dict suitable for downstream aggregation/serialization, like:
           ``{"flytos": [xml_string], "label": label, "last_cam": {...}}``.
    """
    if (lat is None or lon is None) and place:
        lat, lon, _ = coords_from(place=place)
    if (lat is None or lon is None) and last_cam:
        lat, lon = last_cam.get("lat"), last_cam.get("lon")
    if lat is None or lon is None:
        return env(errors=["lat+lon or place or last_cam required"], label=label)

    alt = float(alt) if alt is not None else (last_cam.get("alt") if last_cam is not None else 0.0)
    heading = norm_heading(float(heading)) if heading is not None else (last_cam.get("heading") if (last_cam is not None and "heading" in last_cam) else 0.0)
    tilt = clamp(float(tilt), 0, 90) if tilt is not None else (last_cam.get("tilt") if last_cam is not None else 58.0)
    duration_s = max(float(duration_s), 0.1)
    flytomode = flytomode if flytomode is not None else (last_cam.get("flytomode") if last_cam is not None else "smooth")
    altitudeMode = altitudeMode if altitudeMode is not None else (last_cam.get("altitudeMode") if last_cam is not None else "relativeToGround")
    range_m = range_m if range_m is not None else (last_cam.get("range_m") if last_cam is not None else 1200)

    if mode == "lookat":
        xml = (
            "<gx:FlyTo>"
            f"<gx:duration>{duration_s:.2f}</gx:duration>"
            f"<gx:flyToMode>{esc(flytomode)}</gx:flyToMode>"
            "<LookAt>"
            f"<longitude>{lon:.8f}</longitude>"
            f"<latitude>{lat:.8f}</latitude>"
            f"<altitude>{alt:.2f}</altitude>"
            f"<heading>{heading:.2f}</heading>"
            f"<tilt>{tilt:.2f}</tilt>"
            f"<range>{range_m:.2f}</range>"
            f"<altitudeMode>{esc(altitudeMode)}</altitudeMode>"
            "</LookAt>"
            "</gx:FlyTo>"
        )
    elif mode == "camera":
        xml = (
            "<gx:FlyTo>"
            f"<gx:duration>{duration_s:.2f}</gx:duration>"
            f"<gx:flyToMode>{esc(flytomode)}</gx:flyToMode>"
            "<Camera>"
            f"<longitude>{lon:.8f}</longitude>"
            f"<latitude>{lat:.8f}</latitude>"
            f"<altitude>{alt:.2f}</altitude>"
            f"<heading>{heading:.2f}</heading>"
            f"<tilt>{tilt:.2f}</tilt>"
            f"<altitudeMode>{esc(altitudeMode)}</altitudeMode>"
            "</Camera>"
            "</gx:FlyTo>"
        )
    else:
        return env(errors=[f"unknown mode {mode!r}"], label=label)

    last = {
        "mode": mode,
        "duration_s": duration_s,
        "flytomode": flytomode,
        "lat": lat,
        "lon": lon,
        "alt": alt,
        "heading": heading,
        "tilt": tilt,
        "range_m": range_m,
        "altitudeMode": altitudeMode,
    }
    log.info("backend.flyto mode=%s step added.", mode)
    return env(flytos=[xml], label=label, last_cam=last)

def heading_turn(origin: Dict[str, Any], target: Dict[str, Any], *,
                 range_m: float = 2500.0, tilt: float = 60.0, alt: float = 0.0, duration_s: float = 2.0,
                 altitudeMode: Optional[Literal["clampToGround","relativeToGround","absolute"]] = "relativeToGround",
                 flyToMode: Literal["smooth", "bounce"] = "smooth", method: Literal["rhumb","great_circle"] = "rhumb",
                 label: Optional[str] = None, last_cam: Optional[Dict[str,Any]] = None) -> Dict[str, Any]:
    """Flyto point1 looking at point2"""
    lat1, lon1, _ = _resolve_place(origin)
    lat2, lon2, _ = _resolve_place(target)
    hdg = _bearing_math(lat1, lon1, lat2, lon2, method=("great_circle" if method == "great_circle" else "rhumb"))
    return flyto(mode="lookat", lon=lon1, lat=lat1, alt=alt, heading=hdg, tilt=tilt, range_m=range_m,
                        duration_s=duration_s, flyToMode=flyToMode, altitudeMode=altitudeMode, last_cam=last_cam, label=label)

def orbit(
    mode: Literal["lookat","camera"] = "lookat",
    lon: Optional[float] = None,
    lat: Optional[float] = None,
    place: Optional[Dict[str, Any]] = None,
    steps: int = 48,
    duration_total_s: Optional[float] = None,
    tilt: Optional[float] = 58.0,
    altitudeMode: Optional[Literal["clampToGround","relativeToGround","absolute"]] = "relativeToGround",
    heading: Optional[float] = None,
    start_heading: Optional[float] = None,
    first_step_extra_s: float = 2.0,
    sweep_degrees: float = 360.0,
    reverse: bool = False,
    range_m: Optional[float] = 1200.0,
    alt: Optional[float] = 0.0,
    *,
    display: Union[bool, Dict[str,Any]] = False,
    last_cam: Optional[Dict[str, Any]] = None,
    label: Optional[str] = None,
) -> Dict[str, Any]:
    """Generate a sequence of ``<gx:FlyTo>`` steps forming an orbit around a center point using either LookAt or Camera views.
    The orbit is parameterized by the number of steps, total duration, sweep
    angle, and direction. For ``mode == "camera"``, the camera position is derived
    from the LookAt geometry via :func:`lookat_to_camera` using a small-distance
    approximation.

    Returns:
        Dict[str, Any]: Envelope with fields like: ``{"flytos": [...], "label": label, "last_cam": {...}}``.
    Raises:
        ValueError: If center latitude/longitude cannot be determined from
            inputs (``lat/lon`` or ``place`` or ``last_cam``).
    """
    if (lat is None or lon is None) and place:
        lat, lon, _ = coords_from(place=place)
    if (lat is None or lon is None) and last_cam:
        lat, lon = last_cam.get("lat"), last_cam.get("lon")
    if (lon is None or lat is None):
        return env(errors=["orbit: center lon/lat or place (or last_cam) required"], label=label)

    tilt = tilt if tilt is not None else (last_cam.get("tilt") if last_cam is not None else 58.0)
    altitudeMode = altitudeMode if altitudeMode is not None else (last_cam.get("altitudeMode") if last_cam is not None else "relativeToGround")
    alt = alt if alt is not None else (last_cam.get("alt") if last_cam is not None else 0.0)
    range_m = range_m if range_m is not None else (last_cam.get("range_m") if last_cam is not None else 1200)
    heading = start_heading if start_heading is not None else (heading if heading is not None else (last_cam.get("heading") if (last_cam is not None and "heading" in last_cam) else 0.0))
    base = norm_heading(float(heading))

    steps = max(4, int(steps))
    total = float(duration_total_s) if duration_total_s is not None else steps * 0.33
    per = max(total, 1) / steps
    step_sign = -1.0 if reverse else 1.0
    deg_per_step = step_sign * (abs(float(sweep_degrees)) / max(1, steps))

    parts: List[str] = []
    for i in range(steps):
        hdg = norm_heading(base + (deg_per_step * i))
        dur = per + (first_step_extra_s if i == 0 else 0.0)
        if mode == "lookat":
            one = flyto(mode=mode, lat=lat, lon=lon, alt=alt, heading=hdg, tilt=tilt, range_m=range_m,
                        duration_s=dur, flytomode="smooth", altitudeMode=altitudeMode, last_cam=last_cam)
        elif mode == "camera":
            cam = lookat_to_camera(lat, lon, alt, hdg, tilt, range_m)
            one = flyto(mode=mode, lat=cam["lat"], lon=cam["lon"], alt=cam["alt"], heading=hdg, tilt=tilt,
                        range_m=range_m, duration_s=dur, flytomode="smooth", altitudeMode=altitudeMode, last_cam=last_cam)
        parts += one["flytos"]
        last_cam = one["last_cam"]

    styles: List[str] = []
    geoms: List[str] = []
    if display:
        disp_opts = display if isinstance(display, dict) else {}
        horiz_m = range_m * math.sin(math.radians(tilt))
        circ = kml_circle(disp_opts.get("name","Orbit path"), lat, lon, horiz_m, steps=max(32, steps), altitudeMode="clampToGround",
                          style_color=disp_opts.get("style_color"), style_width=disp_opts.get("style_width"))
        styles += circ["styles"]
        geoms += circ["geometries"]

    log.info("backend.orbit mode=%s start=%.2f steps=%d sweep=%.1f reverse=%s per=%.2fs (+%.1fs first)",
             mode, base, steps, sweep_degrees, reverse, per, first_step_extra_s)

    return env(flytos=parts, geometries=geoms, styles=styles, label=label, last_cam=last_cam)

# PATH/Tour split: raw route -> (optional) smoothing -> polyline -> tour
def route_path_raw(places: Optional[List[Dict[str,Any]]] = None,
                   origin: Optional[Dict[str,Any]] = None,
                   destination: Optional[Dict[str,Any]] = None,
                   mode: Literal["walking","driving","cycling"] = "walking",
                   *,
                   label: Optional[str] = None) -> Dict[str,Any]:
    """
    TOOL: fetch an unresampled path (list of (lat,lon)) for given places / origin-destination.
    Returns env(..., params={"path_ll": [(lat,lon), ...], "distance_m": float, "mode": str})
    """
    if places and len(places) >= 2:
        pts = places
    elif origin and destination:
        pts = [origin, destination]
    else:
        return env(errors=["Provide places (>=2) or origin+destination"], label=label)

    path_ll: List[Tuple[float,float]] = []
    total_m = 0.0
    for i in range(len(pts)-1):
        a_lat, a_lon, a_disp = _resolve_place(pts[i])
        b_lat, b_lon, b_disp = _resolve_place(pts[i+1])
        seg = provider_route(a_lat, a_lon, b_lat, b_lon, mode=mode) or [(a_lat, a_lon), (b_lat, b_lon)]
        if path_ll and seg and path_ll[-1] == seg[0]:
            seg = seg[1:]
        path_ll += seg
        for j in range(1, len(seg)):
            total_m += haversine_m(seg[j-1][0], seg[j-1][1], seg[j][0], seg[j][1])
        log.info("backend.leg %s → %s: %d pts", a_disp, b_disp, len(seg))
    return env(label=label, params={"path_ll": path_ll, "distance_m": total_m, "mode": mode})

def path_smooth(points_ll: List[Tuple[float,float]],
                *,
                spacing_m: float = 120.0,
                geom_smooth_type: Literal["chaikin","moving_average","off"] = "chaikin",
                geom_smooth_passes: int = 2,
                ma_window: int = 5,
                speed_normalize: bool = True,
                heading_tau_s: Optional[float] = None,            # optional: pre-smooth heading
                max_turn_deg_per_s: Optional[float] = None,       # optional: clamp for heading smoothing
                label: Optional[str] = None) -> Dict[str,Any]:
    """
    TOOL: Geometry smoothing + (optional) speed normalization and (optional) heading pre-smoothing.
    Returns env(..., params={"path_ll": [...], "headings_deg": [...?], "distance_m": float})
    """
    if not points_ll or len(points_ll) < 2:
        return env(errors=["path_smooth: need at least 2 points"], label=label)

    total_m = 0.0
    for i in range(1, len(points_ll)):
        total_m += haversine_m(points_ll[i-1][0], points_ll[i-1][1], points_ll[i][0], points_ll[i][1])

    # Resample by spacing along original (piecewise linear)
    res: List[Tuple[float,float]] = [points_ll[0]]
    for i in range(1, len(points_ll)):
        lat1, lon1 = points_ll[i-1]
        lat2, lon2 = points_ll[i]
        seg = haversine_m(lat1, lon1, lat2, lon2)
        if seg <= spacing_m:
            res.append((lat2, lon2))
            continue
        n = int(seg // spacing_m)
        for k in range(1, n+1):
            f = (k * spacing_m) / seg
            res.append((lat1 + (lat2 - lat1) * f, lon1 + (lon2 - lon1) * f))
    if res[-1] != points_ll[-1]:
        res.append(points_ll[-1])

    # Geometry smoothing in local frame
    if geom_smooth_type != "off" and len(res) >= 3:
        latc = sum(p[0] for p in res)/len(res)
        lonc = sum(p[1] for p in res)/len(res)
        c = math.cos(math.radians(latc))
        def fwd(lat: float, lon: float) -> Tuple[float,float]:
            return (R*math.radians(lon-lonc)*c, R*math.radians(lat-latc))
        def inv(x: float, y: float) -> Tuple[float,float]:
            return (latc + math.degrees(y/R), lonc + math.degrees(x/(R*c)))
        xy = [fwd(lat, lon) for lat, lon in res]
        if geom_smooth_type == "chaikin":
            for _ in range(max(1, int(geom_smooth_passes))):
                out = [xy[0]]
                for i in range(len(xy)-1):
                    x0,y0 = xy[i]; x1,y1 = xy[i+1]
                    out.extend([(0.75*x0+0.25*x1, 0.75*y0+0.25*y1), (0.25*x0+0.75*x1, 0.25*y0+0.75*y1)])
                out.append(xy[-1]); xy = out
        elif geom_smooth_type == "moving_average":
            w = max(3, int(ma_window))
            out = []
            for i in range(len(xy)):
                a = max(0, i-w//2)
                b = min(len(xy), i+w//2+1)
                sx=sy=0.0
                for j in range(a,b):
                    sx+=xy[j][0]; sy+=xy[j][1]
                out.append((sx/(b-a), sy/(b-a)))
            xy = out
        res = [inv(x,y) for x,y in xy]

    # Normalize spacing to constant distance if requested (constant speed visually)
    if speed_normalize and len(res) >= 3:
        # Uniform spacing along smoothed path
        latc = sum(p[0] for p in res)/len(res)
        lonc = sum(p[1] for p in res)/len(res)
        c = math.cos(math.radians(latc))
        def fwd(lat: float, lon: float) -> Tuple[float,float]:
            return (R*math.radians(lon-lonc)*c, R*math.radians(lat-latc))
        def inv(x: float, y: float) -> Tuple[float,float]:
            return (latc + math.degrees(y/R), lonc + math.degrees(x/(R*c)))
        xy = [fwd(lat, lon) for lat, lon in res]
        seglen = [math.hypot(xy[i][0]-xy[i-1][0], xy[i][1]-xy[i-1][1]) for i in range(1, len(xy))]
        pts = [xy[0]]; target = spacing_m; i = 0; acc = 0.0
        while i < len(seglen):
            L = seglen[i]
            if acc + L >= target:
                r = (target - acc) / L
                x = xy[i][0] + r*(xy[i+1][0]-xy[i][0])
                y = xy[i][1] + r*(xy[i+1][1]-xy[i][1])
                pts.append((x,y)); acc = 0.0
                xy[i] = (x,y)
                seglen[i] = math.hypot(xy[i+1][0]-x, xy[i+1][1]-y)
            else:
                acc += L; i += 1
        if pts[-1] != xy[-1]: pts.append(xy[-1])
        res = [inv(x,y) for x,y in pts]

    out_params: Dict[str,Any] = {"path_ll": res, "distance_m": total_m}

    # Optional heading pre-smoothing (tau model over uniform dt=1 per step)
    if heading_tau_s is not None and len(res) >= 2:
        raw: List[float] = []
        for i in range(len(res)):
            lat1, lon1 = res[i-1] if i>0 else res[i]
            lat2, lon2 = res[i+1] if i<len(res)-1 else res[i]
            raw.append(_bearing_math(lat1, lon1, lat2, lon2, method="rhumb"))
        tau = max(1e-3, float(heading_tau_s))
        dt = 1.0  # pretend 1s steps; only smooth shape, not time-based rate
        a  = 1.0 - math.exp(-dt / tau)
        ms = float(max_turn_deg_per_s or 0.0)
        H = float(raw[0]); out_h: List[float] = []
        for h in raw:
            delta = (h - H + 540.0) % 360.0 - 180.0
            step  = delta * a
            if ms > 0.0:
                step = max(-ms*dt, min(ms*dt, step))
            if abs(delta) > 0.1 and abs(step) < 0.1:
                step = 0.1 if delta > 0 else -0.1
            H = (H + step + 360.0) % 360.0
            out_h.append(H)
        out_params["headings_deg"] = out_h

    return env(label=label, params=out_params)

def tour_from_path(points_ll: List[Tuple[float,float]],
                   *,
                   mode: Literal["lookat","camera"] = "lookat",
                   tilt: float = 70.0,
                   range_m: float = 700.0,
                   altitudeMode: Literal["clampToGround","relativeToGround","absolute"] = "relativeToGround",
                   alt: Optional[float] = None,
                   duration_total_s: Optional[float] = None,
                   speed_mps: float = 1.4,
                   heading_tau_s: float = 0.6,
                   heading_tao_s: Optional[float] = None,   # alias support
                   max_turn_deg_per_s: Optional[float] = 120.0,
                   flytomode: Literal["smooth","bounce"] = "smooth",
                   display: Union[bool, Dict[str,Any]] = False,
                   label: Optional[str] = None) -> Dict[str,Any]:
    """
    TOOL: build a camera tour from any path. Applies τ-based heading smoothing and
    time-proportional per-segment durations to achieve constant speed.
    Optionally renders the polyline via `display`.
    """
    if not points_ll or len(points_ll) < 2:
        return env(errors=["tour_from_path: need at least 2 points"], label=label)

    # Distances per segment
    seg_m: List[float] = []
    total_m = 0.0
    for i in range(1, len(points_ll)):
        m = haversine_m(points_ll[i-1][0], points_ll[i-1][1], points_ll[i][0], points_ll[i][1])
        seg_m.append(m); total_m += m
    if total_m <= 0.0:
        return env(errors=["tour_from_path: zero-length path"], label=label)

    # Duration model
    if duration_total_s is None:
        duration_total_s = max(10.0, total_m / max(0.2, float(speed_mps)))
    seg_dt = [ (m/total_m) * duration_total_s for m in seg_m ]

    # Raw bearings (central difference) and τ smoothing
    raw_h: List[float] = []
    for i in range(len(points_ll)):
        lat1, lon1 = points_ll[i-1] if i>0 else points_ll[i]
        lat2, lon2 = points_ll[i+1] if i<len(points_ll)-1 else points_ll[i]
        raw_h.append(_bearing_math(lat1, lon1, lat2, lon2, method="rhumb"))

    tau = heading_tau_s if heading_tau_s is not None else (heading_tao_s if heading_tao_s is not None else 0.6)
    tau = max(1e-3, float(tau))
    H = float(raw_h[0]); out_h: List[float] = [H]
    for i in range(1, len(points_ll)):
        dt = max(0.001, seg_dt[i-1])
        a  = 1.0 - math.exp(-dt / tau)
        target = raw_h[i]
        delta  = (target - H + 540.0) % 360.0 - 180.0
        step   = delta * a
        if max_turn_deg_per_s is not None and max_turn_deg_per_s > 0.0:
            step = max(-max_turn_deg_per_s*dt, min(max_turn_deg_per_s*dt, step))
        if abs(delta) > 0.1 and abs(step) < 0.1:
            step = 0.1 if delta > 0 else -0.1
        H = (H + step + 360.0) % 360.0
        out_h.append(H)

    # Build FlyTos
    parts: List[str] = []
    last_cam: Optional[Dict[str,Any]] = None
    for idx, (lat, lon) in enumerate(points_ll):
        dur = seg_dt[idx-1] if idx > 0 else max(0.05, seg_dt[0]*0.5)  # small initial
        hdg = out_h[idx]
        if mode == "lookat":
            step_env = flyto(mode="lookat", lat=lat, lon=lon, alt=(alt or 0.0), heading=hdg, tilt=tilt,
                             range_m=range_m, duration_s=dur, flytomode=flytomode, altitudeMode=altitudeMode,
                             last_cam=last_cam)
        else:
            cam = lookat_to_camera(lat, lon, (alt or 0.0), hdg, tilt, range_m)
            step_env = flyto(mode="camera", lat=cam["lat"], lon=cam["lon"], alt=cam["alt"], heading=hdg, tilt=tilt,
                             range_m=range_m, duration_s=dur, flytomode=flytomode, altitudeMode=altitudeMode,
                             last_cam=last_cam)
        parts += step_env["flytos"]
        last_cam = step_env["last_cam"]

    styles: List[str] = []
    geoms: List[str] = []

    if display:
        disp_opts = display if isinstance(display, dict) else {}
        pl = kml_polyline(disp_opts.get("name","Path"), points_ll,
                           altitudeMode="clampToGround",
                           style_color=disp_opts.get("style_color"),
                           style_width=disp_opts.get("style_width"))
        styles += pl["styles"]; geoms += pl["geometries"]

    return env(flytos=parts, geometries=geoms, styles=styles,
               params={"distance_m": total_m, "duration_total_s": duration_total_s},
               label=label, last_cam=last_cam)

# -------------------------------------------------------------------------------------
# Simplified legacy wrappers (kept for callers; now thin and readable)
# -------------------------------------------------------------------------------------
def camera_follow_path(points_ll: List[Tuple[float,float]],
                       spacing_m: float = 120.0,
                       duration_total_s: Optional[float] = None,
                       speed_mps: float = 1.4,
                       tilt: float = 70.0,
                       range_m: float = 700.0,
                       altitudeMode: Literal["clampToGround","relativeToGround","absolute"] = "relativeToGround",
                       alt: Optional[float] = None,
                       heading_tau_s: float = 0.6,
                       max_turn_deg_per_s: Optional[float] = 120.0,
                       path_smooth_type: Literal["chaikin","moving_average","off"] = "chaikin",
                       path_smooth_passes: int = 2,
                       path_ma_window: int = 5,
                       speed_normalize: bool = True,
                       display: Union[bool, Dict[str,Any]] = False,
                       label: Optional[str] = None) -> Dict[str, Any]:

    sm = path_smooth(points_ll,
                     spacing_m=spacing_m,
                     geom_smooth_type=path_smooth_type,
                     geom_smooth_passes=path_smooth_passes,
                     ma_window=path_ma_window,
                     speed_normalize=speed_normalize,
                     label=f"{label}-smooth" if label else None)
    if sm.get("errors"):
        return sm
    pts = sm["params"]["path_ll"]
    return tour_from_path(pts,
                          mode="lookat",
                          tilt=tilt,
                          range_m=range_m,
                          altitudeMode=altitudeMode,
                          alt=alt,
                          duration_total_s=duration_total_s,
                          speed_mps=speed_mps,
                          heading_tau_s=heading_tau_s,
                          max_turn_deg_per_s=max_turn_deg_per_s,
                          display=display,
                          label=label)

def directions_camera_path(places: Optional[List[Dict[str,Any]]] = None,
                           origin: Optional[Dict[str,Any]] = None,
                           destination: Optional[Dict[str,Any]] = None,
                           mode: Literal["walking","driving","cycling"] = "walking",
                           *,
                           spacing_m: float = 120.0,
                           duration_total_s: Optional[float] = None,
                           speed_mps: float = 1.4,
                           tilt: float = 70.0,
                           range_m: float = 700.0,
                           altitudeMode: Literal["clampToGround","relativeToGround","absolute"] = "relativeToGround",
                           heading_tau_s: float = 0.6,
                           max_turn_deg_per_s: Optional[float] = 120.0,
                           alt: Optional[float] = None,
                           display: Union[bool, Dict[str,Any]] = False,
                           label: Optional[str] = None,
                           last_cam: Optional[Dict[str,Any]] = None) -> Dict[str,Any]:

    raw = route_path_raw(places=places, origin=origin, destination=destination, mode=mode,
                         label=f"{label}-route" if label else None)
    if raw.get("errors"):
        return raw
    pts = raw["params"]["path_ll"]
    return camera_follow_path(pts,
                              spacing_m=spacing_m,
                              duration_total_s=duration_total_s,
                              speed_mps=speed_mps,
                              tilt=tilt, range_m=range_m,
                              altitudeMode=altitudeMode, alt=alt,
                              heading_tau_s=heading_tau_s,
                              max_turn_deg_per_s=max_turn_deg_per_s,
                              display=display,
                              label=label)

# -------------------------------------------------------------------------------------
# Export table (tools)
# -------------------------------------------------------------------------------------
FUNCTION_TABLE: Dict[str, Any] = {
    "kml_point": kml_point,
    "kml_polyline": kml_polyline,
    "kml_circle": kml_circle,
    "flyto": flyto,
    "orbit": orbit,
    "tour_wait": tour_wait,

    "bearing_between": bearing_between,
    "heading_turn": heading_turn,

    "route_path_raw": route_path_raw,
    "path_smooth": path_smooth,
    "tour_from_path": tour_from_path,

    "camera_follow_path": camera_follow_path,
    "directions_camera_path": directions_camera_path,

}

__all__ = [
    "get_logger", "configure_logging",
    "env", "coords_from",
    "_bearing_math", "bearing_between",
    "kml_point", "kml_polyline", "kml_circle",
    "flyto", "orbit", "tour_wait",
    "route_path_raw", "path_smooth", "tour_from_path",
    "camera_follow_path", "directions_camera_path",
    "heading_turn",
    "FUNCTION_TABLE",
]

