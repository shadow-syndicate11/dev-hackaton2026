"""
main.py  — Датасет ↔ Симулятор бүрэн холбогдсон хувилбар
══════════════════════════════════════════════════════════════
Өөрчлөлтүүд:
  1. DatasetBridge импортлосон
  2. lifespan дотор bridge.build_simulator_config() дуудаж
     simulator.apply_dataset_config()-р ачаалдаг болсон
  3. /api/dataset/load endpoint нэмсэн (runtime дахин ачаалах)
"""
from __future__ import annotations

from contextlib import asynccontextmanager

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

from backend.routers import analytics, signals, simulation
from backend.routers.dataset import router as dataset_router
from backend.services.dataset_service import init_dataset
from backend.services.simulator import TrafficSimulator
from backend.services.dataset_bridge import DatasetBridge


# ── Датасетийн файл замыг энд өөрчил ─────────────────────
from pathlib import Path

BASE_DIR = Path(__file__).resolve().parent
DATASET_PATH = BASE_DIR / "data" / "UB_Traffic_Dataset1.csv"

# Анхны ачааллын тохиргоо
DEFAULT_INTERSECTION_ID = 1     # 1–35, аль уулзварыг эхлэх
DEFAULT_USE_PEAK        = True  # True = оргил цагийн датa
DEFAULT_USE_HEAVIEST    = False # True = хамгийн хүнд ачааллын датa
DEFAULT_MODE            = "fixed"  # "fixed" | "ai"


@asynccontextmanager
async def lifespan(app: FastAPI):
    # ── 1. Датасет ачаалах ─────────────────────────────────
    try:
        loader = init_dataset(DATASET_PATH)
        app.state.dataset = loader
        print(f"[dataset] OK — {len(loader.df):,} мөр, "
              f"{loader.df['intersection_id'].nunique()} уулзвар ачааллаа")
    except FileNotFoundError:
        print(f"[dataset] WARN: файл олдсонгүй → {DATASET_PATH}")
        app.state.dataset = None
    except Exception as e:
        print(f"[dataset] ERROR: {e}")
        app.state.dataset = None

    # ── 2. Симулятор үүсгэх ────────────────────────────────
    simulator = TrafficSimulator()
    app.state.simulator = simulator

    # ── 3. Датасет → Симулятор холболт ────────────────────
    if app.state.dataset is not None:
        try:
            bridge = DatasetBridge(app.state.dataset)
            app.state.bridge = bridge

            config = bridge.build_simulator_config(
                intersection_id=DEFAULT_INTERSECTION_ID,
                use_peak=DEFAULT_USE_PEAK,
                use_heaviest=DEFAULT_USE_HEAVIEST,
                mode=DEFAULT_MODE,
            )
            await simulator.apply_dataset_config(config)
            print(
                f"[bridge] Холбогдлоо: "
                f"{config.get('intersection_name','?')} | "
                f"дараалал={sum(config.get('queues',{}).values())} | "
                f"машин={len(config.get('vehicles',[]))} | "
                f"оргил={config.get('peak_hour')}"
            )
        except Exception as e:
            print(f"[bridge] ERROR: {e}")
            app.state.bridge = None
    else:
        app.state.bridge = None
        print("[bridge] Датасетгүй — симулятор default горимд ажиллана")

    # ── 4. Симуляцийн цикл эхлүүлэх ───────────────────────
    await simulator.start_loop()

    try:
        yield
    finally:
        await simulator.stop_loop()


# ══════════════════════════════════════════════════════════
# FastAPI APP
# ══════════════════════════════════════════════════════════

app = FastAPI(
    title="AI Traffic Signal Simulator API",
    version="3.0.0",
    lifespan=lifespan,
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://localhost:3000",  "http://127.0.0.1:3000",
        "http://localhost:3001",  "http://127.0.0.1:3001",
        "http://localhost:5173",  "http://127.0.0.1:5173",
    ],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


# ══════════════════════════════════════════════════════════
# ENDPOINT-УУД
# ══════════════════════════════════════════════════════════

@app.get("/health")
async def health() -> dict[str, str]:
    return {"status": "ok"}


class DatasetLoadRequest(BaseModel):
    intersection_id: int  = DEFAULT_INTERSECTION_ID
    use_peak:        bool = True
    use_heaviest:    bool = False
    mode:            str  = "fixed"


@app.post("/api/dataset/apply")
async def apply_dataset(req: DatasetLoadRequest):
    """
    Runtime дахин: аль нэг уулзварын датаг симуляторт ачаална.

    Жишээ:
        POST /api/dataset/apply
        {"intersection_id": 3, "use_peak": true, "mode": "ai"}
    """
    from fastapi import Request
    from starlette.requests import Request as StarletteRequest

    # app.state-аас авна
    import starlette.requests
    bridge:    DatasetBridge    = app.state.bridge
    simulator: TrafficSimulator = app.state.simulator

    if bridge is None:
        raise HTTPException(
            status_code=503,
            detail="Датасет ачаалагдаагүй байна. DATASET_PATH шалгана уу.",
        )

    try:
        config = bridge.build_simulator_config(
            intersection_id=req.intersection_id,
            use_peak=req.use_peak,
            use_heaviest=req.use_heaviest,
            mode=req.mode,
        )
        await simulator.apply_dataset_config(config)
        return {
            "ok":               True,
            "intersection_id":  req.intersection_id,
            "intersection_name": config.get("intersection_name"),
            "mode":             req.mode,
            "peak_hour":        config.get("peak_hour"),
            "queues":           config.get("queues"),
            "vehicles_spawned": len(config.get("vehicles", [])),
            "weather":          config.get("weather"),
            "congestion_tier":  config.get("congestion_tier"),
            "kpi_baseline":     config.get("kpi_baseline"),
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/api/dataset/intersections")
async def list_intersections():
    """Бүх уулзварын жагсаалт буцаана."""
    loader = app.state.dataset
    if loader is None:
        raise HTTPException(status_code=503, detail="Датасет ачаалагдаагүй.")
    return {
        "intersections": [
            {"id": iid, "name": name}
            for iid, name in loader.get_intersection_names().items()
        ]
    }


@app.get("/api/dataset/summary")
async def dataset_summary():
    """Датасетийн хураангуй статистик."""
    loader = app.state.dataset
    if loader is None:
        raise HTTPException(status_code=503, detail="Датасет ачаалагдаагүй.")
    return loader.summary_statistics()


# ── Байгаа router-уудыг холбох ────────────────────────────
app.include_router(signals.router)
app.include_router(simulation.router)
app.include_router(simulation.ws_router)
app.include_router(analytics.router)
app.include_router(dataset_router)