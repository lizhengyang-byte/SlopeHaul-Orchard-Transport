"""预置评测场景：覆盖松软上坡、横坡重载、坡道驻车、起伏下行与超限拒行五类工况。"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

from ..config import ScenarioParams
from ..sim.terrain import (
    ALL_PROFILES,
    TerrainProfile,
    cross_slope_profile,
    hill_hold_profile,
    rough_downhill_profile,
    soft_slope_profile,
    steep_wet_profile,
)


@dataclass
class ScenarioSpec:
    key: str
    name: str
    terrain_factory: Callable[[], TerrainProfile]
    params: ScenarioParams
    description: str
    focus: str
    expect_block: bool = False

    def outcome(self, summary: dict) -> str:
        """把“到没到终点”翻译成工程结论：到达 / 安全拦停 / 失控失败。"""
        if summary.get("tipped"):
            return "tipover"
        if self.expect_block:
            return "safe_stop" if summary.get("final_s", 0.0) < self.params.length * 0.6 and summary.get("back_slip_distance", 0.0) < 2.0 else "failed"
        return "reached" if summary.get("reached") else "failed"


def _specs() -> dict[str, ScenarioSpec]:
    return {
        "S1_soft_climb": ScenarioSpec(
            key="S1_soft_climb",
            name="松软土重载爬坡",
            terrain_factory=soft_slope_profile,
            params=ScenarioParams(name="S1_soft_climb", length=18.0, payload=180.0, duration=40.0),
            description="8°→20° 松软土长坡，载重 180 kg，坡中段松软带 μ≈0.64",
            focus="打滑抑制与爬坡通过性",
        ),
        "S2_cross_slope": ScenarioSpec(
            key="S2_cross_slope",
            name="横坡弯道重载运输",
            terrain_factory=cross_slope_profile,
            params=ScenarioParams(name="S2_cross_slope", length=16.0, payload=200.0, duration=30.0, initial_speed=0.6),
            description="14° 横坡 + 半径约 6 m 弯道，满载 200 kg，考核侧倾裕度",
            focus="侧倾稳定与弯道限速",
        ),
        "S3_hill_hold": ScenarioSpec(
            key="S3_hill_hold",
            name="坡道停车与起步",
            terrain_factory=hill_hold_profile,
            params=ScenarioParams(
                name="S3_hill_hold", length=12.0, payload=180.0, duration=25.0,
                initial_speed=0.5, stop_at=5.0, restart_after=8.0,
            ),
            description="18° 松软坡中途驻车 8 s 后重新起步，考核驻车能力与溜坡量",
            focus="驻车力评估与起步防溜",
        ),
        "S4_rough_downhill": ScenarioSpec(
            key="S4_rough_downhill",
            name="起伏路面长下坡",
            terrain_factory=rough_downhill_profile,
            params=ScenarioParams(name="S4_rough_downhill", length=20.0, payload=160.0, duration=42.0, initial_speed=0.8),
            description="-16° 长下坡，粗糙度 5.5~7.5 cm，考核下行限速与制动距离",
            focus="下行速度自适应与姿态平顺性",
        ),
        "S5_steep_wet": ScenarioSpec(
            key="S5_steep_wet",
            name="湿滑超限陡坡",
            terrain_factory=steep_wet_profile,
            params=ScenarioParams(name="S5_steep_wet", length=12.0, payload=120.0, duration=25.0),
            description="26° 湿滑黏土（μ≈0.50），物理上不可通过，考核系统能否识别并拒行",
            focus="能力边界识别与安全拒行",
            expect_block=True,
        ),
    }


SCENARIOS = _specs()
