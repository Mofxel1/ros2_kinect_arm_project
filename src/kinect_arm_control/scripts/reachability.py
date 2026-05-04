import math
from dataclasses import dataclass


@dataclass
class ReachabilityResult:
    reachable: bool
    reason: str
    joint0: float = 0.0
    joint1: float = 0.0
    joint2: float = 0.0
    distance: float = 0.0


class ArmReachability:
    """
    Task manager için kaba ulaşılabilirlik kontrolü.

    Burada kesin IK kararı vermiyoruz.
    Kesin karar dynamic_brain_node_queue + MoveIt tarafında verilecek.

    Amaç:
    - Çok saçma hedefleri elemek
    - NaN / sonsuz değerleri elemek
    - Robotun fiziksel olarak çok uzağındaki hedefleri elemek
    """

    def __init__(self):
        # Robot link uzunlukları
        self.L1 = 0.183
        self.L2 = 0.220
        self.L3 = 0.200

        # Toplam erişim
        self.max_reach = self.L2 + self.L3 + 0.05
        self.min_reach = 0.01

        # Çok geniş güvenlik sınırları.
        # Bunlar kutu workspace değil, sadece saçma değerleri elemek için.
        self.absolute_x_limit = 0.60
        self.absolute_y_limit = 0.60
        self.absolute_z_min = -0.05
        self.absolute_z_max = 0.60

    def is_finite_number(self, value):
        return value is not None and math.isfinite(value)

    def is_reachable(self, target_x, target_y, target_z):
        # 1. Sayısal kontrol
        if not self.is_finite_number(target_x):
            return ReachabilityResult(False, "invalid_x")

        if not self.is_finite_number(target_y):
            return ReachabilityResult(False, "invalid_y")

        if not self.is_finite_number(target_z):
            return ReachabilityResult(False, "invalid_z")

        # 2. Aşırı saçma koordinatları ele
        if abs(target_x) > self.absolute_x_limit:
            return ReachabilityResult(False, "x_absolute_limit")

        if abs(target_y) > self.absolute_y_limit:
            return ReachabilityResult(False, "y_absolute_limit")

        if target_z < self.absolute_z_min or target_z > self.absolute_z_max:
            return ReachabilityResult(False, "z_absolute_limit")

        # 3. Brain node ile aynı eksen dönüşümüne göre kaba mesafe
        x = target_y
        y = target_x
        z = target_z

        r = math.sqrt(x * x + y * y)
        z_offset = z - self.L1
        distance = math.sqrt(r * r + z_offset * z_offset)

        if distance > self.max_reach:
            return ReachabilityResult(
                False,
                "too_far",
                distance=distance
            )

        if distance < self.min_reach:
            return ReachabilityResult(
                False,
                "too_close",
                distance=distance
            )

        # 4. Sadece bilgi amaçlı yaklaşık joint0 hesapla
        joint0 = math.atan2(y, x)

        return ReachabilityResult(
            True,
            "rough_ok",
            joint0=joint0,
            distance=distance
        )