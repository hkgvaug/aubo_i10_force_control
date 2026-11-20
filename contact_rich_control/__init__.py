"""
Contact-Rich Robotic Manipulation in Construction via Zero-Shot Learning:
A Diffusion Policy-Guided Adaptive Control Framework.
"""

from .l1_adaptive_controller import L1AdaptiveController, L1ControllerConfig
from .cartesian_admittance_controller import (
    CartesianAdmittanceController,
    AdmittanceConfig,
)

__all__ = [
    "L1AdaptiveController",
    "L1ControllerConfig",
    "CartesianAdmittanceController",
    "AdmittanceConfig",
]

# The diffusion-policy / dataset / statistics modules are not part of this
# deployment (only the kinematic + admittance + L1 stack is shipped), so they are
# imported only when present.  Importing them unconditionally made the whole
# package unusable: `import contact_rich_control` raised ModuleNotFoundError.
for _optional in ("diffusion_policy", "dataset_generator", "statistical_evaluator"):
    try:
        _module = __import__("." + _optional, globals(), locals(), ["*"], 1)
    except ImportError:
        continue
    __all__.extend(_module.__all__ if hasattr(_module, "__all__") else [n for n in dir(_module) if not n.startswith("_")])
