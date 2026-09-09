"""Shared MuJoCo model calculations for ARX-X5-2023 wrench estimation.

This module has no hardware or terminal dependencies.  Online force observation
and offline effort calibration should use this implementation so their joint
indices, inverse-dynamics exclusions, Jacobian, and site frame stay identical.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

import mujoco
import numpy as np
import numpy.typing as npt

from .wrench_sim import WrenchSim, WrenchSimConfig


DEFAULT_ARX_FORCE_SITE = "ee_site"
ARX_ARM_JOINT_NAMES = tuple(f"joint{index}" for index in range(1, 7))
ARX_FINGER_JOINT_NAMES = ("left_finger_joint", "right_finger_joint")
_INVERSE_DISABLE_BITS = (
    "mjDSBL_EQUALITY",
    "mjDSBL_CONTACT",
    "mjDSBL_LIMIT",
    "mjDSBL_ACTUATION",
    "mjDSBL_FRICTIONLOSS",
)


def default_arx_wrench_xml_path() -> Path:
    """Return the packaged fixed X5-2023 model used by the keyboard tool."""
    return (
        Path(__file__).resolve().parents[1]
        / "models"
        / "arx_x5_2023"
        / "arx_x5_2023_fixed.xml"
    )


def _finite_vector(
    values: Sequence[float] | npt.ArrayLike,
    *,
    size: int,
    label: str,
) -> npt.NDArray[np.float64]:
    result = np.asarray(values, dtype=np.float64).reshape(-1)
    if result.shape != (size,):
        raise ValueError(f"{label} shape {result.shape} != ({size},).")
    if not np.all(np.isfinite(result)):
        raise ValueError(f"{label} contains NaN or Inf.")
    return result


@dataclass(frozen=True)
class ARXWrenchModelState:
    """Static or dynamic model quantities for the six arm joints."""

    jacobian_position: npt.NDArray[np.float64]
    jacobian_rotation: npt.NDArray[np.float64]
    site_rotation_base: npt.NDArray[np.float64]
    model_torque_nm: npt.NDArray[np.float64]


class ARXWrenchModel:
    """Compute X5-2023 inverse dynamics and the configured site Jacobian."""

    def __init__(
        self,
        xml_path: Path | str | None = None,
        *,
        site_name: str = DEFAULT_ARX_FORCE_SITE,
    ) -> None:
        self.xml_path = Path(
            default_arx_wrench_xml_path() if xml_path is None else xml_path
        ).expanduser().resolve()
        self.site_name = str(site_name)
        self.sim = WrenchSim(
            WrenchSimConfig(
                xml_path=str(self.xml_path),
                site_names=(self.site_name,),
                fixed_base=True,
            )
        )
        self.qpos_indices: list[int] = []
        self.dof_indices: list[int] = []
        for name in ARX_ARM_JOINT_NAMES:
            joint_id = mujoco.mj_name2id(
                self.sim.model, mujoco.mjtObj.mjOBJ_JOINT, name
            )
            if joint_id < 0:
                raise ValueError(f"ARX wrench model is missing joint {name!r}.")
            self.qpos_indices.append(int(self.sim.model.jnt_qposadr[joint_id]))
            self.dof_indices.append(int(self.sim.model.jnt_dofadr[joint_id]))

        self.finger_qpos_indices: list[int] = []
        self.finger_dof_indices: list[int] = []
        self.finger_rest_qpos: list[float] = []
        key_qpos = (
            np.asarray(self.sim.model.key_qpos[0], dtype=np.float64)
            if int(self.sim.model.nkey) > 0
            else None
        )
        for name in ARX_FINGER_JOINT_NAMES:
            joint_id = mujoco.mj_name2id(
                self.sim.model, mujoco.mjtObj.mjOBJ_JOINT, name
            )
            if joint_id < 0:
                continue
            qpos_adr = int(self.sim.model.jnt_qposadr[joint_id])
            self.finger_qpos_indices.append(qpos_adr)
            self.finger_dof_indices.append(
                int(self.sim.model.jnt_dofadr[joint_id])
            )
            if key_qpos is not None:
                self.finger_rest_qpos.append(float(key_qpos[qpos_adr]))
            else:
                self.finger_rest_qpos.append(
                    0.044 if name.startswith("left") else -0.044
                )

        self.inverse_disableflags = 0
        for bit_name in _INVERSE_DISABLE_BITS:
            self.inverse_disableflags |= int(
                getattr(mujoco.mjtDisableBit, bit_name)
            )

    def evaluate(
        self,
        position_rad: Sequence[float] | npt.ArrayLike,
        velocity_rad_s: Sequence[float] | npt.ArrayLike | None = None,
        acceleration_rad_s2: Sequence[float] | npt.ArrayLike | None = None,
    ) -> ARXWrenchModelState:
        """Evaluate model torque and site Jacobian at one six-joint state."""
        position = _finite_vector(
            position_rad, size=6, label="ARX joint position"
        )
        velocity = _finite_vector(
            np.zeros(6) if velocity_rad_s is None else velocity_rad_s,
            size=6,
            label="ARX joint velocity",
        )
        acceleration = _finite_vector(
            np.zeros(6) if acceleration_rad_s2 is None else acceleration_rad_s2,
            size=6,
            label="ARX joint acceleration",
        )

        model = self.sim.model
        data = self.sim.data
        previous_disable = int(model.opt.disableflags)
        model.opt.disableflags = previous_disable | self.inverse_disableflags
        try:
            data.qvel[:] = 0.0
            data.qacc[:] = 0.0
            data.qfrc_applied[:] = 0.0
            data.xfrc_applied[:] = 0.0
            if data.ctrl.size:
                data.ctrl[:] = 0.0
            data.qpos[self.qpos_indices] = position
            data.qvel[self.dof_indices] = velocity
            data.qacc[self.dof_indices] = acceleration
            for qpos_adr, rest in zip(
                self.finger_qpos_indices, self.finger_rest_qpos
            ):
                data.qpos[qpos_adr] = rest
            for dof_adr in self.finger_dof_indices:
                data.qvel[dof_adr] = 0.0
                data.qacc[dof_adr] = 0.0
            # mj_forward would overwrite the prescribed qacc.  This is an
            # inverse-dynamics calculation, not an actuator/constraint solve.
            mujoco.mj_inverse(model, data)
            jacp, jacr = self.sim.site_jacobian(self.site_name)
            site_id = self.sim.site_ids[self.site_name]
            site_rotation = np.asarray(
                data.site_xmat[site_id], dtype=np.float64
            ).reshape(3, 3)
            model_torque = np.asarray(
                data.qfrc_inverse[self.dof_indices], dtype=np.float64
            ).copy()
        finally:
            model.opt.disableflags = previous_disable

        indices = np.asarray(self.dof_indices, dtype=np.int32)
        return ARXWrenchModelState(
            jacobian_position=np.asarray(jacp[:, indices], dtype=np.float64),
            jacobian_rotation=np.asarray(jacr[:, indices], dtype=np.float64),
            site_rotation_base=site_rotation.copy(),
            model_torque_nm=model_torque,
        )

    def close(self) -> None:
        self.sim.close()

    def __enter__(self) -> "ARXWrenchModel":
        return self

    def __exit__(self, *_args: object) -> None:
        self.close()
