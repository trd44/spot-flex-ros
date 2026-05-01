"""ROS-neutral policy planning for Spot FLEX manipulation.

This module intentionally does not import the old ``flex_spot`` example
package. It keeps the useful policy ideas in a small ROS-friendly layer:
named tasks are expanded into trigger calls and bounded body velocity segments.
If trained actor weights are supplied at runtime, the same interface can use
them to generate those body velocity segments.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import cos, sin
from pathlib import Path
from typing import Iterable, Sequence

import numpy as np


@dataclass(frozen=True)
class BodyCommand:
    """A bounded body velocity command segment."""

    status: str
    duration_sec: float
    linear_x: float = 0.0
    linear_y: float = 0.0
    angular_z: float = 0.0


@dataclass(frozen=True)
class TriggerCommand:
    """A call to a Trigger service."""

    service_name: str
    status: str = ''


PolicyCommand = BodyCommand | TriggerCommand


@dataclass(frozen=True)
class PolicyPlan:
    """A fully expanded policy ready for a ROS node to execute."""

    policy_name: str
    commands: tuple[PolicyCommand, ...]

    @property
    def duration_sec(self) -> float:
        return sum(
            command.duration_sec
            for command in self.commands
            if isinstance(command, BodyCommand)
        )


@dataclass(frozen=True)
class PolicyExecutorConfig:
    """Runtime-tunable policy parameters."""

    push_speed_mps: float = 0.12
    push_lateral_speed_mps: float = 0.0
    push_yaw_rate_radps: float = 0.0
    push_duration_sec: float = 2.0

    revolute_linear_speed_mps: float = 0.04
    revolute_lateral_speed_mps: float = 0.08
    revolute_yaw_rate_radps: float = 0.18
    revolute_duration_sec: float = 4.0

    prismatic_speed_mps: float = 0.08
    prismatic_duration_sec: float = 3.0
    prismatic_axis_xy: tuple[float, float] = (1.0, 0.0)

    policy_step_duration_sec: float = 0.5
    policy_max_steps: int = 12
    action_scale: float = 0.05
    max_linear_speed_mps: float = 0.18
    max_angular_speed_radps: float = 0.35
    invert_action_x: bool = True

    use_learned_policies: bool = False
    revolute_model_dir: str = ''
    revolute_model_name: str = 'final'
    prismatic_model_dir: str = ''
    prismatic_model_name: str = 'final'
    push_model_dir: str = ''
    push_model_name: str = 'best_model'

    position_for_grasp_triggers: tuple[str, ...] = ('arm_unstow',)
    grasp_triggers: tuple[str, ...] = ('close_gripper', 'arm_carry')
    place_triggers: tuple[str, ...] = ('open_gripper',)
    revolute_pre_triggers: tuple[str, ...] = ('arm_unstow',)
    revolute_post_triggers: tuple[str, ...] = ()
    prismatic_pre_triggers: tuple[str, ...] = ('arm_unstow',)
    prismatic_post_triggers: tuple[str, ...] = ()


class PolicyExecutor:
    """Builds executable manipulation plans from policy names."""

    _PUSH_ALIASES = {'push_box', 'box_push', 'push'}
    _REVOLUTE_ALIASES = {'open_cabinet', 'open_door', 'open_revolute', 'open_revolute_door'}
    _PRISMATIC_ALIASES = {'operate_prismatic', 'open_prismatic', 'pull_prismatic', 'slide_prismatic'}

    def __init__(self, config: PolicyExecutorConfig | None = None):
        self.config = config or PolicyExecutorConfig()
        self._actor_cache: dict[tuple[str, str, str], _ActorPolicy] = {}

    def build_plan(self, policy_name: str) -> PolicyPlan:
        """Return an executable plan for ``policy_name``.

        Raises:
            ValueError: if the policy name is unknown.
        """
        policy = self._normalize_name(policy_name)

        if policy in self._PUSH_ALIASES:
            return self._push_box(policy_name)
        if policy in self._REVOLUTE_ALIASES:
            return self._revolute_policy(policy_name)
        if policy in self._PRISMATIC_ALIASES:
            return self._prismatic_policy(policy_name)
        if policy == 'position_for_grasp':
            return self._trigger_plan(policy_name, self.config.position_for_grasp_triggers)
        if policy == 'grasp':
            return self._trigger_plan(policy_name, self.config.grasp_triggers)
        if policy == 'place':
            return self._trigger_plan(policy_name, self.config.place_triggers)

        raise ValueError(f'unknown policy: {policy_name}')

    def _push_box(self, policy_name: str) -> PolicyPlan:
        if self.config.use_learned_policies and self.config.push_model_dir:
            actor = self._load_actor(
                kind='path',
                model_dir=self.config.push_model_dir,
                model_name=self.config.push_model_name,
            )
            if actor is not None:
                commands = self._learned_path_commands(actor)
                return PolicyPlan(policy_name, commands)

        command = BodyCommand(
            status='pushing box',
            duration_sec=self.config.push_duration_sec,
            linear_x=self.config.push_speed_mps,
            linear_y=self.config.push_lateral_speed_mps,
            angular_z=self.config.push_yaw_rate_radps,
        )
        return PolicyPlan(policy_name, (command,))

    def _revolute_policy(self, policy_name: str) -> PolicyPlan:
        commands: list[PolicyCommand] = self._triggers(self.config.revolute_pre_triggers)

        if self.config.use_learned_policies and self.config.revolute_model_dir:
            actor = self._load_actor(
                kind='joint',
                model_dir=self.config.revolute_model_dir,
                model_name=self.config.revolute_model_name,
            )
            commands.extend(self._learned_joint_commands(actor, 'revolute') if actor else [])

        if not any(isinstance(command, BodyCommand) for command in commands):
            commands.append(BodyCommand(
                status='opening revolute joint',
                duration_sec=self.config.revolute_duration_sec,
                linear_x=self.config.revolute_linear_speed_mps,
                linear_y=self.config.revolute_lateral_speed_mps,
                angular_z=self.config.revolute_yaw_rate_radps,
            ))

        commands.extend(self._triggers(self.config.revolute_post_triggers))
        return PolicyPlan(policy_name, tuple(commands))

    def _prismatic_policy(self, policy_name: str) -> PolicyPlan:
        commands: list[PolicyCommand] = self._triggers(self.config.prismatic_pre_triggers)

        if self.config.use_learned_policies and self.config.prismatic_model_dir:
            actor = self._load_actor(
                kind='joint',
                model_dir=self.config.prismatic_model_dir,
                model_name=self.config.prismatic_model_name,
            )
            commands.extend(self._learned_joint_commands(actor, 'prismatic') if actor else [])

        if not any(isinstance(command, BodyCommand) for command in commands):
            axis_x, axis_y = _normalized_xy(self.config.prismatic_axis_xy)
            commands.append(BodyCommand(
                status='operating prismatic joint',
                duration_sec=self.config.prismatic_duration_sec,
                linear_x=axis_x * self.config.prismatic_speed_mps,
                linear_y=axis_y * self.config.prismatic_speed_mps,
            ))

        commands.extend(self._triggers(self.config.prismatic_post_triggers))
        return PolicyPlan(policy_name, tuple(commands))

    def _trigger_plan(self, policy_name: str, services: Sequence[str]) -> PolicyPlan:
        return PolicyPlan(policy_name, tuple(self._triggers(services)))

    def _triggers(self, services: Iterable[str]) -> list[TriggerCommand]:
        return [
            TriggerCommand(service_name=service, status=service)
            for service in services
            if service
        ]

    def _learned_path_commands(self, actor: '_ActorPolicy') -> tuple[BodyCommand, ...]:
        commands = []
        yaw = 0.0
        steps = max(1, self.config.policy_max_steps)
        for step in range(steps):
            progress = step / max(steps - 1, 1)
            if actor.state_dim == 8:
                state = np.array([
                    0.0, 0.0, 0.0, progress, 0.0, 0.0, cos(yaw), sin(yaw)
                ], dtype=np.float32)
            else:
                state = np.zeros(actor.state_dim, dtype=np.float32)
                if actor.state_dim > 3:
                    state[3] = progress
            action = actor.select_action(state)
            commands.append(self._action_to_body_command(
                action,
                status='learned box push',
                include_angular=True,
            ))
            yaw += commands[-1].angular_z * self.config.policy_step_duration_sec
        return tuple(commands)

    def _learned_joint_commands(self, actor: '_ActorPolicy', joint_type: str) -> tuple[BodyCommand, ...]:
        commands = []
        axis = np.array([1.0, 0.0, 0.0], dtype=np.float32)
        if joint_type == 'prismatic':
            axis_x, axis_y = _normalized_xy(self.config.prismatic_axis_xy)
            axis = np.array([axis_x, axis_y, 0.0], dtype=np.float32)

        displacement = np.zeros(3, dtype=np.float32)
        steps = max(1, self.config.policy_max_steps)
        for step in range(steps):
            if actor.state_dim == 8:
                progress = step / max(steps - 1, 1)
                state = np.array([
                    0.0, 0.0, 0.0, progress, 0.0, 0.0, 1.0, 0.0
                ], dtype=np.float32)
            else:
                state = np.concatenate([axis, displacement]).astype(np.float32)
            action = actor.select_action(state)
            if joint_type == 'prismatic':
                action = np.dot(action[:3], axis) * axis
            command = self._action_to_body_command(
                action,
                status=f'learned {joint_type} policy',
                include_angular=(joint_type == 'revolute'),
            )
            commands.append(command)
            displacement += np.array([
                command.linear_x,
                command.linear_y,
                0.0,
            ], dtype=np.float32) * self.config.policy_step_duration_sec
        return tuple(commands)

    def _action_to_body_command(
        self,
        action: np.ndarray,
        status: str,
        include_angular: bool,
    ) -> BodyCommand:
        action = np.asarray(action, dtype=np.float32).reshape(-1)
        if action.size < 2:
            raise ValueError(f'policy action must contain at least two values, got {action}')

        action_x = -action[0] if self.config.invert_action_x else action[0]
        linear_x = _clamp(
            action_x * self.config.action_scale / self.config.policy_step_duration_sec,
            self.config.max_linear_speed_mps,
        )
        linear_y = _clamp(
            action[1] * self.config.action_scale / self.config.policy_step_duration_sec,
            self.config.max_linear_speed_mps,
        )
        angular_z = 0.0
        if include_angular and action.size > 2:
            angular_z = _clamp(
                action[2] * self.config.action_scale / self.config.policy_step_duration_sec,
                self.config.max_angular_speed_radps,
            )

        return BodyCommand(
            status=status,
            duration_sec=self.config.policy_step_duration_sec,
            linear_x=linear_x,
            linear_y=linear_y,
            angular_z=angular_z,
        )

    def _load_actor(self, kind: str, model_dir: str, model_name: str) -> '_ActorPolicy | None':
        key = (kind, model_dir, model_name)
        if key not in self._actor_cache:
            actor_path = Path(model_dir) / f'{model_name}_actor.pth'
            if not actor_path.exists():
                return None
            self._actor_cache[key] = _ActorPolicy.load(kind, actor_path)
        return self._actor_cache[key]

    @staticmethod
    def _normalize_name(policy_name: str) -> str:
        return policy_name.strip().lower().replace('-', '_').replace(' ', '_')


class _ActorPolicy:
    """Small inference-only wrapper for the actor architectures used by FLEX."""

    def __init__(self, actor, torch_module, state_dim: int, action_dim: int):
        self._actor = actor
        self._torch = torch_module
        self.state_dim = state_dim
        self.action_dim = action_dim

    @classmethod
    def load(cls, kind: str, actor_path: Path) -> '_ActorPolicy':
        try:
            import torch
            import torch.nn as nn
            import torch.nn.functional as functional
        except ImportError as exc:
            raise RuntimeError('PyTorch is required to load learned policy weights') from exc

        state = torch.load(str(actor_path), map_location='cpu')
        state_dim = int(state['l1.weight'].shape[1])
        if kind == 'path':
            action_dim = 3 if 'tau_head.weight' in state else 2
            actor = _make_path_actor(torch, nn, functional, state_dim, action_dim)
        elif kind == 'joint':
            if 'dir_head.weight' in state:
                action_dim = 3 if 'tau_head.weight' in state else 2
                actor = _make_path_actor(torch, nn, functional, state_dim, action_dim)
            else:
                action_dim = int(state['l3.weight'].shape[0])
                actor = _make_joint_actor(torch, nn, functional, state_dim, action_dim)
        else:
            raise ValueError(f'unknown actor kind: {kind}')

        actor.load_state_dict(state)
        actor.eval()
        return cls(actor, torch, state_dim, action_dim)

    def select_action(self, state: np.ndarray) -> np.ndarray:
        with self._torch.no_grad():
            tensor = self._torch.as_tensor(state.reshape(1, -1), dtype=self._torch.float32)
            return self._actor(tensor).cpu().numpy().reshape(-1)


def _make_joint_actor(torch, nn, functional, state_dim: int, action_dim: int):
    class JointActor(nn.Module):
        def __init__(self):
            super().__init__()
            self.l1 = nn.Linear(state_dim, 400)
            self.l2 = nn.Linear(400, 300)
            self.l3 = nn.Linear(300, action_dim)
            self.head = nn.Linear(300, 1)

        def forward(self, state):
            x = functional.relu(self.l1(state))
            x = functional.relu(self.l2(x))
            scale = torch.sigmoid(self.head(x))
            return functional.normalize(torch.tanh(self.l3(x)), dim=1) * scale

    return JointActor()


def _make_path_actor(torch, nn, functional, state_dim: int, action_dim: int):
    class PathActor(nn.Module):
        def __init__(self):
            super().__init__()
            self.l1 = nn.Linear(state_dim, 400)
            self.l2 = nn.Linear(400, 300)
            self.dir_head = nn.Linear(300, 2)
            self.mag_head = nn.Linear(300, 1)
            self.tau_head = nn.Linear(300, 1) if action_dim >= 3 else None

        def forward(self, state):
            x = functional.relu(self.l1(state))
            x = functional.relu(self.l2(x))
            direction = functional.normalize(torch.tanh(self.dir_head(x)), dim=1, eps=1e-6)
            magnitude = torch.sigmoid(self.mag_head(x))
            force = direction * magnitude
            if self.tau_head is None:
                return force
            torque = torch.tanh(self.tau_head(x))
            return torch.cat([force, torque], dim=1)

    return PathActor()


def _clamp(value: float, limit: float) -> float:
    limit = abs(float(limit))
    return float(np.clip(value, -limit, limit))


def _normalized_xy(vector: Sequence[float]) -> tuple[float, float]:
    arr = np.asarray(vector, dtype=float).reshape(-1)
    if arr.size < 2:
        return 1.0, 0.0
    norm = float(np.linalg.norm(arr[:2]))
    if norm < 1e-9:
        return 1.0, 0.0
    return float(arr[0] / norm), float(arr[1] / norm)
