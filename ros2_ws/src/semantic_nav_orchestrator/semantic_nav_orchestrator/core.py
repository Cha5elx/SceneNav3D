"""Pure Python semantic navigation rules and approach-pose planning."""

from dataclasses import dataclass
import math
from typing import Callable, Optional

import yaml


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float = 0.0


@dataclass(frozen=True)
class SemanticObject:
    object_id: str
    label: str
    aliases: tuple[str, ...]
    x: float
    y: float
    z: float
    confidence: float
    status: str


@dataclass(frozen=True)
class Resolution:
    kind: str
    pose: Optional[Pose2D] = None
    semantic_object: Optional[SemanticObject] = None
    reason: str = ''

    @property
    def accepted(self) -> bool:
        return self.kind != 'rejected'


@dataclass(frozen=True)
class GridMap:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    data: tuple[int, ...]

    def cell(self, x: float, y: float) -> Optional[tuple[int, int]]:
        epsilon = 1e-9
        col = math.floor((x - self.origin_x) / self.resolution + epsilon)
        row = math.floor((y - self.origin_y) / self.resolution + epsilon)
        if col < 0 or row < 0 or col >= self.width or row >= self.height:
            return None
        return col, row

    def value(self, col: int, row: int) -> int:
        return self.data[row * self.width + col]

    def is_clear(self, x: float, y: float, clearance: float) -> bool:
        center = self.cell(x, y)
        if center is None:
            return False
        radius = math.ceil(clearance / self.resolution)
        center_col, center_row = center
        for row in range(center_row - radius, center_row + radius + 1):
            for col in range(center_col - radius, center_col + radius + 1):
                if col < 0 or row < 0 or col >= self.width or row >= self.height:
                    return False
                if math.hypot(col - center_col, row - center_row) > radius:
                    continue
                if self.value(col, row) != 0:
                    return False
        return True


class SemanticResolver:
    """Resolve configured names without guessing among ambiguous objects."""

    def __init__(self, locations: dict[str, dict], objects: list[SemanticObject]):
        self.locations = locations
        self.objects = objects

    @classmethod
    def from_files(cls, locations_path: str, objects_path: str):
        with open(locations_path, encoding='utf-8') as locations_file:
            locations = yaml.safe_load(locations_file)['locations']
        with open(objects_path, encoding='utf-8') as objects_file:
            raw_objects = yaml.safe_load(objects_file)['objects']
        objects = [
            SemanticObject(
                object_id=item['object_id'],
                label=item['label'],
                aliases=tuple(item.get('aliases', [])),
                x=float(item['position']['x']),
                y=float(item['position']['y']),
                z=float(item['position'].get('z', 0.0)),
                confidence=float(item.get('confidence', 1.0)),
                status=item.get('status', 'active'),
            )
            for item in raw_objects
        ]
        return cls(locations, objects)

    def resolve(self, command: str) -> Resolution:
        text = ''.join(command.lower().split())
        if not text:
            return Resolution('rejected', reason='empty command')

        location_matches = []
        for name, config in self.locations.items():
            aliases = [name, *config.get('aliases', [])]
            if any(''.join(alias.lower().split()) in text for alias in aliases):
                location_matches.append(config)
        if len(location_matches) == 1:
            pose = location_matches[0]['pose']
            return Resolution(
                'location',
                pose=Pose2D(float(pose['x']), float(pose['y']),
                            float(pose.get('yaw', 0.0))),
            )
        if len(location_matches) > 1:
            return Resolution('rejected', reason='ambiguous named location')

        active_matches = []
        inactive_match = False
        for semantic_object in self.objects:
            aliases = [semantic_object.label, *semantic_object.aliases]
            if any(''.join(alias.lower().split()) in text for alias in aliases):
                if semantic_object.status == 'active':
                    active_matches.append(semantic_object)
                else:
                    inactive_match = True
        active_matches = list({item.object_id: item for item in active_matches}.values())
        if len(active_matches) == 1:
            return Resolution('object', semantic_object=active_matches[0])
        if len(active_matches) > 1:
            return Resolution('rejected', reason='ambiguous semantic object')
        if inactive_match:
            return Resolution('rejected', reason='semantic object is inactive')
        return Resolution('rejected', reason='unknown target')


class ApproachPosePlanner:
    """Select a clear, reachable candidate around an object's center."""

    def __init__(self, grid_map: GridMap, approach_distance: float = 0.8,
                 clearance: float = 0.25, candidate_count: int = 16):
        self.grid_map = grid_map
        self.approach_distance = approach_distance
        self.clearance = clearance
        self.candidate_count = candidate_count

    def plan(self, semantic_object: SemanticObject, robot_pose: Pose2D,
             path_is_valid: Optional[Callable[[Pose2D], bool]] = None
             ) -> Optional[Pose2D]:
        candidates = self.candidates(semantic_object, robot_pose)
        if path_is_valid is not None:
            candidates = [pose for pose in candidates if path_is_valid(pose)]
        if not candidates:
            return None
        return candidates[0]

    def candidates(self, semantic_object: SemanticObject,
                   robot_pose: Pose2D) -> list[Pose2D]:
        candidates = []
        for index in range(self.candidate_count):
            angle = 2.0 * math.pi * index / self.candidate_count
            x = semantic_object.x + self.approach_distance * math.cos(angle)
            y = semantic_object.y + self.approach_distance * math.sin(angle)
            if not self.grid_map.is_clear(x, y, self.clearance):
                continue
            pose = Pose2D(
                x=x,
                y=y,
                yaw=math.atan2(semantic_object.y - y, semantic_object.x - x),
            )
            distance = math.hypot(x - robot_pose.x, y - robot_pose.y)
            candidates.append((distance, pose))
        candidates.sort(key=lambda item: item[0])
        return [pose for _, pose in candidates]
