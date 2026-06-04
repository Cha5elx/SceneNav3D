import math

from semantic_nav_orchestrator.core import (
    ApproachPosePlanner,
    GridMap,
    Pose2D,
    SemanticObject,
    SemanticResolver,
)


def make_object(object_id='chair_0001', status='active'):
    return SemanticObject(
        object_id=object_id,
        label='chair',
        aliases=('椅子', '红色椅子'),
        x=2.0,
        y=2.0,
        z=0.0,
        confidence=1.0,
        status=status,
    )


def make_grid(blocked=()):
    width = 20
    data = [0] * (width * width)
    for col, row in blocked:
        data[row * width + col] = 100
    return GridMap(width, width, 0.5, 0.0, 0.0, tuple(data))


def test_resolves_named_location():
    resolver = SemanticResolver(
        {'charging_area': {
            'aliases': ['充电区'],
            'pose': {'x': 1.0, 'y': 1.0, 'yaw': 0.0},
        }},
        [],
    )
    result = resolver.resolve('去充电区')
    assert result.kind == 'location'
    assert result.pose == Pose2D(1.0, 1.0, 0.0)


def test_resolves_unique_object():
    result = SemanticResolver({}, [make_object()]).resolve('去红色椅子旁边')
    assert result.kind == 'object'
    assert result.semantic_object.object_id == 'chair_0001'


def test_rejects_unknown_object():
    result = SemanticResolver({}, [make_object()]).resolve('去桌子附近')
    assert result.reason == 'unknown target'


def test_rejects_inactive_object():
    result = SemanticResolver({}, [make_object(status='inactive')]).resolve(
        '去椅子附近')
    assert result.reason == 'semantic object is inactive'


def test_rejects_ambiguous_object():
    objects = [make_object(), make_object(object_id='chair_0002')]
    result = SemanticResolver({}, objects).resolve('去椅子附近')
    assert result.reason == 'ambiguous semantic object'


def test_planner_selects_clear_candidate_facing_object():
    planner = ApproachPosePlanner(
        make_grid(), approach_distance=1.0, clearance=0.0, candidate_count=4)
    goal = planner.plan(make_object(), Pose2D(0.0, 2.0))
    assert goal is not None
    assert goal.x == 1.0
    assert math.isclose(goal.y, 2.0)
    assert math.isclose(goal.yaw, 0.0)


def test_planner_rejects_enclosed_object():
    blocked = [(6, 4), (4, 6), (2, 4), (4, 2)]
    planner = ApproachPosePlanner(
        make_grid(blocked), approach_distance=1.0, clearance=0.0,
        candidate_count=4)
    assert planner.plan(make_object(), Pose2D(0.0, 2.0)) is None


def test_planner_rejects_candidate_without_nav2_path():
    planner = ApproachPosePlanner(
        make_grid(), approach_distance=1.0, clearance=0.0, candidate_count=4)
    goal = planner.plan(
        make_object(), Pose2D(0.0, 2.0), path_is_valid=lambda pose: False)
    assert goal is None
