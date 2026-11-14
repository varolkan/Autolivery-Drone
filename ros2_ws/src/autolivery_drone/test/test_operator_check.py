"""Every launch shows the license terms; a real vehicle is only flown once they are accepted."""

import importlib.util
import os

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.utilities import perform_substitutions
import pytest

SOURCE = os.environ.get(
    'AUTOLIVERY_DRONE_SOURCE', os.path.join(os.path.dirname(__file__), '..'))
LAUNCH = os.path.join(SOURCE, 'launch', 'autolivery.launch.py')


def _load():
    spec = importlib.util.spec_from_file_location('autolivery_launch', LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


STACK = _load()


def _context(**arguments):
    context = LaunchContext()
    context.launch_configurations.update(arguments)
    return context


def test_real_vehicle_is_refused_without_acceptance():
    with pytest.raises(RuntimeError, match='accept_risk:=true') as refused:
        STACK._operator_check(_context(simulation='false', accept_risk='false'))
    # What the operator is shown is the safety notice itself, and where the license is.
    assert 'solely' in str(refused.value)
    assert 'responsible' in str(refused.value)
    assert 'LICENSE.md' in str(refused.value)


def test_only_an_explicit_true_counts():
    for answer in ('', 'yes', 'True', '1', 'true '):
        with pytest.raises(RuntimeError):
            STACK._operator_check(_context(simulation='false', accept_risk=answer))


def test_real_vehicle_flies_once_accepted_and_the_acceptance_is_logged():
    actions = STACK._operator_check(_context(simulation='false', accept_risk='true'))
    assert len(actions) == 1
    assert isinstance(actions[0], LogInfo)
    logged = perform_substitutions(_context(), actions[0].msg)
    assert 'accepted the license (LICENSE.md) and responsibility' in logged


def test_simulation_needs_no_acceptance():
    assert STACK._operator_check(_context(simulation='true', accept_risk='false')) == []


def test_refusal_is_the_default_and_comes_before_anything_starts():
    entities = STACK.generate_launch_description().entities
    defaults = {
        e.name: perform_substitutions(_context(), e.default_value)
        for e in entities if isinstance(e, DeclareLaunchArgument)}
    assert defaults['accept_risk'] == 'false'
    assert defaults['simulation'] == 'false'
    # The check is the first thing run: before MAVROS is included and before any node.
    first = next(e for e in entities if isinstance(e, (OpaqueFunction, IncludeLaunchDescription)))
    assert isinstance(first, OpaqueFunction)


def test_every_launch_shows_the_license_terms_first():
    entities = STACK.generate_launch_description().entities
    # The first thing a launch does, whatever its arguments, simulated or not.
    first = next(e for e in entities if not isinstance(e, DeclareLaunchArgument))
    assert isinstance(first, LogInfo)
    assert first.condition is None
    shown = perform_substitutions(_context(), first.msg)
    # Word for word the Required Notice at the top of LICENSE.md, so the two cannot drift.
    with open(os.path.join(SOURCE, '..', '..', '..', 'LICENSE.md')) as f:
        notices = [line for line in f.read().splitlines() if line.startswith('Required Notice: ')]
    terms = next(n for n in notices if 'licensed under' in n)[len('Required Notice: '):]
    assert terms in shown
    assert 'Using it means accepting those terms: read LICENSE.md.' in shown


def test_dry_run_and_gazebo_launches_mark_themselves_as_simulation():
    for path in (os.path.join(SOURCE, 'launch', 'dry_run.launch.py'),
                 os.path.join(SOURCE, '..', 'autolivery_gazebo', 'launch', 'sim.launch.py')):
        with open(path) as f:
            assert "'simulation': 'true'" in f.read(), path
