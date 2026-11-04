from __future__ import annotations

import numpy as np
from algorithms.learned_gap import (
    N_BEAM_FEATURES,
    N_GLOBAL,
    B2Params,
    BeamScorerPolicy,
    LearnedGapController,
    StructuredGapPolicy,
    b2_default_vector,
    b2_expand_to_hidden,
    b2_from_b1,
    b2_layout,
    beam_features,
)
from algorithms.registry import CONTROLLERS
from onboard.api import Actuation
from simulator.simulation import Simulation


def test_learned_gap_is_dashboard_selectable() -> None:
    spec = CONTROLLERS["learned_gap"]
    assert spec.id == "learned_gap"
    assert "learned" in spec.label.lower()
    assert isinstance(spec.build(), LearnedGapController)


def test_b2_layout_round_trips_for_linear_and_hidden_policies() -> None:
    for hidden in (0, 4, 16):
        vector = b2_default_vector(hidden)
        assert vector.size == b2_layout(hidden)["_size"]
        params = B2Params.from_vector(vector, hidden)
        assert params.beam_linear_w.shape == (N_BEAM_FEATURES,)
        assert params.beam_res_w1.shape == (N_BEAM_FEATURES, hidden)
        assert params.speed_linear_w.shape == (N_BEAM_FEATURES + N_GLOBAL,)
        assert params.speed_res_w1.shape == (N_BEAM_FEATURES + N_GLOBAL, hidden)


def test_b2_expands_a_linear_policy_without_changing_its_scores() -> None:
    linear = b2_default_vector(0)
    expanded = b2_expand_to_hidden(linear, hidden=8)
    assert expanded.size == b2_layout(8)["_size"]

    angles = np.linspace(-0.6, 0.6, 36)
    ranges = np.linspace(0.4, 3.0, 36)
    valid = np.ones(36, dtype=bool)
    features = beam_features(angles, ranges, valid, max_range=5.0, gap_threshold=0.6)

    linear_params = B2Params.from_vector(linear, 0)
    expanded_params = B2Params.from_vector(expanded, 8)
    linear_scores = features @ linear_params.beam_linear_w + linear_params.beam_linear_b
    expanded_scores = features @ expanded_params.beam_linear_w + expanded_params.beam_linear_b
    expanded_scores += (
        np.tanh(features @ expanded_params.beam_res_w1 + expanded_params.beam_res_b1)
        @ expanded_params.beam_res_w2
        + expanded_params.beam_res_b2
    )
    np.testing.assert_allclose(expanded_scores, linear_scores, atol=1e-9)


def test_b2_from_b1_has_the_linear_layout() -> None:
    # The shipped policy may already be B2; build from an explicit B1 vector.
    b1_vector = np.linspace(0.2, 0.8, 23)
    vector = b2_from_b1(b1_vector)
    assert vector.size == b2_layout(0)["_size"]
    assert np.all(np.isfinite(vector))


def test_shipped_weights_load_and_produce_bounded_actuation() -> None:
    with Simulation(
        "configs/sim/awake_2027.yaml",
        realism="realistic",
        lidar_subrays=1,
        seed=42,
    ) as simulation:
        simulation.scenario.race.laps = 1
        observation = simulation.reset()
        controller = LearnedGapController()
        assert controller.policy_type in {"b1", "b2"}
        controller.reset()
        for _ in range(30):
            command = controller.update(observation)
            assert isinstance(command, Actuation)
            assert 0.0 <= command.throttle <= 1.0
            assert abs(command.steering) <= observation.limits.max_steering_rad + 1e-9
            observation = simulation.step(command)


def test_corrupt_weights_fall_back_to_the_default_policy(tmp_path) -> None:
    path = tmp_path / "broken.npz"
    path.write_bytes(b"not a numpy archive")
    controller = LearnedGapController(path)
    assert controller.policy_type == "b1"
    assert controller.meta.get("source") == "fallback"


def test_structured_policy_exposes_a_finite_score_surface() -> None:
    with Simulation(
        "configs/sim/awake_2027.yaml",
        realism="ideal",
        lidar_subrays=1,
        seed=42,
    ) as simulation:
        observation = simulation.reset()
        for _ in range(3):
            observation = simulation.step(Actuation())
        policy = StructuredGapPolicy()
        surface = policy.score_surface(observation)
        assert surface is not None
        scores = np.asarray(surface["scores"])
        assert np.isfinite(scores).any()
        assert np.isneginf(scores[~np.asarray(surface["front"], dtype=bool)]).all()


def test_beam_scorer_policy_is_finite_with_hidden_layer() -> None:
    vector = b2_expand_to_hidden(b2_default_vector(0), hidden=8)
    policy = BeamScorerPolicy(vector, hidden=8)
    policy.reset()
    with Simulation(
        "configs/sim/awake_2027.yaml",
        realism="ideal",
        lidar_subrays=1,
        seed=42,
    ) as simulation:
        observation = simulation.reset()
        for _ in range(3):
            observation = simulation.step(Actuation())
        command = policy.act(observation)
        assert 0.0 <= command.throttle <= 1.0
        assert np.isfinite(command.steering)
