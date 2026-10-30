import numpy as np
import pytest

from robot.navigation.planners import line_free, nearest_free, plan, simplify


def wall_grid():
    g = np.zeros((30, 30), bool)
    g[0:25, 15] = True          # wall with a gap at the top
    return g


@pytest.mark.parametrize("algo", ["astar", "dijkstra"])
def test_plan_around_wall(algo):
    g = wall_grid()
    path = plan(g, (5, 5), (25, 5), algo)
    assert path is not None
    assert path[0] == (5, 5) and path[-1] == (25, 5)
    assert all(not g[y, x] for x, y in path)
    assert any(y >= 25 for _, y in path)        # went through the gap


def test_astar_and_dijkstra_same_cost():
    g = wall_grid()
    a = plan(g, (5, 5), (25, 5), "astar")
    d = plan(g, (5, 5), (25, 5), "dijkstra")

    def cost(p):
        return sum(np.hypot(p[i + 1][0] - p[i][0], p[i + 1][1] - p[i][1]) for i in range(len(p) - 1))
    assert cost(a) == pytest.approx(cost(d))


def test_unreachable_goal():
    g = np.zeros((20, 20), bool)
    g[:, 10] = True
    assert plan(g, (2, 2), (18, 2)) is None


def test_goal_in_obstacle():
    g = np.zeros((10, 10), bool)
    g[5, 5] = True
    assert plan(g, (0, 0), (5, 5)) is None


def test_no_corner_cutting():
    g = np.zeros((5, 5), bool)
    g[1, 2] = True
    g[2, 1] = True
    path = plan(g, (1, 1), (2, 2))
    assert path is not None and len(path) > 2


def test_simplify_keeps_line_of_sight():
    g = wall_grid()
    path = plan(g, (5, 5), (25, 5))
    s = simplify(g, path)
    assert len(s) < len(path)
    for a, b in zip(s, s[1:]):
        assert line_free(g, a, b)


def test_nearest_free():
    g = np.ones((9, 9), bool)
    g[4, 7] = False
    assert nearest_free(g, (4, 4)) == (7, 4)
