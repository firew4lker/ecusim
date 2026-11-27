import pytest

from ecusim.core.tables import Table1D, Table2D, clamp, interp1


def test_interp1_clamps_and_interpolates():
    xs, ys = [0, 10, 20], [0.0, 100.0, 50.0]
    assert interp1(xs, ys, -5) == 0.0
    assert interp1(xs, ys, 5) == pytest.approx(50.0)
    assert interp1(xs, ys, 15) == pytest.approx(75.0)
    assert interp1(xs, ys, 99) == 50.0


def test_table1d_rejects_unsorted_axis():
    with pytest.raises(ValueError):
        Table1D([0, 2, 1], [0, 0, 0])


def test_table2d_bilinear():
    t = Table2D([0, 10], [0, 1], [[0, 10], [100, 110]])
    assert t(5, 0.5) == pytest.approx(55.0)
    assert t(-1, 2) == pytest.approx(100.0)  # clamped corner


def test_clamp():
    assert clamp(5, 0, 1) == 1
    assert clamp(-5, 0, 1) == 0
    assert clamp(0.5, 0, 1) == 0.5
