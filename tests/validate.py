#!/usr/bin/env python3
"""Independent simulator and deterministic smoke tests for main.cpp."""
import argparse
import collections
import random
import subprocess
import time

N, M, K = 128, 150, 3


def cases(seed):
    rng = random.Random(seed)
    all_cells = [(x, y) for x in range(N) for y in range(N)]
    yield 'uniform', rng.sample(all_cells, M + 1)
    # Distinct goals, tightly clustered around the middle or near a border.
    for name, x0, y0 in [('cluster', 54, 54), ('border', 0, 0)]:
        cells = [(x0+x, y0+y) for x in range(16) for y in range(16)]
        yield name, rng.sample(cells, M + 1)
    # Alternating between distant corners; every cell remains distinct.
    quadrants = [[(bx+x, by+y) for x in range(8) for y in range(8)]
                 for bx, by in [(0, 0), (120, 120), (0, 120), (120, 0)]]
    for quadrant in quadrants:
        rng.shuffle(quadrant)
    points = [quadrants[i % 4][i // 4] for i in range(M + 1)]
    yield 'corners', points


def validate(binary, label, points):
    baseline = sum(abs(a-c) + abs(b-d)
                   for (a, b), (c, d) in zip(points, points[1:]))
    inp = f'{N} {M} {K} {baseline}\n' + ''.join(f'{x} {y}\n' for x, y in points)
    start = time.monotonic()
    result = subprocess.run([binary], input=inp, text=True, capture_output=True, timeout=4)
    elapsed = time.monotonic() - start
    assert result.returncode == 0, f'{label}: exit {result.returncode}: {result.stderr}'
    lines = result.stdout.splitlines()
    assert K <= len(lines) <= K + baseline, f'{label}: invalid output length'
    circles = collections.Counter()
    for line in lines[:K]:
        fields = line.split()
        assert len(fields) == 2, f'{label}: invalid circle line'
        x, y = map(int, fields)
        assert 0 <= x < N and 0 <= y < N, f'{label}: circle out of bounds'
        circles[x, y] += 1
    x, y = points[0]
    visited = 0
    directions = {'U': (-1, 0), 'D': (1, 0), 'L': (0, -1), 'R': (0, 1)}
    for line in lines[K:]:
        fields = line.split()
        assert fields, f'{label}: empty operation'
        if fields[0] == 'J':
            assert len(fields) == 3, f'{label}: invalid teleport'
            cx, cy = map(int, fields[1:])
            assert 0 <= cx < N and 0 <= cy < N and circles[cx, cy] > 0, f'{label}: no circle'
            circles[cx, cy] -= 1
            circles[x, y] += 1
            x, y = 2 * cx - x, 2 * cy - y
        else:
            assert len(fields) == 1 and fields[0] in directions, f'{label}: invalid walk'
            dx, dy = directions[fields[0]]
            x += dx
            y += dy
        assert 0 <= x < N and 0 <= y < N, f'{label}: out of bounds'
        if visited < M and (x, y) == points[visited + 1]:
            visited += 1
    assert visited == M, f'{label}: only {visited}/{M} goals visited'
    print(f'{label}: operations={len(lines)-K}/{baseline}, wall={elapsed:.3f}s')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('binary', help='path to the compiled solver')
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--uniform-cases', type=int, default=8)
    args = parser.parse_args()
    for name, points in cases(args.seed):
        validate(args.binary, name, points)
    for seed in range(args.uniform_cases):
        points = random.Random(seed).sample([(x, y) for x in range(N) for y in range(N)], M + 1)
        validate(args.binary, f'uniform-{seed}', points)


if __name__ == '__main__':
    main()
