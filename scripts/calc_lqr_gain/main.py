"""A,B行列のCSVからLQRゲインを計算し、CSV出力する。

2DOF (3状態1入力) と 3DOF (4状態2入力) の両方に対応。
A,B行列のサイズから自動判定する。

入力: A.csv / A_3dof.csv, B.csv / B_3dof.csv（calc_ab_matrixの出力）
出力: lqrGain.csv
"""

import argparse
from pathlib import Path

import numpy as np
from scipy.linalg import solve_continuous_are

AB_DIR = Path(__file__).resolve().parents[1] / "calc_ab_matrix"


def load_matrix(path: Path) -> np.ndarray:
    return np.loadtxt(path, delimiter=",")


def check_controllability(A, B):
    n = A.shape[0]
    C = B.copy()
    for i in range(1, n):
        C = np.hstack([C, np.linalg.matrix_power(A, i) @ B])
    rank = np.linalg.matrix_rank(C)
    print(f"可制御性行列ランク: {rank}/{n}")
    return rank == n


def solve_lqr(A, B, Q, R):
    P = solve_continuous_are(A, B, Q, R)
    K = np.linalg.solve(R, B.T @ P)
    return K


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", choices=["2dof", "3dof"], default="3dof",
                        help="モデル選択: 2dof (3状態1入力) or 3dof (4状態2入力)")
    args = parser.parse_args()

    np.set_printoptions(precision=4, suppress=True)

    # 0. calc_ab_matrix を先に実行
    import subprocess, sys
    if args.model == "2dof":
        ab_script = AB_DIR / "calc_model_2dof_lagrange.py"
        a_file, b_file = "A.csv", "B.csv"
    else:
        ab_script = AB_DIR / "calc_model_3dof_lagrange.py"
        a_file, b_file = "A_3dof.csv", "B_3dof.csv"

    print(f"=== calc_ab_matrix 実行 ({args.model}) ===")
    ret = subprocess.run([sys.executable, str(ab_script)], cwd=str(AB_DIR))
    if ret.returncode != 0:
        print("calc_ab_matrix 失敗")
        sys.exit(1)
    print()

    # 1. A,B行列読み込み
    A = load_matrix(AB_DIR / a_file)
    B = load_matrix(AB_DIR / b_file)
    if B.ndim == 1:
        B = B.reshape(-1, 1)

    n = A.shape[0]
    m = B.shape[1]
    print(f"A ({n}×{n}):\n{A}")
    print(f"\nB ({n}×{m}):\n{B}")

    # 2. 開ループ固有値
    eigs = np.linalg.eigvals(A)
    print(f"\n開ループ固有値: {np.sort(eigs.real)}")

    # 3. 可制御性
    check_controllability(A, B)

    # 4. LQR設計（モデルに応じたQ,R）
    if n == 3 and m == 1:
        #            ṡ      φ      φ̇
        Q = np.diag([50.0, 0.01, 10.0])
        R = np.atleast_2d(10.0)
    elif n == 4 and m == 2:
        #            ṡ      φ      φ̇     α̇
        Q = np.diag([300.0, 0.01, 15.0, 40.0])
        #            T_φ    T_α
        R = np.diag([50.0, 50.0])
    else:
        print(f"未対応の次元: n={n}, m={m}")
        sys.exit(1)

    K = solve_lqr(A, B, Q, R)
    print(f"\nQ = diag{list(np.diag(Q))}")
    print(f"R = diag{list(np.diag(R))}")
    print(f"K ({m}×{n}):\n{K}")

    # 5. 閉ループ安定性
    A_cl = A - B @ K
    cl_eigs = np.linalg.eigvals(A_cl)
    print(f"\n閉ループ固有値: {np.sort(cl_eigs.real)}")
    stable = np.all(cl_eigs.real < 0)
    print(f"安定性: {'OK' if stable else 'NG'}")

    # 6. CSV出力
    import csv
    out_path = Path(__file__).parent / "lqrGain.csv"
    with open(out_path, "w", newline="") as f:
        writer = csv.writer(f)
        for row in K:
            writer.writerow([f"{v:.6f}" for v in row])
    print(f"\nCSV出力: {out_path}")


if __name__ == "__main__":
    main()
