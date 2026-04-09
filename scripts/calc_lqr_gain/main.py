"""A,B行列のCSVからLQRゲインを計算し、CSV出力する。

連続時間A,Bを離散化（dt=3ms）し、離散時間LQR（DARE）でゲインを求める。
2DOF (3状態1入力) と 3DOF (4状態2入力) の両方に対応。

入力: A.csv / A_3dof.csv, B.csv / B_3dof.csv（calc_ab_matrixの出力）
出力: lqrGain.csv
"""

import argparse
from pathlib import Path

import numpy as np
from scipy.linalg import solve_discrete_are
from scipy.signal import cont2discrete

AB_DIR = Path(__file__).resolve().parents[1] / "calc_ab_matrix"
DT = 0.003  # 制御周期 [s]


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


def solve_dlqr(Ad, Bd, Q, R):
    """離散時間LQR: x(k+1) = Ad x(k) + Bd u(k), u = -K x"""
    P = solve_discrete_are(Ad, Bd, Q, R)
    K = np.linalg.solve(R + Bd.T @ P @ Bd, Bd.T @ P @ Ad)
    return K


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", choices=["2dof", "3dof"], default="2dof",
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

    # 1. 連続時間A,B行列読み込み
    Ac = load_matrix(AB_DIR / a_file)
    Bc = load_matrix(AB_DIR / b_file)
    if Bc.ndim == 1:
        Bc = Bc.reshape(-1, 1)

    n = Ac.shape[0]
    m = Bc.shape[1]
    print(f"連続時間 Ac ({n}×{n}):\n{Ac}")
    print(f"\n連続時間 Bc ({n}×{m}):\n{Bc}")

    eigs_c = np.linalg.eigvals(Ac)
    print(f"\n連続時間 開ループ固有値: {np.sort(eigs_c.real)}")

    # 2. 離散化（ZOH）
    Cc = np.eye(n)
    Dc = np.zeros((n, m))
    (Ad, Bd, _, _, _) = cont2discrete((Ac, Bc, Cc, Dc), DT, method='zoh')

    print(f"\n離散時間 Ad ({n}×{n}), dt={DT*1000:.0f}ms:\n{Ad}")
    print(f"\n離散時間 Bd ({n}×{m}):\n{Bd}")

    eigs_d = np.linalg.eigvals(Ad)
    print(f"\n離散時間 開ループ固有値 (|λ|): {np.sort(np.abs(eigs_d))}")

    # 3. 可制御性（離散時間）
    check_controllability(Ad, Bd)

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

    K = solve_dlqr(Ad, Bd, Q, R)
    print(f"\nQ = diag{list(np.diag(Q))}")
    print(f"R = diag{list(np.diag(R))}")
    print(f"K ({m}×{n}):\n{K}")

    # 5. 閉ループ安定性（離散時間: |λ| < 1 で安定）
    A_cl = Ad - Bd @ K
    cl_eigs = np.linalg.eigvals(A_cl)
    print(f"\n閉ループ固有値: {cl_eigs}")
    print(f"閉ループ |λ|: {np.sort(np.abs(cl_eigs))}")
    stable = np.all(np.abs(cl_eigs) < 1.0)
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
