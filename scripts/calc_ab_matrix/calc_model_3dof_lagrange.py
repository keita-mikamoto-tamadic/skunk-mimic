"""3CoM倒立振子の3次元ラグランジュ運動方程式をSymPyで導出し、A,B行列をCSV出力する。

一般化座標: q = [s, φ, α]（前進距離、ピッチ角、ヨー角）
3質点: ボディ(mb, Ib, It), 左右ホイール(mw, Iw) × 2
拘束: ホイールは地面に接触（すべりなし）
入力: T_φ = TL + TR（バランストルク）, T_α = (TR - TL) * r / D（ステアリングトルク）

出力: 5状態 [ṡ, φ, φ̇, α̇] の連続時間A,B行列
  ※ α は不可観測（ヨー角自体は動特性に現れない）のため状態から除外し、
    α̇（ヨーレート）のみ含む4状態とする
"""

import sys
from pathlib import Path

import numpy as np
import sympy as sp

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "lib"))
from model_loader import load_model
from three_com_model_crate import create_three_com_params


def derive_eom():
    """SymPyで3DOFラグランジュ運動方程式を導出し、線形化してA,B行列を返す。

    座標系（ボディ固定フレーム）:
      - s: 前進方向の移動距離（ヘディング方向）
      - z: 鉛直上向き
      - φ: ピッチ角（前傾が正）
      - α: ヨー角（左旋回が正、右手系 z軸上向き）

    ホイール拘束（すべりなし）:
      左ホイール角速度: ω_L = (ṡ - D/2 · α̇) / r
      右ホイール角速度: ω_R = (ṡ + D/2 · α̇) / r

    ボディCoM位置（ヘディング方向）:
      x_body = s + L sin(φ)  （ヘディング方向）
      z_body = r + L cos(φ)

    Returns:
        A_func, B_func: 3CoMパラメータを代入するとA,B行列(numpy)を返す関数
    """
    t = sp.Symbol('t')

    # 物理パラメータ（シンボリック）
    mb, mw, L, r, Ib, Iw, It, D, g, b_fric = sp.symbols(
        'mb mw L r Ib Iw It D g b', positive=True
    )

    # 一般化座標
    s = sp.Function('s')(t)      # 前進距離
    phi = sp.Function('phi')(t)  # ピッチ角
    alpha = sp.Function('alpha')(t)  # ヨー角

    s_dot = s.diff(t)
    phi_dot = phi.diff(t)
    alpha_dot = alpha.diff(t)

    # --- 運動エネルギー ---

    # ボディ（ヘディング方向並進 + 鉛直 + ピッチ回転 + ヨー回転）
    x_body = s + L * sp.sin(phi)
    z_body = r + L * sp.cos(phi)
    vx_body = x_body.diff(t)
    vz_body = z_body.diff(t)
    T_body = (sp.Rational(1, 2) * mb * (vx_body**2 + vz_body**2)
              + sp.Rational(1, 2) * Ib * phi_dot**2
              + sp.Rational(1, 2) * It * alpha_dot**2)

    # ホイール（左右独立、すべりなし）
    omega_L = (s_dot - D / 2 * alpha_dot) / r
    omega_R = (s_dot + D / 2 * alpha_dot) / r
    # 各ホイールの並進速度
    v_wL = s_dot - D / 2 * alpha_dot  # 左ホイール接地点速度
    v_wR = s_dot + D / 2 * alpha_dot  # 右ホイール接地点速度
    T_wheel = (sp.Rational(1, 2) * mw * (v_wL**2 + v_wR**2)
               + sp.Rational(1, 2) * Iw * (omega_L**2 + omega_R**2))

    T = T_body + T_wheel

    # --- ポテンシャルエネルギー ---
    V = mb * g * z_body

    # --- ラグランジアン ---
    Lag = T - V

    # --- 一般化力 ---
    T_phi = sp.Symbol('T_phi')    # バランストルク TL + TR
    T_alpha = sp.Symbol('T_alpha')  # ステアリングトルク (TR - TL) * r / D

    q = [s, phi, alpha]
    q_dot = [s_dot, phi_dot, alpha_dot]

    eom = []
    for qi, qi_dot in zip(q, q_dot):
        dLdqd = sp.diff(Lag, qi_dot)
        ddt_dLdqd = dLdqd.diff(t)
        dLdq = sp.diff(Lag, qi)
        eom.append(sp.simplify(ddt_dLdqd - dLdq))

    # 外力ベクトル（仮想仕事から導出）
    # s方程式: T_phi / r - b * s_dot / r
    # φ方程式: T_phi - b * phi_dot
    # α方程式: T_alpha * D / r - b * alpha_dot
    Q_forces = [
        T_phi / r - b_fric * s_dot / r,
        T_phi - b_fric * phi_dot,
        T_alpha * D / r - b_fric * alpha_dot,
    ]

    # EoM: eom[i] = Q[i]
    s_ddot = s.diff(t, 2)
    phi_ddot = phi.diff(t, 2)
    alpha_ddot = alpha.diff(t, 2)

    eom_eqs = [sp.Eq(eom[i], Q_forces[i]) for i in range(3)]

    print("=== 運動方程式（3DOF）===")
    for i, eq in enumerate(eom_eqs):
        print(f"  EoM[{i}]: {eq}")

    # 加速度について解く
    sol = sp.solve(eom_eqs, [s_ddot, phi_ddot, alpha_ddot])
    s_ddot_expr = sol[s_ddot]
    phi_ddot_expr = sol[phi_ddot]
    alpha_ddot_expr = sol[alpha_ddot]

    print("\n=== 加速度の式 ===")
    print(f"  s̈  = {sp.simplify(s_ddot_expr)}")
    print(f"  φ̈  = {sp.simplify(phi_ddot_expr)}")
    print(f"  α̈  = {sp.simplify(alpha_ddot_expr)}")

    # --- 線形化（φ=0 周りで展開）---
    # 状態ベクトル: [ṡ, φ, φ̇, α̇]  (αは不可観測、α̇のみ)
    # f1 = s̈
    # f2 = φ̇
    # f3 = φ̈
    # f4 = α̈

    # 置換用シンボル（関数→シンボル）
    s_s, s_dot_s = sp.symbols('s sd')
    phi_s, phi_dot_s = sp.symbols('phi phid')
    alpha_s, alpha_dot_s = sp.symbols('alpha alphad')

    subs_map = {
        s: s_s, s.diff(t): s_dot_s,
        phi: phi_s, phi.diff(t): phi_dot_s,
        alpha: alpha_s, alpha.diff(t): alpha_dot_s,
    }

    f1 = s_ddot_expr.subs(subs_map)
    f3 = phi_ddot_expr.subs(subs_map)
    f4 = alpha_ddot_expr.subs(subs_map)

    states = [s_dot_s, phi_s, phi_dot_s, alpha_dot_s]
    inputs = [T_phi, T_alpha]
    eq_point = {
        s_s: 0, s_dot_s: 0,
        phi_s: 0, phi_dot_s: 0,
        alpha_s: 0, alpha_dot_s: 0,
        T_phi: 0, T_alpha: 0,
    }

    # A行列 (4×4): ∂f/∂x
    A_sym = sp.Matrix([
        [sp.diff(f1, st) for st in states],
        [0, 0, 1, 0],
        [sp.diff(f3, st) for st in states],
        [sp.diff(f4, st) for st in states],
    ]).subs(eq_point)

    # B行列 (4×2): ∂f/∂u
    B_sym = sp.Matrix([
        [sp.diff(f1, u) for u in inputs],
        [0, 0],
        [sp.diff(f3, u) for u in inputs],
        [sp.diff(f4, u) for u in inputs],
    ]).subs(eq_point)

    A_simplified = sp.simplify(A_sym)
    B_simplified = sp.simplify(B_sym)

    print("\n=== 線形化 A行列（シンボリック, 4×4）===")
    sp.pprint(A_simplified)
    print("\n=== 線形化 B行列（シンボリック, 4×2）===")
    sp.pprint(B_simplified)

    # 数値変換関数を作成
    params = [mb, mw, L, r, Ib, Iw, It, D, g, b_fric]
    A_func = sp.lambdify(params, A_simplified, modules='numpy')
    B_func = sp.lambdify(params, B_simplified, modules='numpy')

    return A_func, B_func


def main():
    np.set_printoptions(precision=6, suppress=True)

    # 1. EoM導出（シンボリック、パラメータ非依存）
    A_func, B_func = derive_eom()

    # 2. MuJoCoモデルから3CoMパラメータ抽出
    model, data = load_model()
    p = create_three_com_params(model, data)

    print("\n=== 3CoMパラメータ ===")
    print(f"  mb={p.mb:.3f}, mw={p.mw:.4f}, L={p.L:.4f}, r={p.r:.5f}")
    print(f"  Ib={p.Ib:.6f}, Iw={p.Iw:.6f}, It={p.It:.6f}")
    print(f"  D={p.D:.4f}")

    # 3. 数値A,B行列を計算
    b_fric = 0.1
    g_val = 9.81
    A = np.array(A_func(p.mb, p.mw, p.L, p.r, p.Ib, p.Iw, p.It, p.D,
                         g_val, b_fric), dtype=float)
    B = np.array(B_func(p.mb, p.mw, p.L, p.r, p.Ib, p.Iw, p.It, p.D,
                         g_val, b_fric), dtype=float)

    print(f"\n=== A行列 (4×4) ===\n{A}")
    print(f"\n=== B行列 (4×2) ===\n{B}")

    eigs = np.linalg.eigvals(A)
    print(f"\n開ループ固有値: {np.sort(eigs.real)}")

    # 4. CSV出力
    import csv
    out_dir = Path(__file__).parent
    with open(out_dir / "A_3dof.csv", "w", newline="") as f:
        writer = csv.writer(f)
        for row in A:
            writer.writerow([f"{v:.8f}" for v in row])

    with open(out_dir / "B_3dof.csv", "w", newline="") as f:
        writer = csv.writer(f)
        for row in B:
            writer.writerow([f"{v:.8f}" for v in row])

    print(f"\nCSV出力: {out_dir}/A_3dof.csv, {out_dir}/B_3dof.csv")


if __name__ == "__main__":
    main()
