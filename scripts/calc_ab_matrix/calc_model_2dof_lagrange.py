"""3CoM倒立振子のラグランジュ運動方程式をSymPyで導出し、A,B行列をCSV出力する。

一般化座標: q = [xb, φ]（並進位置、ピッチ角）
3質点: ボディ(mb, Ib), 左右ホイール(mw, Iw) × 2
拘束: ホイールは地面に接触（すべりなし）
入力: T_φ = TL + TR（トータルトルク）

出力: バランス系 3状態 [ẋb, φ, φ̇] の連続時間A,B行列
"""

import sys
from pathlib import Path

import numpy as np
import sympy as sp

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "lib"))
from model_loader import load_model
from three_com_model_crate import create_three_com_params


def derive_eom():
    """SymPyでラグランジュ運動方程式を導出し、線形化してA,B行列を返す。

    座標系:
      - x: 前進方向（ホイール接地点の水平変位）
      - z: 鉛直上向き
      - φ: ピッチ角（前傾が正）

    ホイール拘束（すべりなし）:
      x_wheel = xb （ホイール接地点 = ボディ基準点）
      θ_wheel = xb / r （ホイール回転角）

    ボディCoM位置:
      x_body = xb + L sin(φ)
      z_body = r + L cos(φ)

    Returns:
        A_func, B_func: 3CoMパラメータを代入するとA,B行列(numpy)を返す関数
    """
    t = sp.Symbol('t')

    # 物理パラメータ（シンボリック）
    mb, mw, L, r, Ib, Iw, g, b_fric = sp.symbols(
        'mb mw L r Ib Iw g b', positive=True
    )

    # 一般化座標
    xb = sp.Function('xb')(t)
    phi = sp.Function('phi')(t)

    xb_dot = xb.diff(t)
    phi_dot = phi.diff(t)

    # --- 運動エネルギー ---
    # ボディ（並進 + 回転）
    x_body = xb + L * sp.sin(phi)
    z_body = r + L * sp.cos(phi)
    vx_body = x_body.diff(t)
    vz_body = z_body.diff(t)
    T_body = sp.Rational(1, 2) * mb * (vx_body**2 + vz_body**2) + \
             sp.Rational(1, 2) * Ib * phi_dot**2

    # ホイール × 2（並進 + 回転、すべりなし: θ_dot = xb_dot / r）
    omega_w = xb_dot / r
    T_wheel = 2 * (sp.Rational(1, 2) * mw * xb_dot**2 +
                   sp.Rational(1, 2) * Iw * omega_w**2)

    T = T_body + T_wheel

    # --- ポテンシャルエネルギー ---
    V = mb * g * z_body  # ホイールは高さ一定

    # --- ラグランジアン ---
    Lag = T - V

    # --- 一般化力 ---
    # T_φ はホイール反力トルク（ボディに対する反作用）
    # 散逸: b * (xb_dot/r - (-phi_dot)) ... 簡略化のためここでは省略
    T_phi = sp.Symbol('T_phi')

    # ラグランジュ方程式: d/dt(∂L/∂q_dot) - ∂L/∂q = Q
    # xb: 外力 = -T_phi/r（ホイールトルクの接地点反力）+ 摩擦
    # ※ すべりなし拘束で T_phi がホイール回転→地面反力→並進に作用
    # φ: 外力 = -T_phi（ホイールトルクのボディへの反作用）

    q = [xb, phi]
    q_dot = [xb_dot, phi_dot]

    eom = []
    for qi, qi_dot in zip(q, q_dot):
        dLdqd = sp.diff(Lag, qi_dot)
        ddt_dLdqd = dLdqd.diff(t)
        dLdq = sp.diff(Lag, qi)
        eom.append(sp.simplify(ddt_dLdqd - dLdq))

    # 外力ベクトル
    # xb方程式: 力 = T_phi / r（トルク→接地力）- b * xb_dot / r（粘性摩擦）
    # φ方程式: トルク = +T_phi（正トルク→前進→前傾）- b * phi_dot（粘性摩擦）
    Q = [T_phi / r - b_fric * xb_dot / r, T_phi - b_fric * phi_dot]

    # EoM: M(q) * q_ddot + h(q, q_dot) = Q
    # eom[i] = Q[i] を整理
    xb_ddot = xb.diff(t, 2)
    phi_ddot = phi.diff(t, 2)

    eom_eqs = [sp.Eq(eom[i], Q[i]) for i in range(2)]

    print("=== 運動方程式 ===")
    for i, eq in enumerate(eom_eqs):
        print(f"  EoM[{i}]: {eq}")

    # q_ddot について解く
    sol = sp.solve(eom_eqs, [xb_ddot, phi_ddot])
    xb_ddot_expr = sol[xb_ddot]
    phi_ddot_expr = sol[phi_ddot]

    print("\n=== 加速度の式 ===")
    print(f"  ẍb = {sp.simplify(xb_ddot_expr)}")
    print(f"  φ̈  = {sp.simplify(phi_ddot_expr)}")

    # --- 線形化（φ=0, φ̇=0, ẋb=0 周りで展開）---
    # 状態ベクトル: [ẋb, φ, φ̇]
    # f1 = ẍb(ẋb, φ, φ̇, T_phi)
    # f2 = φ̇
    # f3 = φ̈(ẋb, φ, φ̇, T_phi)

    # 置換用シンボル（関数→シンボル）
    xb_s, xb_dot_s, phi_s, phi_dot_s = sp.symbols('xb xbd phi phid')

    subs_map = {
        xb: xb_s, xb.diff(t): xb_dot_s,
        phi: phi_s, phi.diff(t): phi_dot_s,
    }

    f1 = xb_ddot_expr.subs(subs_map)
    f3 = phi_ddot_expr.subs(subs_map)

    states = [xb_dot_s, phi_s, phi_dot_s]
    eq_point = {xb_s: 0, xb_dot_s: 0, phi_s: 0, phi_dot_s: 0, T_phi: 0}

    # A行列: ∂f/∂x
    A_sym = sp.Matrix([
        [sp.diff(f1, s) for s in states],
        [0, 0, 1],
        [sp.diff(f3, s) for s in states],
    ]).subs(eq_point)

    # B行列: ∂f/∂u
    B_sym = sp.Matrix([
        [sp.diff(f1, T_phi)],
        [0],
        [sp.diff(f3, T_phi)],
    ]).subs(eq_point)

    A_simplified = sp.simplify(A_sym)
    B_simplified = sp.simplify(B_sym)

    print("\n=== 線形化 A行列（シンボリック）===")
    sp.pprint(A_simplified)
    print("\n=== 線形化 B行列（シンボリック）===")
    sp.pprint(B_simplified)

    # 数値変換関数を作成
    params = [mb, mw, L, r, Ib, Iw, g, b_fric]
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
    print(f"  Ib={p.Ib:.6f}, Iw={p.Iw:.6f}")

    # 3. 数値A,B行列を計算
    b_fric = 0.1  # 粘性摩擦係数
    g = 9.81
    A = np.array(A_func(p.mb, p.mw, p.L, p.r, p.Ib, p.Iw, g, b_fric),
                 dtype=float)
    B = np.array(B_func(p.mb, p.mw, p.L, p.r, p.Ib, p.Iw, g, b_fric),
                 dtype=float)

    print(f"\n=== A行列 (3×3) ===\n{A}")
    print(f"\n=== B行列 (3×1) ===\n{B}")

    eigs = np.linalg.eigvals(A)
    print(f"\n開ループ固有値: {np.sort(eigs.real)}")

    # 4. CSV出力（トルク入力のA,B行列をそのまま出力）
    import csv
    out_dir = Path(__file__).parent
    with open(out_dir / "A.csv", "w", newline="") as f:
        writer = csv.writer(f)
        for row in A:
            writer.writerow([f"{v:.8f}" for v in row])

    with open(out_dir / "B.csv", "w", newline="") as f:
        writer = csv.writer(f)
        for row in B:
            writer.writerow([f"{v:.8f}" for v in row])

    print(f"\nCSV出力: {out_dir}/A.csv, {out_dir}/B.csv (torque input)")


if __name__ == "__main__":
    main()
