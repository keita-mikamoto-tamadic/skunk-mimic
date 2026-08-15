# PEAK PCAN-M.2 を SocketCAN として使う手順

PEAK-System の M.2 CAN-FD インターフェース (PCAN-M.2) を **SocketCAN (`canX` netdev)**
として動かすまでの手順。これにより既存の `SocketCanComm` (C++) がコード無改修で使える
（PEAK 専用 API = PCAN-Basic は不要）。

## 背景

- PEAK のカードは Linux SocketCAN にネイティブ対応するが、対応カーネルドライバ
  (`peak_pciefd` / `peak_canfd`) が **この Tegra カーネルには含まれていない**
  (`CONFIG_CAN_PEAK_PCIEFD is not set`)。
- そのため PEAK 公式の `peak-linux-driver` を **NETDEV (SocketCAN) モードで外部ビルド**して導入する。
- 検証環境:
  - カーネル: `6.8.12-1021-tegra` (aarch64, Jetson)
  - カード: `PEAK-System Technik GmbH Device 001a` @ PCIe `0004:01:00.0` (4ch PCAN-M.2)
  - ドライバ: `peak-linux-driver-8.20.0`

> ⚠️ ここで入れるモジュールは **カーネル更新でカーネルヘッダが変わると無効化される**。
> 更新のたびに再ビルド、または DKMS 化が必要（末尾参照）。

## 0. 事前診断（カードとドライバ状況の確認）

```bash
# カードが PCIe で見えるか (PEAK = Device 001a)
lspci | grep -i peak
# → 0004:01:00.0 Network controller: PEAK-System Technik GmbH Device 001a

# SocketCAN ドライバが入っているか (未コンパイルなら "is not set")
grep -i "PEAK" /boot/config-$(uname -r)
# → # CONFIG_CAN_PEAK_PCIEFD is not set   ← 未対応。外部ビルドが必要

# カーネルヘッダ（外部ビルドに必須）
ls -d /lib/modules/$(uname -r)/build   # あれば OK
```

現在の `can0` は **Tegra 内蔵 CAN** (`c310000.mttcan`) であり PEAK ではない点に注意。

## 1. ドライバのダウンロードと展開

```bash
cd ~
wget https://www.peak-system.com/fileadmin/media/linux/files/peak-linux-driver-8.20.0.tar.gz
tar xzf peak-linux-driver-8.20.0.tar.gz
cd peak-linux-driver-8.20.0
```

## 2. NETDEV (SocketCAN) モードでビルド

`NET=NETDEV_SUPPORT` が肝。これで `/dev/pcanX` チャーデバではなく **SocketCAN の `canX`** として登録される。

```bash
sudo apt install build-essential linux-headers-$(uname -r)   # 未導入なら
make -C driver NET=NETDEV_SUPPORT
```

成果物 `driver/pcan.ko` を確認:

```bash
modinfo driver/pcan.ko | grep -E "description|vermagic"
# description: Netdev driver for PEAK-System CAN interfaces   ← netdev 版
# vermagic:    6.8.12-1021-tegra ... aarch64                  ← 実行中カーネルと一致
```

## 3. ロード

### 3a. 試験ロード（`rmmod` で戻せる・恒久化しない）

```bash
sudo insmod ~/peak-linux-driver-8.20.0/driver/pcan.ko
dmesg | grep -i pcan | tail
# → pcan: registered CAN-FD netdevice can1 for pcifd hw (490,0)
#   ... can2/can3/can4 も同様（4ch カードのため）
```

### 3b. 恒久インストール（再起動後も残る／`modprobe pcan` が使える）

```bash
cd ~/peak-linux-driver-8.20.0
sudo make -C driver install NET=NETDEV_SUPPORT
sudo depmod -a
sudo modprobe pcan
```

## 4. チャンネル対応表（この環境）

| netdev | 実体 | 用途 |
|--------|------|------|
| `can0` | `c310000.mttcan` | Tegra 内蔵 CAN（`can_setup.bash` が管理） |
| `can1` | PEAK pcifd hw (490,0) | PEAK ch1 |
| `can2` | PEAK pcifd hw (490,1) | PEAK ch2 |
| `can3` | PEAK pcifd hw (490,2) | PEAK ch3 |
| `can4` | PEAK pcifd hw (490,3) | PEAK ch4 |

確認方法:

```bash
ip -br link show type can
dmesg | grep -i pcan          # can1..can4 = pcifd の登録ログ
```

## 5. インターフェースの起動（CAN-FD）

リポジトリ同梱の `pcan_setup.bash` を使う（`can_setup.bash` の PEAK 版）。
既定は PEAK ch1 = `can1`。チャンネルは複数指定可。最後の引数でモードを指定可。

```bash
bash pcan_setup.bash can1             # UP して待機。Ctrl-C / 終了で自動 DOWN (デフォルト)
bash pcan_setup.bash can0 can1        # 複数チャンネルを UP、Ctrl-C で全部 DOWN
bash pcan_setup.bash can0 can1 keep   # UP したまま終了 (従来挙動。ロボット常用時)
bash pcan_setup.bash can0 can1 down   # DOWN するだけ
bash pcan_setup.bash all down         # PEAK 全ch を DOWN (テスト後の一括掃除)
```

デフォルトが hold (UP したまま待機し、Ctrl-C で DOWN) になった点に注意。tview 等の
moteus ツールは UP な socketcan チャンネルを全て自動検出するため、テストで上げた
チャンネルを放置すると意図しないバスまで見え続ける。それを防ぐための挙動。

ビットレートは moteus 実績値 `1Mbps 名目 / 5Mbps データ` に合わせている。
起動後の確認（`can1` が `state ERROR-ACTIVE` = 健全、`<FD>` 表示、`mtu 72`）:

```
can <FD> state ERROR-ACTIVE (berr-counter tx 0 rx 0) restart-ms 100
  bitrate 1000000 sample-point 0.800
  dbitrate 5000000 dsample-point 0.750
  clock 80000000
```

> メモ: PEAK PCIe-FD のクロックは **80MHz**。名目は sample-point 0.8 を正確に取れるが、
> データ 5Mbps の `dsample-point` はドライバが 0.750 を自動選択する（mttcan の 50MHz では
> 0.8 が必要だったのに対しクロックが違うため）。moteus/相手ノードと BUS-OFF する場合のみ
> `pcan_setup.bash` の timing を調整する。

疎通確認:

```bash
candump can1                  # 相手からフレームが流れれば物理層 OK (can-utils)
```

## 6. skunk-mimic 側の設定

`robot_config` の `comm_ch` フィールド（実装済み）でチャンネルを指定する:

```json
{
  "comm_ch": ["can0", "can1"],
  "axes": [
    {"index": 0, "name": "hip_pitch_r", "device_id": 50, "comm_ch": 0, ...},
    {"index": 3, "name": "hip_pitch_l", "device_id": 80, "comm_ch": 1, ...}
  ]
}
```

- トップレベル `comm_ch` = SocketCAN netdev 名の順序付きリスト（省略時 `["can0"]`）。
- 各 axis の `comm_ch` = リストへのインデックス（省略時 0）。
- `device_control_manager` はチャンネル毎に `MotorDriver` を 1 インスタンスずつ
  開く（`Communication` / `MotorDriver` の抽象化はそのまま、`SocketCanComm` 共用）。
- device_id は**チャンネルをまたいで全体で一意**が必須（settings 系 API が
  device_id のみでモータを特定するため。重複はパース時エラー）。
- 2チャンネル実例: `robot_config/mimic_v2_5.json`（can0=右脚 / can1=左脚）。

PEAK 専用 API (PCAN-Basic) 版が将来必要になった場合は、`Communication` インターフェースを
コンストラクタ注入化して `PcanBasicComm` を追加する（`robot_config` に `can_backend`:
`socketcan` | `pcan` を追加）。ただし上記の SocketCAN 経路で足りるなら不要。

## 6b. 割り込み合体 (fdirqcl / fdirqtl) は既定のままにする

pcan.ko の `fdirqcl=16` (16 フレーム溜まるまで IRQ を上げない) / `fdirqtl=10`
(またはタイムリミット) は割り込み合体で、送信→応答の遅延が ~1.3ms に丸められる。
これを `fdirqcl=1 fdirqtl=1` にすると遅延は 0.4〜0.6ms に縮むが、**IRQ が
~4000/s (CPU0 固定、tegra-pcie と共有 INTA) に増えて DCM の 3ms tick が揺れ、
READY 補間がカクつく** (実測: can_max 1.6→4.5ms、補間中の停止 33→69 回)。
**既定 (16/10) のままにすること。** 遅延 1.3ms は DCM の 2ms 受信窓に収まっている。

> 参考: パラメータを変えるなら `/etc/modprobe.d/pcan.conf` の `options pcan ...` 行で
> (コマンドライン `modprobe pcan fdirqcl=…` は末尾の `install pcan modprobe
> --ignore-install pcan` に引数を捨てられて効かない)。反映は CAN down → rmmod →
> modprobe → up、確認は `/sys/module/pcan/parameters/fdirqcl`。

## 7. カーネル更新への対応（DKMS）

`insmod` / `make install` で入れたモジュールはカーネル更新で消える。恒久運用するなら
DKMS 化して自動再ビルドさせる:

```bash
cd ~/peak-linux-driver-8.20.0
sudo make -C driver dkms_install   # 以降カーネル更新時に自動リビルド
```

（DKMS ビルドも NETDEV モードにするには PEAK のドキュメント `Documentation/` を参照。）
