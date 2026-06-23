# RT 有効化手順 (setcap)

dora daemon と device_control_manager (DCM) がリアルタイム動作するために必要な
Linux capability を `setcap` で付与する手順。

## 背景

- RT スケジューリング (`SCHED_FIFO`) とメモリロック (`mlockall`) は通常ユーザでは権限不足で失敗する。
  - daemon: `RT: sched_setscheduler failed: ... CAP_SYS_NICE` / `RT: mlockall failed: ...`
  - DCM: `Warning: Failed to set RT Process priority`
- `setcap` で各バイナリに **必要な capability だけ** を付与すれば、sudo 無しで RT が通る。
  **「ビルドしたら setcap」**。
- **注意**: capability はバイナリファイルに紐づく。**再ビルドでバイナリが置き換わると消える**ので、
  ビルドのたびに打ち直す。

## capability の意味

| capability | 許可される操作 | 使う機能 |
|---|---|---|
| `cap_sys_nice` | `SCHED_FIFO` 等の RT スケジューリング設定 | FIFO 優先度 |
| `cap_ipc_lock` | `mlockall` (メモリロック) | ページフォルト防止 |

- `+ep` = effective + permitted で有効化する指定。
- 補足: 自スレッドの CPU affinity (`sched_setaffinity` / `pthread_setaffinity_np`) は
  **無権限で実行できる**ので capability は不要。

## 誰に何を付けるか

| 対象バイナリ | 付与する capability | 理由 |
|---|---|---|
| `dora` (daemon --rt) | `cap_sys_nice`, `cap_ipc_lock` | daemon が FIFO50 + mlockall |
| `device_control_manager` | `cap_sys_nice` のみ | FIFO80 のみ (mlockall 無し / affinity は無権限) |

## コマンド

### dora daemon (`dora daemon --rt` 用)

```bash
sudo setcap cap_sys_nice,cap_ipc_lock+ep ~/dora/target/release/dora
```
- `cargo build` で dora を再ビルドすると消える → 打ち直す。

### device_control_manager

```bash
sudo setcap cap_sys_nice+ep src/cpp/node/device_control_manager/build/device_control_manager
```
- cmake で DCM を再ビルドすると消える → ビルド後に打ち直す。

## 確認

付与されているか:
```bash
getcap ~/dora/target/release/dora
# → cap_ipc_lock,cap_sys_nice=ep
getcap src/cpp/node/device_control_manager/build/device_control_manager
# → cap_sys_nice=ep
```

起動後のログで成否を確認:
- daemon: `RT: mlockall enabled` / `RT: SCHED_FIFO priority 50 enabled` が出れば成功。
- DCM: `Warning: Failed to set RT Process priority` が **出なければ** 成功。

実際のスケジューリング / メモリロック:
```bash
chrt -p <pid>                    # → SCHED_FIFO なら成功
grep VmLck /proc/<pid>/status    # → 0 以外なら mlock 成功 (daemon)
```
