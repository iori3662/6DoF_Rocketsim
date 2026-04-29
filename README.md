# Hybrid Rocket 6DoF Simulator

ハイブリッドロケット向けのローカル実行型6自由度シミュレータです。C++20/CMakeで実装し、WindowsではGUI、全環境でCLIを提供します。

## 現在できること

- 手入力CSVの機体モデル読み込み
- 推力履歴CSV読み込み
- 風モデルCSV読み込み
- Barrowman methodによる風圧中心の概算
- 6DoF状態量のRK4積分
- 軌道CSV出力
- Google Earth向けKML出力
- WindowsローカルGUI
- GitHub Actionsでビルド・テスト・成果物作成

## ビルド

```powershell
cmake -S . -B build
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

MinGW環境でCMake/Makeが日本語パスを扱えない場合は、ローカル用スクリプトを使えます。

```powershell
.\scripts\build_mingw.ps1
```

## CLI実行例

```powershell
.\build\Release\hrocket_cli.exe --vehicle samples\vehicle.csv --thrust samples\thrust.csv --wind samples\wind.csv --out out
```

落下分散を出したい場合:

```powershell
.\build\Release\hrocket_cli.exe --vehicle samples\vehicle.csv --thrust samples\thrust.csv --wind samples\wind.csv --out out --descent parachute --wind-mode nominal --dispersion 100 --wind-sigma 2.0 --seed 42
```

降下方式は `--descent freefall` または `--descent parachute`、風は `--wind-mode nominal` または `--wind-mode calm` を選択できます。

FROGS型の風速・風向スイープで落下分散を見る場合:

```powershell
.\build\Release\hrocket_cli.exe --vehicle samples\vehicle.csv --thrust samples\thrust.csv --wind samples\wind.csv --out out --descent parachute --dispersion-mode wind-sweep --sweep-max-wind 7 --sweep-step 1 --sweep-directions 16
```

このモードでは、ノミナル1ケースに加えて、風速 `1..sweep-max-wind` と `sweep-directions` 方位の全組み合わせを計算し、`dispersion.csv` に風速・風向・着地点を保存します。これはFROGSの `FROGSghp.m` が風速レンジと16方位を回して着地点群を作る考え方に合わせたものです。

出力:

- `out\trajectory.csv`
- `out\trajectory.kml` は太線の軌道と落下分散点を含み、Google Earthで表示できます。
- `out\summary.csv`
- `out\dispersion.csv`
- `out\graph_trajectory.svg`
- `out\graph_profile.svg`
- `out\graph_attitude.svg`
- `out\graph_velocity.svg`
- `out\graph_control.svg`
- `out\graph_dispersion.svg`

## GUI

Windowsでは `hrocket_gui.exe` を起動し、機体・推力・風CSVと出力フォルダを指定して実行します。

GUIでは以下を選択できます。

- 降下方式: 自由落下 / パラシュート
- 風条件: CSVノミナル風 / 無風
- 落下分散run数
- 風ばらつき標準偏差
- 乱数seed
- 分散方式: Monte Carlo / Wind sweep
- Wind sweep最大風速・方位数

実行後、画面右側で以下のグラフを切り替えて確認できます。

- 軌道
- 高度 / 速度 / 推力
- 姿勢
- 速度成分
- 可動尾翼制御
- 落下分散

同じグラフはSVGとして出力フォルダにも保存されます。
SVGグラフには軸目盛りと数値ラベルが含まれます。

## 入力CSV

### 機体モデル

`samples/vehicle.csv` を参照してください。基本形式は `key,value` です。
パラシュート諸元も同じCSVに `parachute_area_m2`、`parachute_cd`、`parachute_deploy_altitude_m` として入力します。

可動尾翼による姿勢制御も同じCSVで設定します。ランチャ離脱後かつ指定対気速度以上でPD制御を有効にします。

- `control_fin_count,2`: `reference/control_model.jpg` のように機軸に対して対称に取り付けられた2枚翼を想定し、ロール制御のみを行います。
- `control_fin_count,4`: 4枚翼を想定し、ロール・ピッチ・ヨー制御を行います。

設定例は `samples/vehicle_control_symmetric.csv` と `samples/vehicle_control_4fin.csv` です。

主な設定項目:

- `control_enabled`
- `control_fin_count`
- `control_tail_area_m2`
- `control_tail_lift_slope_per_rad`
- `control_tail_distance_from_nose_m`
- `control_max_deflection_deg`
- `control_min_speed_mps`
- `control_roll_target_deg`
- `control_pitch_target_deg`
- `control_yaw_target_deg`
- `control_roll_kp`
- `control_roll_kd`
- `control_pitch_kp`
- `control_pitch_kd`
- `control_yaw_kp`
- `control_yaw_kd`

### 推力履歴

```csv
time_s,thrust_n
0,0
0.1,1200
```

### 風モデル

```csv
altitude_m,wind_north_mps,wind_east_mps,wind_down_mps
0,0,0,0
1000,5,1,0
```

## 今後の拡張候補

- Monte Carlo落下分散
- 制御器・アクチュエータのGUI設定
- 詳細な空力係数テーブル
- 地球楕円体/ENU/NED変換の高精度化
