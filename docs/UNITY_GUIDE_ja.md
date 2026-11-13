# Unity 組み込み手順書（汎用 ＋ 箱庭）

目的：フライトログを変換し、Unity で最初の 1 回を再生できるようにする。

- **A. 汎用手順**：どの Unity プロジェクトでも使える手順。空のプロジェクトと四角の物体から始めてもよい。
- **B. 箱庭での使い方**：hakoniwa-unity-drone の WebAvatar シーンに組み込む例。
- **トラブルシューティング**

リプレイ中は、シミュレータ（箱庭コア・SITL・webserver）を起動する必要はありません。Unity エディタだけで動きます。

---

## 0. ログを変換する（共通）

```bash
cd longarm-drone-replay
pip install -r python/requirements.txt
python tests/test_frames.py               # すべて ok になることを確認
```

### ArduPilot の .bin の場合

```bash
python python/bin2replay.py D:\logs\00000042.BIN --dump-fields
python python/bin2replay.py D:\logs\00000042.BIN -o out\flight_replay --skip-gaps
```

ArduPilot は、通常アームしている（モーターが回せる）間だけログを記録する。そのため、1 本のログの中に「飛んでいない空白の時間」が何分も入ることがある。`--skip-gaps` を付けると、その空白を取り除き、飛行の区間（セグメント）だけをつなげて 1 本のリプレイにする。変換の最後に、各区間のリプレイ上の時刻と、元のログの時刻が表示される。

`--dump-fields` では、`ATT` に Roll/Pitch/Yaw があり、`XKF1` に PN/PE/PD があることを確認する。`XKF1` がない場合は `--pos-source nkf1` か `--pos-source pos` を使う。

### CSV の場合

まず列名を確認する。

```bash
python python/csv2replay.py D:\logs\log.csv --dump-columns
```

次に、どの列が「時刻・位置・姿勢」かと、座標の決め方を指定する。迷ったら下の表で選ぶ。

| ログの出どころ | 位置 `--pos-frame` | 姿勢 `--att-frame` | 姿勢の列 |
|---|---|---|---|
| ArduPilot / PX4 を CSV にしたもの | `ned` | `ned_frd` | `--euler-cols`（度なら `--angle-unit deg`） |
| ROS（odom・pose）を CSV にしたもの | `enu` | `enu_flu` | `--quat-cols x,y,z,w --quat-order xyzw` |
| GPS の緯度・経度・高度だけ | `geo` | 姿勢の列に合わせる | |
| Unity で記録したもの | `unity` | `unity`（クォータニオンのみ） | |

```bash
python python/csv2replay.py D:\logs\odom.csv --time-col stamp --pos-cols x,y,z --pos-frame enu ^
  --quat-cols qx,qy,qz,qw --quat-order xyzw --att-frame enu_flu --arm-length 0 -o out\flight_replay
```

`[done] ... frames=xxxx valid=yyyy` と表示されれば成功。アームがない機体では `--arm-length 0` を付ける。

### アームの付け根の位置（`--arm-mount`）

> Unity で見るだけなら、Inspector の Arm Mount Reference／Arm Mount Local で指定する方が簡単（A-3）。`--arm-mount` は、CSV に出す先端位置を実機の寸法に合わせたいとき（グラフや評価に使うとき）に使う。

何も指定しないと、アームは機体の中心を通る（論文の式と同じ）。実機のアームが中心より上や下、前に付いている場合は、付け根の位置を「前,右,下」の順に m で指定する。上にずれているときは「下」をマイナスにする。

```bash
# 付け根が中心より 8 cm 上
python python/bin2replay.py 00000004.BIN -o out\flight_replay --skip-gaps --arm-mount 0,0,-0.08
# 1 つ目の値がマイナスのときは = でつなぐ
python python/bin2replay.py 00000004.BIN -o out\flight_replay --arm-mount=-0.05,0,-0.08
```

Unity のモデルから値を読む場合：アームの付け根のオブジェクト（例：ArmBase）を選び、Transform の Position（親から見た位置）を読む。そのうえで、次のように並べ替える。

| Unity の Position | `--arm-mount` の値 |
|---|---|
| Z（前） | 前 |
| X（右） | 右 |
| Y（上） | 下 ＝ **−Y** |

親のオブジェクト（F450 など）の Scale が 1 でない場合は、その倍率を掛ける。また、モデルのピボットが機体の中心にあることが前提（A-3）。

---

## A. 汎用手順（どの Unity プロジェクトでも）

### A-1. ファイルを置く

```
<UnityProject>/Assets/
├── FlightReplay/               ← unity/Assets/FlightReplay/ をフォルダごとコピー（Scripts と Editor）
└── StreamingAssets/            ← なければ作る（綴りは正確に）
    └── flight_replay.json
```

Console にコンパイルエラーが出ていないことを確認する。

### A-2. 一式を作る（メニュー 1 回）

- **四角で試す場合**：メニューの **Tools → FlightReplay → Create Replay Rig** を選ぶ。
- **自分の物体（丸・模型など）を使う場合**：Hierarchy でその物体を選んでから、**Tools → FlightReplay → Create Replay Rig (selected object = vehicle)** を選ぶ。

これで次のものが自動で作られる。

| 作られるもの | 役割 |
|---|---|
| `ReplayOrigin` | ログの原点。これを動かすと、軌跡全体の置き場所が変わる |
| FlightReplayPlayer | 再生と、機体の位置・姿勢の更新 |
| FlightReplayVisualizer | アーム・先端・軌跡・色分けの表示 |
| FlightReplayUI | 画面下の操作パネル |
| FlightReplayGrid | 1 m 間隔の地面の格子（赤＝東、青＝北） |
| Main Camera の FlightReplayFollowCamera | 機体を追いかけるカメラ（右ドラッグで回転、ホイールで拡大縮小） |

不要なものは、コンポーネントのチェックを外せば止まる。

### A-3. 設定を確認する

ReplayOrigin の FlightReplayPlayer で、次の項目を確認する。

| 項目 | 内容 |
|---|---|
| Json Path | `flight_replay.json`（StreamingAssets 内のファイル名） |
| Target | 空なら仮の機体を自動で作る。自分の物体を使う場合はそれが入っている |
| Placeholder Shape / Size | 仮の機体の形（Box・Sphere・Capsule）と大きさ（初期値は F450 程度の 0.45×0.12×0.45 m） |
| Model Euler Offset | 自分の物体の前がローカル +Z でないときの補正（T2） |
| Height Offset | 地面にめり込むときの上下の補正 |
| Arm From Inspector | オン（初期値）：アームの長さ・付け根を下の 3 項目で決め、Unity で先端を計算する。オフ：変換時に Python で計算した先端（`--arm-length` / `--arm-mount`）を使う |
| Arm Length | アームの長さ [m]。0 にするとアームを表示しない |
| Arm Mount Local | 付け根の位置 [m]。並びは **x＝右、y＝上、z＝前**（機首が +Z のモデルなら、子オブジェクトの Position をそのまま書き写せる） |
| Arm Mount Reference | モデルにアームの付け根のオブジェクト（例：ArmBase）があれば、ここにドラッグする。その位置が自動で付け根になり、Arm Mount Local より優先される |

自分の物体を使うときは、次の 2 点に注意する。

- 物体の原点（ピボット）が機体の中心にあること。アームは「物体の原点 → 記録された先端」の間に描かれるため。
- 物体に、動きを書き込む別のスクリプトや Animator が付いていたら無効にする。Rigidbody は Player が自動で kinematic にする。

### A-4. 再生する

Play を押す。Console に `[FlightReplay] loaded N frames` と出れば成功。

操作パネルの `log t` は、元のログの時刻（.bin では起動からの秒数）。Mission Planner でログのグラフを見るときの時刻と照らし合わせられる。`seg 2/5` は、いま 5 区間中の 2 番目を再生していることを示す。区間の切り替わりでは機体が瞬間移動することがあるが、その間を補間はせず、軌跡の線もリセットする。

| 操作 | ボタン | キー（Game ビューをクリックしてから） |
|---|---|---|
| 再生／一時停止 | Play / Pause | Space |
| 先頭へ | `|<` | – |
| ±1 秒 | -1s / +1s | ← / → |
| 前／次の区間へ | `<seg` / `seg>`（区間が 2 つ以上のときに表示） | PageUp / PageDown |
| 速度 | x0.1〜x8 | ↑（2 倍）/ ↓（半分） |
| 任意の時刻へ | スライダー | – |
| 視点 | – | 右ドラッグ（回転）／ホイール（拡大縮小） |
| パネルを隠す／出す | Hide [H] | H |

Game ビューが狭いときは、パネル全体が自動で縮小される。パネルを上に置きたい場合は FlightReplayUI の Anchor を Top に、文字が小さすぎる場合は Ui Scale を上げる。

### A-5. 向きを確認する（最初の 1 回は必ず）

- 機首（仮の機体なら赤い印、アームがあればアーム）が、ログのヨー角 90° の時刻に青い線（北）から時計回りに 90°、つまり赤い線（東）の方向を向いていれば正しい。
- 前進している場面で機首が下がっていれば、ピッチの符号も正しい。
- ROS のログでは、ヨー 0 のときに機首が東を向く（ROS の決まり）。

### 飛行データの表示（FlightReplayTelemetry）

画面左上に、いまの時刻の値を表示する。

| 表示 | 内容 |
|---|---|
| log t | 元のログの時刻 [s]（.bin では起動からの秒数） |
| North / East | 原点（最初の有効なサンプル）から北・東への距離 [m] |
| Alt | 原点からの高さ [m]。下の括弧内は、元のログの原点（ArduPilot では EKF 原点）からの高さ |
| GndSpd / Course | 対地速度 [m/s] と進行方向（北＝0°、東＝90°。0.2 m/s 未満では表示しない） |
| VertSpd | 上昇速度 [m/s]（上がプラス） |
| Roll / Pitch / Yaw | 姿勢 [°]。ロールは右下がりが＋、ピッチは機首上げが＋、ヨーは北＝0°・東＝90° |
| AttErr | 目標姿勢とのずれ [°] |
| ARM TIP | アーム先端の北・東・高さ [m] |

速度は、リサンプリング後の位置の差から計算した値で、ログの速度そのものではない。表示が要らなければ、Show のチェックを外す。

### プロペラの回転・推力（FlightReplayPropellers）

ログのモーター出力（ArduPilot では RCOU の PWM）から、回転数と推力を **推定** して表示する。ログにあるのは「モーターへの指令」で、実測の回転数ではない。推定の精度は、下で入れる較正値で決まる。

1. ReplayOrigin に FlightReplayPropellers を付ける（新しい版の Create Replay Rig で作った場合は付いている。前の版で作った ReplayOrigin には、Add Component で FlightReplayPropellers と FlightReplayTelemetry を足す）。箱庭の DronePropeller は、DroneAvatarWeb を止めていれば動かないので、二重には回らない。
2. Props の欄が空か、空の行（Transform 未指定・Motor が 0）があれば、Play したときに Target の子から名前に `Pivot` を含む 4 つを探し、位置から ArduPilot のクアッドX の順に割り当てる（M1 右前・反時計回り、M2 左後・反時計回り、M3 左前・時計回り、M4 右後・時計回り）[要出典確認: ArduPilot モーター順序の公式図]。Play しなくても、コンポーネント右上の︙ → Auto-assign (ArduPilot Quad X) で割り当てられる。結果は Console に出る。
3. 較正値を入れる。**初期値は仮の値なので、必ず自分の測定値に置き換える。**

| 項目 | 内容 |
|---|---|
| Rpm Per Pwm_A, Rpm Offset_B | 回転数 rpm ＝ A × PWM ＋ B |
| Spin Min Pwm | これ未満の PWM は停止（0 rpm）とみなす |
| Thrust Model | `KtRpm2`：T＝kT·rpm²（kT の単位 N/rpm²）／`KtOmega2`：T＝kT·ω²（ω＝rpm·2π/60、単位 N/(rad/s)²）／`CtRhoN2D4`：T＝Ct·ρ·n²·D⁴（n＝rpm/60、Ct は無次元） |
| Kt / Ct / Air Density / Prop Diameter | 選んだ式で使う値 |
| Visual Speed Scale / Visual Max Rev Per Sec | 見た目の回転だけを遅くする（実際の回転数のままだと、画面のコマ数では止まって見えたり逆回転に見えたりするため）。画面の数値には影響しない |
| Show Thrust Arrows / Arrow Meters Per Newton | 推力を矢印で表示する（初期値 1 N で 5 cm） |

画面右上に、各モーターの PWM・推定回転数・推定推力と、推力の合計（N と kgf）が出る。値は Play 中に Inspector で書き換えても、すぐ反映される。

### 実際の景色を表示する（FlightReplayEnvironment）

ログの GPS から、リプレイ原点の緯度・経度・高度を JSON に書き出している（`[geo ]` の行。CSV では `--pos-frame geo` か `--geo-origin` を指定したとき）。この位置に実際の景色を読み込み、Inspector の **Mode** で切り替える（Play 中でも切り替えられる）。

| Mode | 中身 | 必要なもの |
|---|---|---|
| None | 景色なし（今のシーンのまま） | – |
| Cesium | Cesium for Unity で読み込む地形・航空写真・建物（Google Photorealistic 3D Tiles など） | インターネット接続、Cesium ion アカウント（無料枠あり） |
| GSI | 国土地理院の標高タイルと航空写真から作った地形（建物の立体はない） | 事前に `gsi_terrain.py` を 1 回実行（ネット接続が必要なのはこのときだけ） |

**Hide When Scenery On** には、景色を出しているときに隠したい床や障害物を入れる。

> **注意：カメラの入った親オブジェクトを入れない。** 入れたオブジェクトは丸ごと止まる（SetActive(false)）。箱庭のシーンでは `ENV` の中に Camera があるため、ENV を丸ごと入れると Game ビューに何も映らなくなり、追従カメラも止まる。ENV の中の Plane、Obstacle1、Obstacle2、AREA1〜6（必要なら Baggage）を個別に入れる。
>
> まとめて入れる方法：ReplayOrigin を選び、Inspector 右上の鍵アイコンで表示を固定する。Hierarchy で Ctrl を押しながら対象を選び、Hide When Scenery On の欄にドラッグする。

#### Cesium の準備（初回のみ）

手順は Cesium 公式の「Photorealistic 3D Tiles from Google Maps Platform in Cesium for Unity」に沿っている（2026 年 11 月時点）。

**1. Cesium for Unity をインストールする**

1. Edit → Project Settings → Package Manager を開き、Scoped Registry を追加して Save する。
   - Name：`Cesium`
   - URL：`https://unity.pkg.cesium.com`
   - Scope(s)：`com.cesium.unity`
2. Window → Package Manager を開き、Packages の切り替えで My Registries を選ぶ。Cesium for Unity を選んで Install する。

インストールで証明書（SSL）のエラーが出る場合は、ネットワークが GitHub からのダウンロードを止めている可能性がある（推測）。そのときは GitHub の CesiumGS/cesium-unity のリリースページから `com.cesium.unity-x.y.z.tgz` を落とし、Package Manager の ＋ → Install package from tarball で入れる。

**2. Cesium ion に接続する**

1. 上部メニューの Cesium → Cesium でパネルを開き、**Connect to Cesium ion** を押す。
2. ブラウザでアカウントにログインする（無料アカウントを作れる。GitHub や Google のアカウントも使える）。確認が出たら **Allow** を押して Unity に戻る。
3. トークンを作る：Cesium パネル上部の **Token** ボタンを押し、**Create a new token** を選んで **Create New Project Default Token** を押す。既存のトークンを使う場合は **Use an existing token** から選ぶ。

**3. Google の 3D タイルを追加する**

1. Cesium パネルの **Quick Add Cesium ion Assets** で、**Google Photorealistic 3D Tiles** の横のボタンを押す。Hierarchy に CesiumGeoreference と、その子の Cesium3DTileset ができ、地形が少しずつ読み込まれる。
2. Cesium3DTileset を選び、**Show Credits On Screen** をオンにする。Google Maps の利用規約を守るために必要で、データの出典が画面下に表示される。**公開する画像や動画でも、この表示を消さないこと。**

建物の立体が要らず軽くしたい場合は、Google Photorealistic 3D Tiles のかわりに、Quick Add から Cesium World Terrain と航空写真を追加してもよい。

**4. リプレイとつなぐ**

1. Hierarchy の **CesiumGeoreference** を、ReplayOrigin の FlightReplayEnvironment の **Cesium Georeference** 欄にドラッグする。
2. Mode を **Cesium** にして Play を押す。
3. Console に `[FlightReplay] Cesium origin set: lat ..., lon ..., height ... m` と `Ground snap: offset ... m` が出れば成功。

緯度・経度・高さは手で入力しなくてよい。公式の手順では CesiumGeoreference の Inspector に Latitude・Longitude・Height を入れるが、FlightReplayEnvironment が JSON の geo_origin から自動で設定する。別の場所に合わせたいときだけ、Override Geo Origin をオンにして値を入れる。

**5. カメラ（公式の手順と違う点）**

- 公式の手順にある「Main Camera を消して Dynamic Camera を追加する」は**行わない**。今のカメラと FlightReplayFollowCamera をそのまま使う（Dynamic Camera を足すとカメラが二重になる）。
- 遠くの景色まで映すため、使っているカメラの Clipping Planes の **Far** を 5000 程度に上げる。

**6. 読み込みと高さ**

- 景色はストリーミングで少しずつ細かくなる。最初の 20 秒（Snap Refine Seconds）は、1 秒ごとに高さを合わせ直す。
- 機体が浮く・埋まるときは、数秒待ってから FlightReplayEnvironment の︙ → **Snap to ground now** を選ぶか、Manual Height Offset で合わせる。
- Cesium の高さは楕円体高、ログの高度は海抜（推測）なので、数十 m ずれることがある。これは自動の高さ合わせで吸収される。ジオイド高が分かっていれば Geoid Height に入れてもよい。

#### GSI（国土地理院）の準備

インターネットにつながる PC で、変換済みの JSON から地形を作る。JSON には geo_origin が必要（最新の bin2replay.py で変換したもの。変換時に `[geo ]` の行が出る）。出力先は Unity の StreamingAssets の下にする。

```bash
cd C:\Users\student\Desktop\claude\unity\longarm-drone-replay
pip install pillow
python python/gsi_terrain.py C:/Users/student/drone/simulation/Assets/StreamingAssets/flight_replay.json --out-dir C:/Users/student/drone/simulation/Assets/StreamingAssets/gsi
```

成功すると、次の 3 行が出る。

```
[gsi ] 51x51 grid (5.0 m), area E -100..150 m, N -106..144 m
[gsi ] elevation dem5b_png z15: 147.8..155.0 m (origin alt 156.3 m)
[gsi ] photo seamlessphoto z17: 291x291 px -> .../StreamingAssets/gsi
```

| オプション | 既定値 | 意味 |
|---|---|---|
| `--margin` | 100 | 飛行範囲の外側に足す幅 [m] |
| `--grid` | 5 | 標高の格子の間隔 [m] |
| `--photo-zoom` | 17 | 航空写真の細かさ（17 で約 1 m/画素、18 で約 0.5 m/画素。上げるほど重い） |

- **標高データ**：5 m メッシュ（dem5a → dem5b の順）を探し、なければ 10 m メッシュ（dem_png）を使う。どれを使ったかは 2 行目に出る。
- **写真が粗いとき**：`--photo-zoom 18` を付けて実行し直す。ダウンロードしたタイルは `gsi/_tile_cache` に残るので、2 回目以降は速い（不要なら消してよい）。
- **高さのずれ**：2 行目の地面の標高と origin alt（ログ上の原点の高度）は、数 m ずれることがある。地図とログで高さの基準が違うため（推測）。Unity の自動の高さ合わせで吸収される。
- **出力**：`gsi_terrain.json`（標高）、`gsi_photo.jpg`（写真）、`ATTRIBUTION.txt`（出典）。Mode を GSI にすると読み込まれ、画面右下に出典が表示される。論文や公開資料に画像を載せるときは、国土地理院の利用規約に従って出典を書くこと。
- **確認**：Play したとき、Console に `GSI terrain loaded: 51x51, 5 m grid` と `Ground snap: offset ... m` が出れば成功。
- 引数なしで `python python/gsi_terrain.py` を実行すると、使い方が表示されるだけで何も作られない。

#### 高さ合わせ（自動＋手動）

地図の高さの基準（Cesium は楕円体高、地理院は標高）と、ログの高度の基準は一致しないことが多い。そのため、次の 2 つで合わせる。

| 項目 | 内容 |
|---|---|
| Auto Snap To Ground | オン（初期値）：最初のフレームの機体の真下で地面を探し、機体の中心が地面から Ground Clearance の高さになるよう、リプレイ全体を上下に動かす。Cesium はタイルが後から細かくなるので、Snap Refine Seconds（初期値 20 秒）の間、1 秒ごとに合わせ直す |
| Ground Clearance | 着地しているときの、地面から機体の中心までの高さ [m]（初期値 0.2） |
| Manual Height Offset | 手動の上下の調整 [m]。自動の結果に足される。Play 中に変えるとすぐ反映される |
| Geoid Height | （Cesium のみ）ジオイド高 [m]。分かれば入れる。分からなければ 0 のままで、自動の高さ合わせに任せる |

自動の高さ合わせをもう一度やりたいときは、コンポーネント右上の︙ → Snap to ground now を選ぶ。ログの最初のフレームで機体が地面にいない場合（空中からログが始まる場合）は、Auto Snap To Ground をオフにして、手動で合わせる。

### 色の意味

| 色 | 意味 |
|---|---|
| 緑 → 黄 → 赤 | 姿勢誤差（2°以下で緑、10°以上で赤）。アームがあればアームを、なければ機体を塗る |
| 灰 | ログが途切れている区間（`valid=0`）、または目標姿勢が記録されていない |
| 青い線 | 先端の軌跡（アームがなければ機体中心の軌跡）。直近 10 秒分 |

---

## B. 箱庭（hakoniwa-unity-drone）での使い方

前提：『ArduCopter のリファレンスマニュアル ver.2.2』1.6 節の手順で、simulation プロジェクトの WebAvatar シーンが動く状態になっていること。

1. A-1 と同じく、ファイルを置く。
2. Project ウィンドウで WebAvatar シーンを `Ctrl+D` で複製し、`ReplayScene` という名前にして開く（元のシーンは触らない）。
3. **箱庭が機体を動かす処理を止める。**
   - Hierarchy 上の検索欄に `t:DroneAvatarWeb` と入れ、出てきたオブジェクトの「Drone Avatar Web」のチェックを外す（終わったら検索欄を空に戻す）。
   - このスクリプトは、WebSocket で受け取った位置・姿勢を機体の Transform に書き込む。止めないと `Can not find pduManager...` の警告が出続けるうえ、通信がつながったときにリプレイと機体を取り合う（T4）。
   - `SERVER IS NOT ENABLED` などのメッセージも気になる場合は、`HakoniwaWeb` オブジェクトも無効にしてよい（リプレイには使わない）。
4. 機体の「いちばん外側の親」を選び、**Tools → FlightReplay → Create Replay Rig (selected object = vehicle)** を選ぶ。
5. 箱庭のシーンにすでにカメラ制御がある場合は、Main Camera に付いた FlightReplayFollowCamera を無効にする。地面がある場合は FlightReplayGrid も無効にしてよい。
6. Play を押して、A-4・A-5 と同じように確認する。

---

## トラブルシューティング

| No. | 症状 | 原因と対処 |
|---|---|---|
| T1 | `JSON not found` | StreamingAssets の綴り、または Json Path が違う。絶対パスを書いてもよい。JSON を Assets 内に置いて `Json Asset` 欄にドラッグしてもよい。 |
| T2 | 機首が横や後ろを向く | 自分の物体の前がローカル +Z ではない。Model Euler Offset を (0,90,0)・(0,-90,0)・(0,180,0) の順に試す。アームは記録どおりに描かれるので、物体の前がアームと同じ向きになる値が正解。仮の機体では起きない。 |
| T3 | 上下逆さま、またはピッチが逆 | 物体の上が +Y ではない（Blender から Z-up のまま取り込んだなど）。Model Euler Offset の X を ±90 にする。CSV の場合は、`--pos-frame` と `--att-frame` が合っているかも確認する（ROS なのに `ned` を指定しているなど）。 |
| T4 | ガタつく、または動かない | 別のスクリプト（箱庭の受信処理、Animator など）が同じ Transform を動かしている。物体のコンポーネントを 1 つずつ無効にして切り分ける。 |
| T5 | 灰色の区間が多い | ログに途切れがある（ArduPilot ではアームしていない時間）。`--skip-gaps` を付けて変換し直す。短い途切れなら `--max-gap` を大きくしてもよい（例：1.0）。途切れの中は直線で補間しているだけなので、図や評価には `valid=1` の区間だけを使う。 |
| T6 | `ATT message not found` | ログ設定（LOG_BITMASK）で姿勢が記録されていない。 |
| T7 | `No usable position message` | `--pos-source nkf1`、`pos` の順に試す。XKF1 があるのに失敗する場合は `--ekf-core 1` を試す。 |
| T8 | `column(s) not found` | CSV の列名が違う。`--dump-columns` の表示どおりに書く（大文字小文字・空白も含む）。区切りがタブなどで自動判定に失敗する場合は `--delimiter` を指定する。 |
| T9 | 機体がはるか遠くに出る | `--no-origin-shift` を付けている。外して変換し直せば、最初の有効なサンプルが (0,0,0) になる。 |
| T10 | 地面にめり込む、浮く | 記録の高度原点と地面の高さが違う。Height Offset で合わせる。 |
| T11 | 操作パネルが出ない | FlightReplayUI の Show がオフ、または Scene ビューを見ている。 |
| T12 | 右ドラッグやホイールでカメラが動かない | Game ビューをクリックしてフォーカスを当てる。それでも動かない場合は、FlightReplayFollowCamera の Distance・Elevation・Azimuth を Inspector で直接変える（推測：Unity の版によっては、IMGUI でのドラッグの受け取り方が異なる可能性がある）。 |
| T13 | 物体や格子がピンク色 | レンダーパイプライン（URP／HDRP）とマテリアルが合っていない。仮の機体の場合は、自分で作ったマテリアル付きの物体を Target に指定する。 |
| T14 | 姿勢誤差が常に `n/a` | 目標姿勢が記録されていない（.bin：ATT に DesRoll などがない／CSV：`--des-euler-cols` を指定していない）。 |
| T15 | メニューに Tools → FlightReplay が出ない | `Editor` フォルダがコピーされていない、またはコンパイルエラーがある。Assembly Definition を使っているプロジェクトでは、FlightReplay 用の asmdef を作り、Editor 側から参照させる。 |
| T17 | 先端の球が、モデルのアームより上下にずれる | アームの付け根の高さが、計算（既定は機体中心）とモデルで違う。ピッチが変わってもずれ幅が同じなら、付け根の高さの違い。FlightReplayPlayer の Arm Mount Reference にモデルの付け根（ArmBase など）をドラッグする。CSV の先端位置も合わせたいときは、`--arm-mount` を指定して変換し直す。ピッチに応じてずれが増減する場合は、モデルのアームが傾いて付いている（ArmBase の Rotation X が 0 でない）可能性がある。 |
| T18 | 操作パネルが見えない、はみ出す | 最新版ではパネルを自動で縮小する。それでも見えない場合は、H キーで隠れていないか、Anchor（Top／Bottom）を切り替えて箱庭の画面表示と重なっていないかを確認する。 |
| T19 | プロペラが回らない、または二重に回る | Console の `Propellers auto-assigned` を確認する。`found N objects` と出たら、Props に手で割り当てる。二重に回る場合は、箱庭側のプロペラを回すスクリプトを無効にする。PWM が Spin Min Pwm 未満のときは止まる。 |
| T20 | Cesium の景色が出ない、場所が違う | Console の `Cesium origin set` を確認する。`no geo origin` と出たら、最新の bin2replay.py で変換し直す（`[geo ]` の行が出る）。Cesium ion への接続とトークン、Cesium Georeference 欄の割り当ても確認する。 |
| T21 | GSI の地形が出ない | Console の `not found` を確認する。`gsi_terrain.py` の出力先が `Assets/StreamingAssets/gsi` で、Gsi Folder が `gsi` になっているか確認する。 |
| T22 | 機体が地面に埋まる、または浮く | Console の `Ground snap` を確認する。`no ground found` の場合、Cesium のタイルがまだ読み込まれていないか、当たり判定がない。少し待ってから Snap to ground now を選ぶか、Manual Height Offset で合わせる。 |
| T23 | Mode を Cesium／GSI にしたら、Game ビューに何も映らない | Hide When Scenery On に、カメラの入ったオブジェクト（箱庭の ENV など）が入っている。ENV を外し、床や障害物だけを個別に入れる。 |
| T24 | Cesium for Unity のインストールで証明書（SSL）のエラーが出る | ネットワークが GitHub からのダウンロードを止めている可能性がある。GitHub のリリースページから `.tgz` を落とし、Package Manager の Install package from tarball で入れる。 |
| T25 | Cesium の景色にクレジットが出ない | Cesium3DTileset の Show Credits On Screen をオンにする（Google のデータでは必須）。 |
| T16 | `FindObjectOfType is obsolete` の警告 | Unity 2023 以降で出る警告で、動作には影響しない。 |
