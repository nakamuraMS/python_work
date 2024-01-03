# 配布データと応募用ファイル作成方法の説明

本コンペティションで配布されるデータと応募用ファイルの作成方法や投稿する際の注意点などについて説明する.

1. [配布データ](#配布データ)
1. [応募用ファイルの作成方法](#応募用ファイルの作成方法)
1. [コンペティションサイトで取得できる対戦ログデータ](#コンペティションサイトで取得できる対戦ログデータ)
1. [投稿時の注意点](#投稿時の注意点)

## 配布データ

配布されるデータは以下の通り.

- [Readme](#readme)
- [説明資料](#説明資料)
- [シミュレータ](#シミュレータ)
- [応募用サンプルファイル](#応募用サンプルファイル)

### Readme

本体は`readme.zip`. 解凍すると以下のようなディレクトリが生成される.

```bash
readme
├─ readme.md
├─ run_test.gif
└─ vscode_test.gif
```

`readme.md`はこのファイルで, 配布用データの説明と応募用ファイルの作成方法を説明したドキュメント. マークダウン形式で, プレビューモードで見ることを推奨する.

### 説明資料

説明資料は`docs.zip`で, 解凍すると以下のようなディレクトリ構造のデータが作成される.

```bash
docs
├─ 基準シミュレータ 取扱説明書.pdf
├─ 基準モデルの簡易説明.pdf
└─ 基準モデル及びパラメータに関する資料.pdf
```

内容は以下の通り.

#### 問題設定

本コンペティションにおける戦闘場面や勝敗に関するルールなどについては`基準モデル及びパラメータに関する資料.pdf`や`基準モデルの簡易説明.pdf`で説明されている.

#### 配布シミュレータの取扱説明書

本コンペティションで使用するシミュレータの導入方法や仕様, 一般的な行動判断の実装方法などの詳細については`基準シミュレータ 取扱説明書.pdf`を参照.

#### 初期行動判断モデル

シミュレータにデフォルトで実装されている初期行動判断モデルの説明については`基準モデル及びパラメータに関する資料.pdf`や`基準モデルの簡易説明.pdf`を参照. なお, 本モデルは本コンペティションにおいてベンチマークとして参戦する.

### シミュレータ

空戦を再現するシミュレータ. `simulator_dist.zip`が本体で, 解凍すると, 以下のようなディレクトリ構造のデータが作成される.

```bash
simulator_dist
├── Agent                             : 投稿用の行動判断モジュール
├── BenchMark                         : 初期行動判断モデルモジュール
├── common                            : コンペティションで定義されている戦闘場面などの設定ファイル
├── root
│   ├── addons                        : 本シミュレータの追加機能として実装されたサブモジュール
│   │   ├── AgentIsolation            : 行動判断モデルをシミュレータ本体と隔離して動作させるための評価用機能
│   │   ├── HandyRLUtility            : HandyRLを用いて学習を行うための拡張機能
│   │   ├── MatchMaker                : 複数の行動判断モデルを用いてSelf-Play等の対戦管理を行うための拡張機能
│   │   ├── rayUtility                : ray RLlibを用いて学習を行うための拡張機能
│   │   └── torch_truncnorm           : PyTorchで切断ガウス分布を用いるための拡張機能
│   ├── ASRCAISim1                    : 最終的にPythonモジュールとしての本シミュレータが格納されるディレクトリ
│   ├── include                       : コア部を構成するヘッダファイル
│   ├── sample                        : 戦闘環境を定義し学習を行うためのサンプル
│   │   ├── modules
│   │   │   └── OriginalModelSample   : 独自のAgentクラスと報酬クラスを定義するためのサンプル
│   │   └── scripts
│   │       └── R5Contest
│   │           ├── HandyRLSample     : HandyRLを用いて基本的な強化学習を行うためのサンプル
│   │           ├── MinimumEvaluation : 各行動判断モデルを対戦させる最小限の評価環境
│   │           └── raySample         : rayを用いて基本的な強化学習を行うためのサンプル
│   ├── src                           : コア部を構成するソースファイル
│   └── thirdParty                    : 外部ライブラリの格納場所(同梱は改変を加えたもののみ)
│　     └── include
│           └── pybind11_json　       ※オリジナルを改変したものを同梱
├── src                               : 動作確認を行うためのプログラム
├── build_deps.sh                     : C++依存パッケージなどを導入するスクリプト
├── build_sim.sh                      : シミュレータを導入するスクリプト
├── docker-compose.yml                : 仮想環境を構築するための定義ファイル
├── Dockerfile                        : Dockerによる環境構築に必要なDockerfile
├── Dockerfile.cpu_base               : Dockerによる環境構築に使われるベースイメージ(CPU環境)
├── Dockerfile.gpu_base               : Dockerによる環境構築に使われるベースイメージ(GPU環境)
├── make_agent.ipynb                  : エージェントを作成するための簡単な手順を示したチュートリアルノートブック
├── replay.py                         : 戦闘シーンを動画として作成するプログラム
└── validate.py                       : AgentとBenchMarkの対戦を実行するプログラム
```

### 応募用サンプルファイル

応募用のサンプルファイル. 実体は`sample_submit.zip`で, 解凍すると以下のようなディレクトリ構造のデータが作成される.

```bash
sample_submit
├── __init__.py
├── args.json
└── config.json
```

詳細や作成方法については[応募用ファイルの作成方法](#応募用ファイルの作成方法)を参照すること.

## 応募用ファイルの作成方法

応募用ファイルは学習済みモデルなどを含めた, 行動判断を実行するためのソースコード一式をzipファイルでまとめたものとする.

### 環境構築

まずは実行テストなどを行う準備として, 環境構築を行っておく. 主にローカル環境においてソースからビルドする方法とDockerでコンテナ型仮想環境を構築する方法がある. 評価システムで運用されている環境を再現する意味ではDockerにより構築する方が望ましい.

#### ソースからビルドする方法

取扱説明書の`1.2 環境の構築`にある通りにC++依存パッケージなどを導入する. 推奨OSは`Ubuntu 20.04`, または`Ubuntu 22.04`でAnacondaなどのPython環境を構築している前提で, `torch==1.13.1`と`tensorflow==2.13.0`がインストールされているとする. これらをまずは事前に導入しておくこと. WindowsOSの場合は, `wsl`を導入したうえで, Ubuntu環境を構築しておくとよい. 導入方法については[こちら](https://learn.microsoft.com/ja-jp/windows/wsl/install)を参照されたい. Anacondaを導入する場合は[ここ](https://www.anaconda.com/download)より, 自身のOSから判断してインストーラをダウンロードしてインストールできる(Pythonの推奨バージョンは3.8). `torch`や`tensorflow`は`pip`などによりインストール可能(インストール方法について, `torch`は[ここ](https://pytorch.org/get-started/previous-versions/), `tensorflow`は[ここ](https://www.tensorflow.org/install)を参照.).

C++依存パッケージなどを導入するためのスクリプト`build_deps.sh`とシミュレータを導入するためのスクリプト`build_sim.sh`を用意しているので, 以下のコマンドを実行して, 実際に構築されたい.

```bash
$ cd /path/to/simulator_dist
$ sudo bash build_deps.sh
$ bash build_sim.sh
...
```

エラーなく実行が終わるとシミュレータの実行環境が構築される.

#### Dockerで構築する方法

Dockerで環境構築する場合, Docker Engineなど, Dockerを使用するために必要なものがない場合はまずはそれらを導入しておく. [Docker Desktop](https://docs.docker.com/get-docker/)を導入すると必要なものがすべてそろうので, 自身の環境に合わせてインストーラをダウンロードして導入しておくことが望ましい. 現状, Linux, Mac, Windowsに対応している. デフォルトでは, GPU上での実行を前提とした環境(ベースイメージの元となっているDockerfileは`Dockerfile.gpu_base`)となっている. GPU環境を持っていないならば, CPU上での実行を前提とした環境(ベースイメージの元となっているDockerfileは`Dockerfile.cpu_base`)を構築する. 具体的には, `Dockerfile`の最初の1行を以下のように編集する. なお, 評価システムではCPU上での実行となる.

```bash
FROM signate/runtime-cpu:dl_env
```

そして, `/path/to/simulator_dist`に同封してある`docker-compose.yml`で定義されたコンテナを, 以下のコマンドを実行することで立ち上げる.

```bash
$ cd /path/to/simulator_dist
$ docker compose up -d
...
```

`docker-compose.yml`は好きに編集するなりして, 自身が使いやすいように改造してもよい. 無事にコンテナが走ったら, 必要なデータなどをコンテナへコピーする.

```bash
$ docker cp /path/to/some/file/or/dir {コンテナ名}: {コンテナ側パス}
... 
```

そして, 以下のコマンドでコンテナの中に入り, 分析や開発を行う.

```bash
$ docker exec -it --user root {コンテナ名} bash    # デフォルトでsignateという名前のユーザー名が追加されているが, そのユーザーでログインしたい場合は`--user root`をはずす 
...
```

`コンテナ名`には`docker-compose.yml`の`services`->`dev1`->`container_name`に記載の値を記述する. `/path/to/simulator_dist`をコンテナ側の`/workspace`へバインドマウントした状態(`/path/to/simulator_dist`でファイルの編集などをしたらコンテナ側にも反映される. 逆もしかり.)で, `/workspace`からスタートする. さらにGUI機能も導入されているため, GUIでシミュレータを動作可能(後述).

##### CUDA環境の利用

ホスト側にCUDA環境に対応したGPUがある場合, CUDA環境に対応したイメージからコンテナを構築可能. 実際にGPUがコンテナ内で有効になっているかどうかは以下のコマンドで確認できる.

```bash
# コンテナに入った後
$ python -c "import torch; print(torch.cuda.is_available())"
True
$ python -c "import tensorflow as tf; print(tf.config.list_physical_devices('GPU'))"
...{GPUデバイスのリスト}...
```

#### GUI機能の利用

[VSCode](https://code.visualstudio.com/)を導入済みであれば, VSCodeから(コンテナに接続などして)実際に分析や開発などを行うことが可能. また, [ソースからビルドした場合](#ソースからビルドする方法)でも[Dockerから構築した場合](#dockerで構築する方法)でも, GUIとしては`lxde`と, VNC(Virtual Network Computing)とnoVNCを導入した状態となる. よって, HTTPサーバーで公開すれば, クライアント側はブラウザでアクセスするだけで原理的にはリモートデスクトップ環境を使えるようになる. Dockerから構築した場合は`jupyter lab`も利用可能となっている.

##### VSCodeの利用

例えばWindowsOSの場合は[Remote-WSL](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-wsl)を導入しておくと, VSCodeからlinux環境(wsl)にアクセスすることができる. 導入後, コマンドパレットで`wsl`と打つと一覧に`WSL: New Window`などとでるので, それを選択することで機能が使える. また, [VSCode](https://code.visualstudio.com/)には立ち上げたコンテナに接続して開発を行える[拡張機能](https://code.visualstudio.com/docs/devcontainers/containers)がある.  まずVSCode [設定]→[拡張機能]→[Dev Container]:インストールする. その後コマンドパレットで`dev..`と打つと一覧に`Dev Containers: Attach to Running Container`が出るのでそれを選択して, 立ち上げたコンテナ名を選択すると, 別ウィンドウでワークスペースが立ち上がる. Pythonやjupyterの拡張機能をインストールし, Pythonインタープリタとjupyterのカーネルはベースイメージのものを選択すると作業環境ができる. これらを導入することで, VSCodeの高度なインテリセンス機能などを活用できる.

##### VNCの利用

ローカル環境で実行する場合, すでにGUI環境があるならば以下の操作は必要ない.

```bash
$ USER={ユーザー名} vncserver :1 -geometry 1500x700 -depth 24
...
```

一旦立ち上がったがすぐkillされた場合は以下のコマンドで再度サーバーを立ち上げてみる.

```bash
$ tigervncserver -xstartup /usr/bin/startlxde -geometry 1500x700 -depth 24
...
```

ログインユーザーとして, ディスプレイ:1に解像度1500x700, 24bit colorのVNCデスクトップを立ち上げている. 解像度と色は好みで設定する. パスワード入力が求められるので, 6-8文字の簡単なパスワードを設定する.

起動しているかどうかは以下のコマンドで確認できる.

```bash
$ vncserver -list
...
```

削除するときは`vncserver -kill :1`を実行すればよい. ログは`/root/.vnc/*.log`に出力されているので, `tail -F /root/.vnc/*.log`で監視できる.

次にnoVNCを設定したポートに公開する.

```bash
websockify -D --web=/usr/share/novnc/ {ポート番号} localhost:5901
```

`ポート番号`にはVNCサーバー側のポート番号(Dockerで構築した場合は`docker-compose.yml`で定義したコンテナ側のポート番号(`ports`の`:`の右側の値))を指定する. VNCでは"5900+ディスプレイ番号"のポートにサーバーが立つので, ここでは`localhost:5901`を指定している.

以上の設定のあと, ホスト側でブラウザを立ち上げて`localhost:{ホスト側のポート番号}/vnc.html`にアクセスすると, noVNCの画面が立ち上がってGUIデスクトップが表示される. 画面左のメニューで各種設定や, クリップボードの確認・編集も可能.

##### jupyter labの利用

コンテナには`jupyter lab`がインストールされている. 以下のコマンドで`jupyter lab`を立ち上げることができる.

```bash
# コンテナに入った後
$ jupyter lab --port={ポート番号} --ip=0.0.0.0 --allow-root
...
copy and paste one of these URLs:
        ...
     or ...

```

`ポート番号`には`docker-compose.yml`で定義したコンテナ側のポート番号(`ports`の`:`の右側の値)を指定する. 表示されるURLをブラウザに張り付ければjupyterの環境で作業が可能となる. Desktopから直接入ってもよい(最初にtokenを聞かれるので, URLの`token=`に続く文字列を入力する).

#### 動作確認プログラムの実行

取扱説明書にある通り, 以下のコマンドにより実際に動画付きでシミュレータが動くかどうかを確認する. 成功すると動画が再生されて勝敗結果等が出力される.

```bash
$ cd /path/to/simulator_dist/root/sample/scripts/R5Contest/MinimumEvaluation
$ python evaluator.py "Rule-Fixed" "Rule-Fixed" -n 1 -v -l "./results"
...
```

VSCodeからコンテナに接続した際にターミナルを開いて実行したときは以下のように動画付きで結果が出力される.

![vscode_test](./vscode_test.gif)

以下はnovncによりリモートデスクトップで実行したときの実行画面

![run_test](./run_test.gif)

### ディレクトリ構造

提出すべき行動判断を実行するためのソースコード一式は以下のようなディレクトリ構造となっていることを想定している.

```bash
.
├── __init__.py              必須: 実行時に最初に呼ばれるinitファイル
├── args.json                必須: アルゴリズムに渡す引数に関する情報をまとめたjson形式のファイル
└── ...                      任意: 追加で必要なファイルやディレクトリ
```

- エージェントに対する設定に関しては例えば"config.json"などの名前でファイルとして保存しておいて, 実行時に読み込んで使用する想定である.
- `__init__.py`で定義されたメソッドに渡す情報は`args.json`を想定している.
  - 名前は必ず`args.json`とすること.
- 学習済みモデルなど, 実行に必要なその他ファイルも全て含まれている想定(必要に応じてディレクトリを作成してもよい).

### `__init__.py`の実装方法

以下のメソッドを実装すること. `基準シミュレータ 取扱説明書.pdf`の7も参照.

#### getUserAgentClass

Agentクラスオブジェクトを返すメソッド. 以下の条件を満たす.

- 引数argsを指定する.
  - `args.json`の内容が渡される想定である.
- Agentクラスオブジェクトを返す.
  - Agentクラスオブジェクトは自作してもよいし, もともと実装されているものを直接importして用いてもよい. Agentクラスの詳細は`基準シミュレータ 取扱説明書.pdf`の4.5や5.1などを参照. また, 実装例については`make_agent.ipynb`や`/path/to/root/sample/modules/OriginalModelSample/`以下のファイルを参照すること.

#### getUserAgentModelConfig

Agentモデル登録用にmodelConfigを表すjsonを返すメソッド. 以下の条件を満たす.

- 引数argsを指定する.
  - `args.json`の内容が渡される想定である.
- `modelConfig`を返す. なお, `modelConfig`とは, Agentクラスのコンストラクタに与えられる二つのjson(dict)のうちの一つであり、設定ファイルにおいて

    ```json
    {
        "Factory":{
            "Agent":{
                "modelName":{
                    "class":"className",
                    "config":{...}
                }
            }
        }
    }
    ```

    の`config`の部分に記載される`{...}`の`dict`が該当する. `基準シミュレータ 取扱説明書.pdf`の4.1なども参照すること.  

#### isUserAgentSingleAsset

Agentの種類(一つのAgentインスタンスで1機を操作するのか, 陣営全体を操作するのか)を返すメソッド. 以下の条件をみたす.

- 引数argsを指定する.
  - `args.json`の内容が渡される想定である.
- 1機だけならば`True`, 陣営全体ならば`False`を返す想定である.

#### getUserPolicy

`StandalonePolicy`を返すメソッド. `StandalonePolicy`については, `基準シミュレータ 取扱説明書.pdf`の2.6などを参照すること. 以下の条件を満たす.

- 引数argsを指定する.
  - `args.json`の内容が渡される想定である.
- `StandalonePolicy`を返す.

以下は`__init__.py`の実装例.

```Python
import os,json
import ASRCAISim1
from ASRCAISim1.policy import StandalonePolicy


def getUserAgentClass(args=None):
    from ASRCAISim1 import R4InitialFighterAgent01
    return R4InitialFighterAgent01


def getUserAgentModelConfig(args=None):
    configs=json.load(open(os.path.join(os.path.dirname(__file__),"config.json"),"r"))
    modelType="Fixed"
    if(args is not None):
        modelType=args.get("type",modelType)
    return configs.get(modelType,"Fixed")


def isUserAgentSingleAsset(args=None):
    #1機だけならばTrue,陣営全体ならばFalseを返すこと。
    return True


class DummyPolicy(StandalonePolicy):
    """
    actionを全く参照しない場合、適当にサンプルしても良いし、Noneを与えても良い。
    """
    def step(self,observation,reward,done,info,agentFullName,observation_space,action_space):
        return None

def getUserPolicy(args=None):
    return DummyPolicy()
```

応募用サンプルファイル`sample_submit.zip`も参照すること. また, `/path/to/root/sample/scripts/R5Contest/MinimumEvaluation`以下にある`HandyRLSample01S`や`RaySample01S`などのサンプルモジュールも参考にされたい. なお, `HandyRL`と`rayRLlib`についてはサンプルで使用しているNNクラスに対応した`StandalonePolicy`の派生クラスが提供されている(`/path/to/root/addons/HandyRLUtility`, `/path/to/root/addons/rayUtility/extension`以下を参照.).

### 実行テスト

モデル学習などを行い, 行動判断を行うプログラムが実装できたら, 初期行動判断モデル(BenchMark)との対戦を実行する.

```bash
$ python validate.py  --agent-id agent_id --agent-module-path /path/to/Agent --result-dir /path/to/results
...
```

- `--agent-id`には好きな自分のエージェント名を指定する. デフォルトは`0`.
- `--agent-module-path`には実装したプログラム(`__init__.py`など)が存在するパス名を指定する. デフォルトは`./Agent`
- `--result-dir`には対戦結果ログを格納するディレクトリを指定する. デフォルトは`./results`

実行に成功すると対戦が行われ, `{result_dir}/{agent_id}`以下に対戦結果ログ等が格納される.

- `validation_results.json`が対戦結果ログとして保存される. 対戦時の終了時間や勝利数やスコアなどの情報が含まれる.
- `--movie`を`1`にすると対戦動画情報(`.dat`ファイル)が保存される. デフォルトは`0`.
  - さらに`--visualize`を`1`にすると実行中に動画が再生されるが, 実行に時間がかかる. デフォルトは`0`.
- `--num-validation`に指定した値の回数だけ対戦が行われる. デフォルトは`3`.
- `--time-out`に指定した値(単位は[秒])までに行動を返さなかった場合は行動空間からランダムサンプリングされる. デフォルトは`0.5`.
- `--memory-limit`に指定した値(単位は[GB])で対戦実行時の使用メモリの上限を設定する. 超えた場合はエラーとなり, 対戦は無効となる. デフォルトは`3.5`.
- `--random-test`を`1`にするとシミュレーションを行わずランダムに勝敗を付ける. デフォルトは`0`であるが, デバッグをしたいときなどに`1`にする.
- `--control-init`を`1`にするとお互いの戦闘機の初期配置(位置と初速度)を点対称でランダムに決定する. デフォルトは`1`で, こちらがコンペティションにおける設定. `/path/to/simulator_dist/commom/initial_states.json`で設定された範囲でランダムに決まる(`pos`が位置, `vel`が速度, `heading`が向き).
  - 初期配置に関する情報は`{result_dir}/{agent_id}`以下に`config_{fight_n}.json`として保存される(`AssetConfigDispatcher`->`{Blue,Red}InitialState`->`elements`の`value`->`instanceConfig`).
- `--make-log`を`1`にすると対戦実行時に各ステップで返される自分の陣営における`observations`, `infos`とエージェントが返す`actions`のログを記録する. デフォルトは`1`でこちらがコンペティションにおける設定.
  - `{result_dir}/{agent_id}`以下に`log_{fight_n}.json`として保存される.
- `--max-size`に指定した値(単位は[GB])で対戦実行時のログデータがメモリを占有する大きさの上限を決める. 上限を超えると`observations`, `infos`, `actions`のログは記録として残らない. デフォルトは`0.3`.
  - 実行後標準出力で`Log size for fight {n}: ...[GB]`などと表示されるので, サイズを確認できる.
  - 上限を超えた場合は`log_{fight_n}.json`は保存されない.
- `--color`に"Red"または"Blue"を指定することで自分の陣営の色を指定する. "Red"を指定した場合は西軍, "Blue"を指定した場合は東軍となる. デフォルトは"Red".

`validation_results.json`のフォーマットは以下の通り.

```json
{
 "own": "Red",
 "details": {
  "1": {
   "finishedTime": 174.0,
   "numAlives": {
    "Blue": 1.0,
    "Red": 0.0
   },
   "endReason": "ELIMINATION",
   "error": false,
   "scores": {
    "Blue": 2.0,
    "Red": 1.0
   }
  },
  ...
 }
}
```

`own`は自分の陣営の色, `details`には各戦闘の`finishedTime`(シミュレーション内の終了時刻), `numAlives`(生き残った戦闘機の数), `endReason`(終了理由), `error`(プログラム的なエラーを起こしたか否か), `scores`(各陣営のスコア).

`log_{n}.json`のフォーマットは以下の通り.

```json
{
 "observations":
 [
  {
    "ptime":0.1,
    "data":
    {
      "1":...,
      "2":...
    }
  },
  ...
 ],
 "actions":
 [
  {
    "ptime":0.1,
    "data":
    {
      "1":...,
      "2":...
    }
  },
  ...
 ],
 "infos":
 [
  {
    "ptime":0.1,
    "data":
    {
      "1":...,
      "2":...
    }
  },
  ...
 ]
}
```

`observations`は前処理後の観測値, `actions`は取った行動, `infos`はスコアなどの情報が各ステップごとに格納されている. `ptime`がシミュレーション内の時刻で`data`が実際のデータ. なお, `ptime`の間隔は`modelConfig`で設定した設定値によって変わる.

`config_{n}.json`のフォーマットは以下の通り.

```json
{
 "AgentConfigDispatcher": {
  ...
 },
 "AssetConfigDispatcher": {
  "BlueInitialState": {
   "elements": [
    {
     "type": "direct",
     "value": {
      "instanceConfig": {
       "heading": 255.15947390144268,
       "pos": [
        1113.8725822326887,
        23039.4034502106,
        -9701.622629402118
       ],
       "vel": 258.9497408941761
      }
     }
    },
    {
     "type": "direct",
     "value": {
      "instanceConfig": {
       "heading": 138.35047754432557,
       "pos": [
        6912.059737051884,
        20922.273928319133,
        -11553.278206447263
       ],
       "vel": 274.263630148469
      }
     }
    }
   ],
   "order": "fixed",
   "type": "group"
  },
  "Fighter": {
   "model": "R5ContestFighter",
   "type": "PhysicalAsset"
  },
  "Fighters": {
   "elements": [
    {
     "alias": "Fighter",
     "type": "alias"
    },
    {
     "alias": "Fighter",
     "type": "alias"
    }
   ],
   "order": "fixed",
   "type": "group"
  },
  "RedInitialState": {
   "elements": [
    {
     "type": "direct",
     "value": {
      "instanceConfig": {
       "heading": 75.15947390144265,
       "pos": [
        -1113.8725822326887,
        -23039.4034502106,
        -9701.622629402118
       ],
       "vel": 258.9497408941761
      }
     }
    },
    {
     "type": "direct",
     "value": {
      "instanceConfig": {
       "heading": 318.3504775443256,
       "pos": [
        -6912.059737051884,
        -20922.273928319133,
        -11553.278206447263
       ],
       "vel": 274.263630148469
      }
     }
    }
   ],
   "order": "fixed",
   "type": "group"
  }
 },
 "Assets": {
  ...
 },
 "CommunicationBuffers":{
  ...
 },
 "Loggers": {},
 "Rewards": [],
 "Ruler": "R5ContestRuler",
 "TimeStep": {
  "baseTimeStep": 0.05,
  "defaultAgentStepInterval": 20
 },
 "ViewerType": "None",
 "seed": 680022080
}
```

`AssetConfigDispatcher`->`{Blue,Red}InitialState`->`elements`の`value`->`instanceConfig`で初期配置に関する情報を確認できる.

対戦動画情報を保存した場合, 以下のコマンドによりmp4形式の対戦動画を作成できる(存在する`.dat`ファイル全てに対して作成される).

```bash
$ python replay.py
...
```

GUI機能がうまく導入できていない場合はヘッドレスで実行する.

```bash
$ xvfb-run python replay.py
...
```

なお, BenchMark以外と対戦したい場合は別のエージェントを作成してそのモジュールディレクトリを`--benchmark-module-path`に指定して実行すればよい.

### 応募用ファイルの作成

上記の[ディレクトリ構造](#ディレクトリ構造)となっていることを確認して, zipファイルとして圧縮する.

```bash
$ zip -r submit ./Agent
...
```

実行後, 作業ディレクトリにおいて`submit.zip`が作成される.

## コンペティションサイトで取得できる対戦ログデータ

コンペティションサイトに投稿したアルゴリズムの対戦ログデータをサイトで取得することができる. 1対戦のログデータ(`detail`->`logs`)のメモリを占有する大きさの上限は0.3[GB]で, 超えた場合は対戦実行時に各ステップで返される自分の陣営における`observations`, `infos`とエージェントが返す`actions`のログは残らない(対戦自体は有効)が, 勝敗結果や最終スコアなどの集約した情報は残る. フォーマットは以下の通り.

```json
{
 "own": "Red",
 "result": "Lose",
 "detail": {
  "finishedTime": 300.0,
  "numAlives": {
   "Blue": 2.0,
   "Red": 1.0
  },
  "endReason": "TIMEUP",
  "error": false,
  "logs": {
   "observations": [...],
   "actions": [...],
   "infos": [...]
  },
  "scores": {
   "Blue": 0.5748469871489575,
   "Red": 0.0
  }
 }
}
```

サイトでは1日の対戦分を取得できる(検証時の対戦は含まない). 対戦が終了するとメールで通知され, その中に対戦ログを取得できるURLが記載されるので, 過去の対戦分はそこで参照可能となる. なお, 投稿後検証処理時に初期行動判断モデルとの対戦が行われるが, そちらの対戦ログ(`logs`の内容)は含まれないので, 情報が欲しい場合はローカル環境で実行して確認すること.

## 投稿時の注意点

投稿する前に自身のローカル環境で実行テストを行い, エラーなく実行できるか確認すること. 投稿時にエラーが出た場合, 以下のことについて確認してみる.

- 提出するプログラム内でインターネット接続を行うような処理を含めていないか. 評価システム上でインターネット接続はできない.
- 実行時間がかかりすぎていないか. 時間がかかりすぎるとランタイムエラーとなることがある. 使用メモリなども見直すこと.
- コンペティションサイトで対戦ログデータをダウンロードして確認したい場合, 上限があるので, ローカル環境でテストして上限を超えないか確認しておくこと.

## 更新情報

- [2023/12/20]
  - Python版のクラスを使用する際のメモリリークを解消した. エピソード終了時に該当クラスのインスタンスが解放されなくなっていたため, Factoryへのクラス登録方法を修正. 修正ファイルは以下の通り.
    - `simulator_dist/root/ASRCAISim1/common.py L6`
    - `simulator_dist/root/include/Factory.h L23`
    - `simulator_dist/root/src/Factory.cpp L23-29,203`
    - `simulator_dist/root/version.txt L1 (バージョン番号を1.4.0→1.4.1に)`
- [2023/12/11]
  - MWSの探知条件に関して射撃前や消失後でもミサイルが探知できてしまうバグを修正した. 修正ファイルは以下の通り.
    - `simulator_dist/root/src/Sensor.cpp L166`
- [2023/12/06]
  - シミュレーション実行時にobservationやactionが`numpy.ndarray`の場合でもログが保存されるように修正した. 修正ファイルは以下の通り.
    - `simulator_dist/src/fight.py`
  - サンプルで与えられているエージェントモデルを修正した. 修正以前は行動空間で射撃間隔を制御するが射程制限は行わない設定とした場合のみエラーが発生していた. 更新を反映するにはそのままビルドしなおしてもよいが, 通常モジュールがインストールされるディレクトリ(`site-packages`の`OriginalModelSample`)以下の該当ファイルをそのまま上書きしてしてもよい. 修正ファイルは以下の通り.
    - `simulator_dist/root/sample/modules/OriginalModelSample/OriginalModelSample/R5PyAgentSample01S.py L842`
    - `simulator_dist/root/sample/modules/OriginalModelSample/OriginalModelSample/R5PyAgentSample01M.py L673`
  - ray RLLibのサンプルプログラムで, 存在しないインポート部分を削除した. 修正ファイルは以下の通り.
    - `simulator_dist/root/sample/scripts/R5Contest/raySample/ImitationSample.py L23`
    - `simulator_dist/root/sample/scripts/R5Contest/raySample/LearningSample.py L23,26,48,51,55`
  - HandyRLのサンプルプログラムで学習中の重みを初期化する際のバグを修正した. 修正ファイルは以下の通り.
    - `simulator_dist/root/sample/scripts/R5Contest/HandyRLSample/handyrl/train.py L426,429-432,484-493`
  - TwoCombatMatchMakerで学習中の重みを初期化する際のバグ修正した. 更新を反映するにはそのままビルドしなおしてもよいが, 通常モジュールがインストールされるディレクトリ(`site-packages`の`ASRCAISim1`)以下の該当ファイルをそのまま上書きしてしてもよい. 修正ファイルは以下の通り.
    - `simulator_dist/root/addons/MatchMaker/TwoTeamCombatMatchMaker.py L358`
  - README.mdの誤記を修正した. 修正ファイルは以下の通り.
    - `simulator_dist/root/README.md L43,113`
