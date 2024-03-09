# 配布データと応募用ファイル作成方法の説明

本コンペティションで配布されるデータと応募用ファイルの作成方法について説明する.

1. [配布データ](#配布データ)
2. [応募用ファイルの作成方法](#応募用ファイルの作成方法)
<br>
<br>

## 配布データ

配布されるデータは以下の通り.

- [readme](#readme)
- [学習用データ](#学習用データ)
- [分析用追加配布プログラム](#分析用追加配布プログラム)
- [動作確認用のプログラム](#動作確認用のプログラム)
- [応募用サンプルファイル](#応募用サンプルファイル)
<br>
<br>

### readme

本ファイルである"readme.md"のことで, 配布用データの説明と応募用ファイルの作成方法を説明したドキュメント. マークダウン形式で, プレビューモードで見ることを推奨する.
<br>
<br>

### 学習用データ

以下のデータが学習用データとなる.

```bash
├─ train.csv
├─ search_specified.csv
├─ search_unspecified.csv
├─ road_all.csv
├─ road_local.csv
└─ search_raw_log.zip
```
内容は以下の通り.

- [道路構造データ](#道路構造データ)
- [トラフィックカウンターデータ](#トラフィックカウンタートラカンデータ)
- [ドラぷら検索ログ二次データ](#ドラぷら検索ログ二次データ)
  - [ドラぷら検索ログ一次データ](#ドラぷら検索ログ一次データ)
<br>
<br>

#### 道路構造データ

- road_local.csv
  - 関越道・東北道における各区間の始点、終点や⻑さ、距離などの属性データ. サイズ:12Kb

|  No |  column           |  型      |  説明   |  値例  |
| :---|  :---             |  :---:   |  :---  |  :---  |
|  1  |  start_name       | str      |  区間の始点ICの名称  |  所沢  |
|  2  |  end_name         | str      |  区間の終点ICの名称  |  大泉JCT  |
|  3  |  start_code       | str      |  区間の始点ICコード  |  1800006  |
|  4  |  end_code         | str      |  区間の終点ICコード  |  1110210  |
|  5  |  start_pref_code  | int      |  区間の始点ICがおける[県コード](https://nlftp.mlit.go.jp/ksj/gml/codelist/PrefCd.html)  |  11  |
|  6  |  end_pref_code    | int      |  区間の終点ICがおける[県コード](https://nlftp.mlit.go.jp/ksj/gml/codelist/PrefCd.html)  |  13  |
|  7  |  start_lat        | float    |  区間の始点ICの緯度  |  35.80615  |
|  8  |  end_lat          | float    |  区間の終点ICの緯度  |  35.75582  |
|  9  |  start_lng        | float    |  区間の始点ICの経度  |  139.535511  |
|  10 |  end_lng          | float    |  区間の終点ICの経度  |  139.601514  |
|  11 |  start_degree     | float    |  区間の始点ICが接続する他ICの数  |  2.0  |
|  12 |  end_degree       | float    |  区間の終点ICが接続する他ICの数  |  4.0  |
|  13 |  direction        | str      |  方向  |  上り  |
|  14 |  start_KP         | float    |  始点ICの[KP](#kpキロポストとは)(km)  |  43.7  |
|  15 |  end_KP           | float    |  終点ICの[KP](#kpキロポストとは)(km)  |  35.7  |
|  16 |  limit_speed      | float    |  区間の制限速度(km/h)  |  100.0  |
|  17 |  road_code        | int      |  道路コード(関越道:1800, 東北道:1040)  |  1800  |

- road_all.csv
  - 関越道・東北道を含む全国道路における各区間の始点、終点や⽅向、距離などの属性データ.ドラぷらルート検索ログデータと合わせて使⽤します。 サイズ:328Kb

|  No |  column           |  型      |  説明   |  値例  |
| :---|  :---             |  :---:   |  :---  |  :---  |
|  1  |  start_name       | str      |  区間の始点ICの名称  |  所沢  |
|  2  |  end_name         | str      |  区間の終点ICの名称  |  ⼤泉ＪＣＴ  |
|  3  |  start_code       | str      |  区間の始点ICコード  |  1800006  |
|  4  |  end_code         | str      |  区間の終点ICコード  |  1110210  |
|  5  |  direction        | str      |  方向  |  上り  |
|  6  |  distance         | float    |  区間距離  |  5.0  |
|  7  |  road_code        | str      |  道路コード  |  1800  |

#### KP(キロポスト)とは
*KPは起点からの距離を表す. 関越道起点は大泉JCT（練馬IC）, 館山道起点は篠崎IC（京葉道路）としている.*
<br>
<br>

#### トラフィックカウンター(トラカン)データ

- train.csv
  - 各道路区間を通過する自動車の速度, 台数などを記録した時系列データ. サイズ:458Mb

|  No  |  column           |  型      |  説明  |  値例  |
| :--- |  :---             |  :---:   |  :---  |  :---  |
|  1   |  datetime         | str      |  日時  |  2021-04-08 00:00:00  |
|  2   |  start_code       | int      |  区間の始点ICコード  |  1800006  |
|  3   |  end_code         | int      |  区間の終点ICコード  |  1110210  |
|  4   |  KP               | float    |  トラフィックカウンターが位置するキロポストの値  |  2.26  |
|  5   |  allCars          | int      |  1時間内の全車線の通過台数の合計  |  568  |
|  6   |  OCC              | float    |  1時間内の全車線の占有率(%)  |  1.6  |
|  7   |  speed            | float    |  1時間内の全車線の平均速度  |  87.0  |
|  8   |  is_congestion    | int      |  渋滞あり・なし(目的変数)  |  0 or 1  |

*※speedが40以下の場合にis_congestionが1(渋滞あり)と定義する.*

<br>
<br>

#### ドラぷら検索ログ二次データ

- search_specified.csv
  - 各道路区間の時間指定**あり**検索数. サイズ:66.4Mb

|  No |  column           |  型      |  説明  |  値例  |
| :---|  :---             |  :---:   |  :---  |  :---  |
|  1  |  datetime         | str      |  日時 |  2021-04-08 00:00:00  |
|  2  |  start_code       | int      |  区間の始点ICコード  |  1800006  |
|  3  |  end_code         | int      |  区間の終点ICコード  |  1110210  |
|  4  |  search_specified | float    |  1時間ごとに集計したドラぷらルート検索データ(時間指定あり検索数)<br>datetimeの日時にstart_code→end_code間を通過する検索結果を集計したもの(集計はdatetimeから過去7日間分)  |  15.0  |

<br>
<br>


- search_unspecified.csv
  - 各道路区間の時間指定**なし**検索数. サイズ:2.3Mb  

|  No |  column           |  型      |  説明  |  値例  |
| :---|  :---             |  :---:   |  :---  |  :---  |
|  1  | date              | str      |  日付  |  2021-04-08  |
|  2  | start_code        | int      |  区間の始点ICコード  |  1800006  |
|  3  | end_code          | int      |  区間の終点ICコード  |  1110210  |
|  4  | search_unspecified| float    |  前日分として集計したドラぷらルート検索データ(時間指定なし検索数)<br>dateの日付にstart_code→end_code間を通過する検索結果を集計したもの(集計はdateの前日分)  |  2142.0  |

<br>
<br>

##### ドラぷら検索ログ一次データ

- search_raw_log.zip
  - ドラぷらから収集された検索履歴をまとめたフォルダ. サイズ:536.9Mb<br>
  実体は"search_raw_log.zip"で, 解凍すると以下のようなディレクトリ構造のデータが作成される.(解凍時サイズ: 4.38Gb)
```bash
search_raw_log
├─ 2021_04_01.csv
├─ 2021_04_02.csv
├─ ...
├─ 2023_07_30.csv
└─ 2023_07_31.csv
```

例: 2021_04_01.csvの場合、2021年4月1日に検索されたドラぷら検索ログデータが格納されている.

|  No |  column           |  型      |  説明  |  値例  |
| :---|  :---             |  :---:   |  :---  |  :---  |
|  1  |  datetime         | str      |  検索日時  |  2021-04-01 00:00:10  |
|  2  |  start_code       | str      |  区間の始点ICコード  |  1040263  |
|  3  |  end_code         | str      |  区間の終点ICコード  |  1010046  |
|  4  |  spec_datetime    | float    |  指定日時  |  2021-04-01 07:00:00  |
|  5  |  spec_type        | int      |  指定日時の種別（出発時 or 到着時）  |  1 or 2  |
|  6  |  car_type         | int      |  車両種別（軽自動車等, 普通車, 中型車, 大型車, 特大車）  |  1 or 2 or 3 or 4 or 5  |


### 分析用追加配布プログラム
以下は予測モデルを作成する際に参考となるコードやプログラムである.

```bash
├─ baseline.ipynb
├─ expressway_router.py
└─ test_expressway_router.ipynb
```
- baseline.ipynb
  - モデルの前処理・学習や検索ログの処理方法などが記載されているコード.
- expressway_router.py
  - 高速道路上のIC間経路を計算するクラスを定義したプログラム.
  - 詳細な使い方はtest_expressway_router.ipynbの内部を参照.
- test_expressway_router.ipynb
  - expressway_router.pyの使用方法が記載されたプログラム.
  - 指定した区間内の移動時間や経由・通過するICや時刻の検索方法が記載されている.

### 動作確認用のプログラム

予測を行うプログラムの動作を確認するためのプログラム. "run_test.zip"が本体で, 解凍すると, 以下のようなディレクトリ構造のデータが作成される.

```bash
run_test
└─ run.py
```

使用方法等については[応募用ファイルの作成方法](#応募用ファイルの作成方法)を参照すること.
<br>
<br>

### 応募用サンプルファイル

応募用のサンプルファイル. 実体は"sample_submit.zip"で, 解凍すると以下のようなディレクトリ構造のデータが作成される.

```bash
sample_submit
├── model
│   └── ...
├── src
│   └── predictor.py
└── requirements.txt
```

sample_submit/modelは空のディレクトリとなる. sample_submit/requirements.txtには何も書いていない. 詳細や作成方法については[応募用ファイルの作成方法](#応募用ファイルの作成方法)を参照すること.
<br>
<br>

## 応募用ファイルの作成方法

学習済みモデルを含めた, 予測を実行するためのソースコード一式をzipファイルでまとめたものとする.
<br>
<br>

### ディレクトリ構造

以下のようなディレクトリ構造となっていることを想定している.

```bash
sample_submit
├── model              必須: 学習済モデルを置くディレクトリ
│   └── ...
├── src                必須: Pythonのプログラムを置くディレクトリ
│   ├── predictor.py   必須: 投稿時に予測プログラムとして呼び出されるファイル
│   └── ...            任意: その他のファイル (ディレクトリ作成可能)
└── requirements.txt   任意: 追加で必要なライブラリ一覧
```

- 学習済みモデルの格納場所は"model"ディレクトリを想定している.
  - 学習済みモデルを使用しない場合でも空のディレクトリを作成する必要がある.
  - 名前は必ず"model"とすること.
- Pythonのプログラムの格納場所は"src"ディレクトリを想定している.
  - 学習済みモデル等を読み込んで推論するためのメインのソースコードは"predictor.py"を想定している.
    - ファイル名は必ず"predictor.py"とすること.
  - その他予測を実行するために必要なファイルがあれば作成可能である.
  - ディレクトリ名は必ず"src"とすること.
- 実行するために追加で必要なライブラリがあれば, その一覧を"requirements.txt"に記載することで, 評価システム上でも実行可能となる.
  - インストール可能で実行可能かどうか予めローカル環境で試しておくこと.
  - 評価システムの実行環境については, [*こちら*](https://github.com/signatelab/runtime-gpu)を参照すること.
<br>
<br>

### predictor.pyの実装方法

以下のクラスとメソッドを実装すること.
<br>
<br>

#### ScoringService

予測実行のためのクラス. 以下のメソッドを実装すること.
<br>
<br>

##### get_model

モデルを取得するメソッド. 以下の条件を満たす必要がある.

- クラスメソッドであること.
- 引数としてmodel_path(str型)を指定すること.
  - 学習済みモデルが格納されているディレクトリのパスが渡される.
- 引数としてinference_df(DataFrame型)を指定すること.
  - 予測対象の期間よりも過去のDataFrameが推論補助データとして渡される. 渡されるDataFrameは下記.
    - ローカルでの検証時: 2021-04-08以降, かつ検証実行時に指定したstart_date以前DataFrame
    - 暫定評価時: 2021-04-08 ~ 2023-07-31のDataFrame
    - 最終評価時: 2021-04-08 ~ 2024-03-31のDataFrame
    - inference_dfに与えられるDataFrameのフォーマットは以下
      - ※search_specifiedとsearch_unspecifiedについてはdatetimeと**同じ日**の数値が渡される<br>
      例: datetimeが2023-08-01の場合、対応するsearch_specifiedとsearch_unspecifiedには当日の2023-08-01の数値が入力される


|  No |  column            |  型       |
| :---|  :---              |  :---:    |
|  1  | datetime           |  datetime |
|  2  | start_code         |  int      |
|  3  | end_code           |  int      |
|  4  | KP                 |  float    |
|  5  | OCC                |  float    |
|  6  | allCars            |  int      |
|  7  | speed              |  float    |
|  8  | is_congestion      |  int      |
|  9  | start_name         |  str      |
|  10 | end_name           |  str      |
|  11 | road_code          |  int      |
|  12 | direction          |  str      |
|  13 | limit_speed        |  float    |
|  14 | start_KP           |  float    |
|  15 | end_KP             |  float    |
|  16 | start_pref_code    |  int      |
|  17 | end_pref_code      |  int      |
|  18 | start_lat          |  float    |
|  19 | end_lat            |  float    |
|  20 | start_lng          |  float    |
|  21 | end_lng            |  float    |
|  22 | start_degree       |  float    |
|  23 | end_degree         |  float    |
|  24 | search_specified   |  float    |
|  25 | search_unspecified |  float    |

- ログデータには引数としてinference_log(データパスのリスト)を指定すること.
  - 予測対象の期間よりも過去のデータパスが推論補助データとして渡される. 
    - ローカルでの検証時: 2021-04-01以降, かつ検証実行時に指定したstart_date以前DataFrame
    - 暫定評価時: 2021-04-01 ~ 2023-07-31のDataFrame
    - 最終評価時: 2021-04-01 ~ 2024-03-31のDataFrame
    - inference_logの各パスを読み込み得られるDataFrameのフォーマットは以下.

|  No |  column            |  型       |
| :---|  :---              |  :---:    |
|  1  | datetime           |  str      |
|  2  | start_code         |  str      |
|  3  | end_code           |  str      |
|  4  | spec_datetime      |  str      |
|  5  | spec_type          |  int      |
|  6  | car_type           |  int      |

  **※最終評価では暫定評価時よりも読み込まれるinference_dfのデータの期間が長いため、暫定評価時に動作していたコードが最終評価でメモリエラー・タイムアウトで評価されない可能性がございます。このため投稿時は消費メモリや実行時間に関してあらかじめ余裕を持って投稿することを推奨します。**
- 学習済みモデルの読み込みに成功した場合はTrueを返す.
  - モデル自体は任意の名前(例えば"model")で保存しておく.
<br>
<br>

##### predict

予測を実行するメソッド. 以下の条件を満たす必要がある.

- クラスメソッドであること.
- 引数としてinput(DataFrame型)を指定すること.
  - inputには**当日**の[学習用データ](#学習用データ)をマージしたものがDataFrameで渡される.
  - `get_model`メソッドで読み込んだ学習済みモデルや推論補助データを用いて**翌日**の渋滞予測を行う想定である.
  - inputに与えられるDataFrameのフォーマットは以下.
    - ※search_specifiedとsearch_unspecifiedについてはdatetimeの**翌日(予測対象日)**の数値が渡される<br>
      例: datetimeが2023-08-01の場合、対応するsearch_specifiedとsearch_unspecifiedには予測対象日の2023-08-02の数値が入力される


|  No |  column            |  型       |
| :---|  :---              |  :---:    |
|  1  | datetime           |  datetime |
|  2  | start_code         |  int      |
|  3  | end_code           |  int      |
|  4  | KP                 |  float    |
|  5  | OCC                |  float    |
|  6  | allCars            |  int      |
|  7  | speed              |  float    |
|  8  | is_congestion      |  int      |
|  9  | start_name         |  str      |
|  10 | end_name           |  str      |
|  11 | road_code          |  int      |
|  12 | direction          |  str      |
|  13 | limit_speed        |  float    |
|  14 | start_KP           |  float    |
|  15 | end_KP             |  float    |
|  16 | start_pref_code    |  int      |
|  17 | end_pref_code      |  int      |
|  18 | start_lat          |  float    |
|  19 | end_lat            |  float    |
|  20 | start_lng          |  float    |
|  21 | end_lng            |  float    |
|  22 | start_degree       |  float    |
|  23 | end_degree         |  float    |
|  24 | search_specified   |  float    |
|  25 | search_unspecified |  float    |

<br>
<br>

  - input_logに与えられるDataFrameのフォーマットは以下.


|  No |  column            |  型       |
| :---|  :---              |  :---:    |
|  1  | datetime           |  str      |
|  2  | start_code         |  str      |
|  3  | end_code           |  str      |
|  4  | spec_datetime      |  str      |
|  5  | spec_type          |  int      |
|  6  | car_type           |  int      |

<br>
<br>


- 予測結果を**DataFrame型**で返すように作成すること. フォーマットは以下.

|  No |  column           |  型            |
| :---|  :---             |  :---:         |
|  1  |  datetime         |  datetime      |
|  2  |  start_code       |  int/str       |
|  3  |  end_code         |  int/str       |
|  4  |  KP               |  float/str     |
|  5  |  prediction       |  int           |

- 各要素は以下を満たす.
  - "datetime", "start_code", "end_code", "KP"は, ある区間のある日時を表すindexである.
    - 注意点: pandasのバージョンに応じて"KP"のデータ型が変わる可能性がございます.データ型が変わると想定した予測結果とならず評価されない可能性がありますので、pandasなどのデータフレームのバージョンにはご注意ください.
  - "prediction"は, 各indexの翌日が渋滞しているかどうかを予測したもの(0か1)とすること.

<br>
<br>

##### 実装例
以下は実装例. 翌日が土曜日もしくは日曜日の場合にのみ, 渋滞と予測する単純な処理である.

```Python
import pandas as pd

class ScoringService(object):
    @classmethod
    def get_model(cls, model_path, inference_df, inference_log):
        """Get model method

        Args:
            model_path (str): Path to the trained model directory.
            inference_df: Past data not subject to prediction.
            inference_log: Past log data that is not subject to prediction.

        Returns:
            bool: The return value. True for success.
        """
        cls.model = None
        cls.data = inference_df
        cls.log_pathes = inference_log

        return True


    @classmethod
    def predict(cls, input, input_log):
        """Predict method

        Args:
            input: meta data of the sample you want to make inference from (DataFrame)
            input_log: meta data of the sample you want to make inference from (DataFrame)

        Returns:
            prediction: Inference for the given input. Return columns must be ['datetime', 'start_code', 'end_code', 'KP'](DataFrame).
        """

        prediction = input.copy()
        prediction['weekday'] = pd.to_datetime(prediction['datetime'], format='%Y-%m-%d').apply(lambda x: x.weekday())
        prediction['prediction'] = prediction['weekday'].apply(lambda x: 1 if x==4 or x==5 else 0)
        prediction = prediction[['datetime', 'start_code', 'end_code', 'KP', 'prediction']]

        return prediction
```

応募用サンプルファイル"sample_submit.zip"も参照すること.

##### runtime投稿の評価の流れについて
runtimeの評価について、投稿ファイルの実行は「デバッグ」→「本番」の流れで行われます.

2つの流れで使用されるデータは異なっており、以下のようなデータが受け渡されます

「デバッグ」では以下のデータが入力されます.
- 推論用データ(inference_df): 2023-06-01～2023-06-30
- 推論用データ(inference_log): 2023-05-25〜2023-06-30
- 評価用データ(input_df): 2023-07-01〜2023-07-31
- 評価用データ(input_log): 2023-07-01〜2023-07-31

「本番」では以下のデータが入力されます.

- 暫定評価時
  - 推論用データ(inference_df): 2021-04-08～2023-07-31
  - 推論用データ(inference_log): 2021-04-01〜2023-07-31
  - 評価用データ(input_df): 2023-08-01〜2023-09-30
  - 評価用データ(input_log): 2023-08-01〜2023-09-30
- 最終評価時
  - 推論用データ(inference_df): 2021-04-08～2024-03-31
  - 推論用データ(inference_log): 2021-04-01〜2024-03-31
  - 評価用データ(input_df): 2024-04-01〜2024-05-06
  - 評価用データ(input_log): 2024-04-01〜2024-05-06

このため、暫定評価・最終評価のデータ量前提で実装を行うと、手元の推論で問題なく動作してもデバッグの段階でエラーが発生する恐れがございます.

実装する際は、このデバッグの段階の日付にも対応するように実装してください.
<br>
<br>

### 推論テスト

予測を行うプログラムが実装できたら, 推論テスト(正常に動作するかの確認)を行う.
推論テストを行うには下記のようなディレクトリ構造を作成する.

```bash
run_test
├── sample_submit
│   ├── model
│   │   └── ...
│   ├── src
│   │   └── predictor.py
│   └── requirements.txt
├── train
│   ├─ search_specified.csv
│   ├─ search_unspecified.csv
│   ├─ train.csv
│   └─ search_raw_log
│         ├─ 2021_04_01.csv
│         ├─ 2021_04_02.csv
│         ├─ ...
│         ├─ 2023_07_30.csv
│         └─ 2023_07_31.csv
└── run.py

```

<br>
<br>

#### 環境構築

評価システムと[同じ環境](https://github.com/signatelab/runtime-gpu)を用意する.
<br>
<br>

#### 予測の実行手順

1. [推論テストのディレクトリ構造](#推論テスト)を参照し, 同じ構造のファイルを作成
2. src/predictor.pyに, 予測プログラムを実装する.
    - 機械学習モデルを作成する場合, 下記手順で実装する.
      - 配布データを[input](#predict)のフォーマットに変形し学習を行い, modelディレクトリ内に作成したモデルを格納する.
      - modelディレクトリから学習済みモデルを呼び出し, 再学習や予測を行う処理をpredictor.pyの`predict`メソッドに実装する.
    - inference_dfを使用する場合, `get_model`メソッド内で受け取ったinference_dfを`predict`メソッドで呼び出すことができる. (デフォルトの場合`cls.data`に格納されている)
3. [動作確認用のプログラム](#動作確認用のプログラム)を用いて予測を実行し, 最後に出力される精度(F1_score)を確認する. 下記コマンドを実行することで処理が行われる.


```bash
$ cd /path/run_test
$ python run.py  --exec-path /path/to/submit/src --data-dir /path/to/train --start-date start_date --end-date end_date
...
```

- 引数"--exec-path"には実装した予測プログラム("predictor.py")が存在するパス名を指定する.
- 引数"--data-dir"には配布された学習用データ"train"のパス名を指定する.
- 引数"--start-date"には検証用データを作成するときの最初の日付(2021-04-08 ~ 2023-07-31)をYYYY-MM-ddのフォーマットで指定する. (指定しない場合は2023-07-01)
- 引数"--end-date"には検証用データを作成するときの最後の(2021-04-08 ~ 2023-07-31)の日付をYYYY-MM-ddのフォーマットで指定する. (指定しない場合は2023-07-31)
  - "--start-date"で指定した日付よりも後の日付を設定すること.

"run.py"の実行に成功すると, 検証用のデータを作成し実装したプログラム(predictor.py)による予測が行われ, 最後に精度が出力される. 
プログラム上のエラーがない場合でも, モデルの読み込みに失敗したときや, 予測結果のフォーマットが正しくないときは途中でエラーのメッセージが返され, 止まる. 
具体例は下記.
- `get_model`メソッドを呼んだときにTrueを返さない.
- 予測結果がDataFrame型でない.
- `predict`メソッドから出力されるpredictionのcolumnが"datetime", "start_code", "end_code", "KP", "prediction"でない.

投稿する前にエラーが出ずに実行が成功することを確認すること.
<br>
<br>

### 応募用ファイルの作成

上記の[ディレクトリ構造](#ディレクトリ構造)となっていることを確認して, zipファイルとして圧縮する.