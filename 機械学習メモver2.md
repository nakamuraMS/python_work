# 評価指標
## 回帰問題における評価関数
> 評価指標
>> モデルの性能(予測精度)を定量的に表すための関数  
>> 評価関数の値が改善するようにモデル開発を行う

> MAE(Mean Absolute Error)
>> 実測値と予測値の「誤差の絶対値」を元に算出する指標  
>> 平均絶対値誤差とも呼ばれ、値が小さい(0に近い)ほど予測精度が高い  
>> RMSEとともに、回帰問題で一般的に使用される  
>> 例）平均年収予測  

> RMSE(Root Mean Squared Error)
>> 実測値と予測値の「誤差の2乗」を元に算出する指標  
>> 平均平方2乗誤差とも呼ばれ、値が小さい(0に近い)ほど予測精度が高い  
>> 誤差を2乗しているため、外れ値(実測と予測の大きな乖離)が一点でもあると、RMSEは著しく大きくなる  
>> 例）店舗別の売上予測  

> MAPE(Mean Absolute Percentage Error)
>> 実測値に対する「誤差の割合(誤差率)」を元に算出する指標  
>> 平均絶対パーセント誤差呼ばれ、値が小さい(0に近い)ほど予測精度が高い  
>> 例）メーカー別の売上高予測(スケールの異なるデータ)  

> R2(決定係数)
>> 「データそのもののばらつき（分散）」と「予測値のズレ」を元に算出する指標  
>> モデルの予測値の当てはまり度合い(適合性)を評価  
>> 値が1に近いほど予測精度が高い  

## 分類問題における評価関数
> 混同行列
>> 分類結果(正しく分類した数/誤って分類した数)をまとめた表  
>> 特に２値分類(0か1かの分類)のモデルの性能を図る指標として使用される  
>> 例）迷惑メール(スパム)かどうかの判定問題  

>> Accuracy(正解率)  
>>> 予想結果が実際に当たっていた割合  

>> Precision(適合率)  
>>> 陽性であると予測したもののうち、実際に陽性だった割合  
>>> 誤判定を避けたい場合はこれを重視  

>> Recall(真陽性率、再現率)  
>>> 実際に陽性であるもののうち、正しく陽性であると予測できた割合  
>>> 見落としを避けたい場合はこれを重視  

> F値(F1-score、F-measuer)
>> PrecisionとRecallの両方を加味した評価指標  
>> 0から1の値を取り、値が大きいほど予測精度が高い

> ROC曲線(Receiver Operating Characteristic)
>> 2値分類問題で、閾値を変化された時、モデルの性能が  
>> どのように変わるかを可視化する手法  

>> AUC(Area Under the Curve)  
>>> ROC曲線の下の領域面積  
>>> 2値分類の評価指標で0~1の値を取り値が大きいほどよい  

## 推薦・検索問題における評価関数
> Precision@k  
>> 推薦度上位k件に占める、適合アイテムの割合  

> Recall@k  
>> 全ての適合アイテムに占める、推薦度上位k件に含まれる適合アイテムの割合  

> AP(Average Precision)  
>> 適合アイテムが得られた時点での適合率の平均を取った値  

> MAP(Mean Average Precision)  
>> ユーザ毎に計算したAPの平均を取った値  

> DCG(Discounted Cumulative Gain)  
>> 推薦したアイテムの関連度に重みをつけて合計する  
>> 関連度はユーザがアイテムにどの程度適合しているかを表す数値  
>> 下位に順位付けしたアイテムほど重みが減衰する  

> nDCG(normalized Discounted Cumulative Gain)  
>> DCGを理想的な順位付けを行った時のDCGprefectで割った値  
>> DCGを0~1の値を取るように正規化したもの  

## 物体検出（領域検出）問題における評価関数
> IoU(Intersection over Union)  
>> 検出領域と真の領域の一致度を測る指標  

> mean Average Precision(mAP)  
>> マルチクラスの物体検出問題において、  
>> 各クラスについて算出したAverage Precisionの平均値をmAPとする  


# scikit-learn
## データ前処理
> データの標準化【Standardscaler】
>> 平均を0に、標準偏差を1にするスケーリング

> データの正規化【Normalization】
>> 特徴量の値の範囲を一定の範囲におさめるスケーリング

> ラベルエンコード【LabelEncoder】
>> カテゴリ変数を数値にエンコードすること

> ワンホットエンコード【OneHotEncoder】

## データ分割
- 学習データとテストデータへの分割【train_test_split】
- K-分割交差検証【KFold】
- 層化K-分割交差検証【StratifiedKFold】
- グループ付き交差検証【GroupKFold】
- 時系列データ分割【TimeSeriesSplit】

## 教師あり学習
> 線形回帰【LinearRegression】
>> 単回帰分析  
>> 重回帰分析

> ロジスティック回帰【LogisticRegression】
>> 2値分類  
>> 多値分類

> ランダムフォレスト【RandomForest】
>> 分類  
>> 回帰

> サポートベクターマシン【SupportVectorMachine】
>> サポートベクターマシンを使った分類【GridSearchCV】

> K近傍法【K-NearestNeighbor】
>> 分類  
>> 回帰

## 教師なし学習
> PCA【Principal Component Analysis】

> k-means法
>> クラスタリング（分類）  
>> エルボー法を使ったk-meansクラスタリング

> t-SNE
>> t-SNEを使った次元圧縮

## 機械学習モデルの評価
### 分類問題に対しての評価
> AccuracyScore  
> ConfusionMatrix【混合行列】  
> ClassificationReport

### 回帰問題に対しての評価
> MeanAbsoluteError【平均絶対誤差】  
> MeanSquaredError【平均二乗誤差】  
> R²Score【決定係数】


伝説のポケモンかどうかの判断モデル爆速で作るぜ（PyCaret, fasteda）
https://qiita.com/MAsa_min/items/8bd7625510ca48382b8e