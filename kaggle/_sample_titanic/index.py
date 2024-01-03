# ライブラリをインポート
import pandas as pd
from sklearn.ensemble import RandomForestClassifier
from sklearn.metrics import accuracy_score
from sklearn.model_selection import train_test_split

# データを読み込む
train_data = pd.read_csv('train.csv')
test_data = pd.read_csv('test.csv')

# データの前処理
# 'Age'列の欠損値を中央値で置き換える
train_data['Age'].fillna(train_data['Age'].median(), inplace=True)
test_data['Age'].fillna(test_data['Age'].median(), inplace=True)

# 'Embarked'列の欠損値を最頻値で置き換える
train_data['Embarked'].fillna(train_data['Embarked'].mode()[0], inplace=True)

# 'Fare'列の欠損値を中央値で置き換える
test_data['Fare'].fillna(test_data['Fare'].median(), inplace=True)

# 'Cabin'列を削除する
train_data.drop('Cabin', axis=1, inplace=True)
test_data.drop('Cabin', axis=1, inplace=True)

# 'Name'列と'Ticket'列を削除する
train_data.drop(['Name', 'Ticket'], axis=1, inplace=True)
test_data.drop(['Name', 'Ticket'], axis=1, inplace=True)

# カテゴリ変数をダミー変数に変換する
train_data = pd.get_dummies(train_data, columns=['Sex', 'Embarked'])
test_data = pd.get_dummies(test_data, columns=['Sex', 'Embarked'])

# 訓練用データセットを作成する
X = train_data.drop(['Survived', 'PassengerId'], axis=1)
y = train_data['Survived']
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)

# モデルをトレーニングする
model = RandomForestClassifier(n_estimators=100, random_state=42)
model.fit(X_train, y_train)

# モデルの評価
y_pred = model.predict(X_test)
accuracy = accuracy_score(y_test, y_pred)
print(f'Accuracy: {accuracy:.2f}')

# テストデータの予測を生成する
X_test = test_data.drop('PassengerId', axis=1)
y_pred_test = model.predict(X_test)

# 提出用ファイルを作成する
submission = pd.DataFrame({
    'PassengerId': test_data['PassengerId'],
    'Survived': y_pred_test
})
submission.to_csv('submission.csv', index=False)
