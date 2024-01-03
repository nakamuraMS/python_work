import pandas as pd
from sklearn.preprocessing import OneHotEncoder
from sklearn.preprocessing import LabelEncoder

def one_hot_encoding(data, columns):
    """
    カテゴリーをOne-Hotエンコーディング

    Parameters
    ----------
    data : array
        エンコードしたいデータ
    columns : array
        エンコーディング後のカラム

    Returns
    -------
    data : array
        変換後のデータ
    """
    data_categories = data[['sex','smoker','region']]
    ohe = OneHotEncoder(sparse=False, categories='auto',dtype=int)
    sl_np = ohe.fit_transform(data_categories)
    # データを作り替える
    sl_df = pd.DataFrame(data = sl_np, columns = columns)
    return pd.concat([data.drop(['sex','smoker','region'], axis=1), sl_df], axis=1)

def label_encoding(data):
    """
    カテゴリーをLabelエンコーディング

    Parameters
    ----------
    data : array
        エンコードしたいデータ(カラム指定)

    Returns
    -------
    data : array
        変換後のデータ
    """
    LE = LabelEncoder()
    return LE.fit_transform(data.values)

def count_encoding(data, column_name):
    """
    カテゴリーをcountエンコーディング

    Parameters
    ----------
    data : array
        エンコードしたいデータ
    column_name : string
        カラム名

    Returns
    -------
    data : array
        変換後のデータ
    """
    return data.groupby(column_name)[column_name].transform('count')


def data_binned(data, bins, labels):
    """
    ビン分割
    区切る数とラベルの数は合わせる必要がある

    Parameters
    ----------
    data : array
        エンコードしたいデータ(カラム指定)
    bins : array
        区切る数
    labels : array
        ラベル

    Returns
    -------
    data : array
        変換後のデータ
    """

    # データのビン分割
    return pd.cut(data, bins=bins, labels=labels)