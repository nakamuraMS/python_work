"""
高速道路上のIC間経路を計算するクラスを定義したモジュール
"""

from typing import Dict, List, Literal, Tuple

import networkx as nx
import pandas as pd


class ExpresswayRouter:
    # 省略すべきICコード（SA/PAなど）
    __SMALL_IC_SET = {
        "1040014",
        "1040019",
        "1040021",
        "1040041",
        "1040071",
        "1040081",
        "1040111",
        "1800004",
        "1800031",
        "1800046",
        "1800093",
        "1800101",
    }

    def __init__(
        self,
        road_all_csv: str = "./road_all.csv",
        road_local_csv: str = "./road_local.csv",
    ) -> None:
        # 道路構造データから高速道路の有向グラフを構築
        dtype = {"start_code": str, "end_code": str, "road_code": str}
        df_road_all = pd.read_csv(road_all_csv, dtype=dtype)
        df_road_local = pd.read_csv(road_local_csv, dtype=dtype)

        self.ic_graph_all = nx.from_pandas_edgelist(
            df_road_all,
            source="start_code",
            target="end_code",
            edge_attr=["distance", "road_code", "direction"],
            create_using=nx.DiGraph(),
        )
        self.ic_graph_local = nx.from_pandas_edgelist(
            df_road_local,
            source="start_code",
            target="end_code",
            edge_attr=["limit_speed", "road_code", "direction"],
            create_using=nx.DiGraph(),
        )

        self.__ic_nodes_set: set = set(self.ic_graph_all.nodes)

        # 有向グラフを基にダイクストラ法で事前に全IC間の最短経路を計算
        print(f"[{self.__class__.__name__}] Loading Road Network...")
        self.__route_dict: Dict[str, Dict[str, List[str]]] = dict(
            nx.all_pairs_dijkstra_path(self.ic_graph_all, weight="distance")
        )
        print(f"[{self.__class__.__name__}] Finished.")

    def __get_route(self, src: str, dest: str) -> List[str]:
        if not (src in self.__ic_nodes_set and dest in self.__ic_nodes_set):
            return []

        try:
            path = self.__route_dict[src][dest]
            return path
        except KeyError:  # 経路が存在しない場合
            return []

    def get_route(self, src: str, dest: str) -> List[str]:
        """
        高速道路上で出発ICから目的ICに至るIC経路を得る関数

        Parameters
        ----------
        src: str
            出発ICコード
        dest: str
            目的ICコード

        Returns
        -------
        path: List[str]
            出発ICから目的ICに至るまでに通過するICコードの配列
            出発・目的ICが存在しない場合や最短経路が存在しない場合は空配列が返る
        """
        path = [
            ic
            for ic in self.__get_route(src, dest)
            if ic not in ExpresswayRouter.__SMALL_IC_SET
        ]
        return path

    def get_route_with_time(
        self, src: str, dest: str, spec_datetime: pd.Timestamp, spec_type: Literal[1, 2]
    ) -> List[Tuple[str, pd.Timestamp]]:
        """
        高速道路上で出発ICから目的ICに至るIC経路と各ICの予想通過時刻を得る関数

        Parameters
        ----------
        src: str
            出発ICコード
        dest: str
            目的ICコード
        spec_datetime: pd.Timestamp
            指定日時
        spec_type: Literal[1, 2]
            指定種別（出発時 or 到着時）

        Returns
        -------
        path_with_time: List[Tuple[str, pd.Timestamp]]
            出発ICから目的ICに至るまでに通過するICコードとその予想通過時刻の配列
            出発・目的ICが存在しない場合や最短経路が存在しない場合は空配列が返る
        """
        # 関越道・東北道以外の道路の移動速度を80km/hと仮定する
        DEFAULT_SPEED = 80

        path = self.__get_route(src, dest)

        # 経路中の各区間の予想通過時刻を計算する
        elapsed = pd.Timedelta(0, unit="h")
        elapsed_time_list = [elapsed]
        for start_ic, end_ic in zip(path, path[1:]):
            distance = self.ic_graph_all[start_ic][end_ic]["distance"]
            try:
                speed = self.ic_graph_local[start_ic][end_ic]["limit_speed"]
            except KeyError:
                speed = DEFAULT_SPEED

            # 所要時間を積算
            td_hour = pd.Timedelta(distance / speed, unit="h")
            elapsed += td_hour
            elapsed_time_list.append(elapsed)

        if spec_type == 1:  # 出発時指定
            passage_time_list = [spec_datetime + td for td in elapsed_time_list]
        elif spec_type == 2:  # 到着時指定
            passage_time_list = [spec_datetime - td for td in elapsed_time_list[::-1]]
        else:
            raise ValueError("spec_type value error")

        path_with_time = [
            (ic, time)
            for (ic, time) in list(zip(path, passage_time_list))
            if ic not in ExpresswayRouter.__SMALL_IC_SET
        ]
        return path_with_time
