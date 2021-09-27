"""
This is my SPARS algorithm, which only implement visible method.
"""

from ctypes.wintypes import BOOL
import math
import numpy as np
import random
import matplotlib.pyplot as plt

from typing import Tuple

show_animation = True

class MySpars:
    """
    docstring
    """
    class Node:
        def __init__(self, x : float, y : float, cost : float , parent_index : int) -> None:
            """
            docstring
            """
            self.x = x
            self.y = y
            self.cost = cost
            self.parent_index = parent_index
        def __str__(self) -> str:
            return str(self.x) + ", " + str(self.y) + ", " + str(self.cost) + ", " + str(self.parent_index)

    def __init__(self) -> None:
        self.road_map = []
        self.sample_x = []
        self.sample_y = []
        self.edges = []

    @staticmethod
    def searchNearestFromList(sample_x : float, sample_y : float, ox : list[float], oy : list[float]) -> Tuple[int, float] :
        min_index = 0
        min_dist = np.hypot(ox[0] - sample_x, oy[0] - sample_y)
        for i in range(len(ox)):
            dist = np.hypot(ox[i] - sample_x, oy[i] - sample_y)
            if (dist < min_dist):
                min_dist = dist
                min_index = i

        #print("nearest index : {}, dist : {}".format(min_index, min_dist))

        return min_index, min_dist

    def searchVisibleNode(self, sample_list_x : list[float], sample_list_y : list[float], target_x : float, target_y : float, delta : float, ox : list[float], oy : list [float], rr : float) -> list[int]:
        """
        sample_list_x : 探索対象の x座標 のリスト
        sample_list_y : 探索対象の y座標 のリスト
        target_x : 基準の x座標
        target_y : 基準の y座標
        radius : 半径
        
        return : 探索対象のリストのうち、基準の座標から指定された半径の中にある座標のインデックスのリスト。
        """
        near_nodes = []
        ans = []
        for i in range(len(sample_list_x)):
            dist = np.hypot(sample_list_x[i] - target_x, sample_list_y[i] - target_y)
            if (dist < delta):
                near_nodes.append(i)

        for node in near_nodes:
            if not self.is_collision(target_x, target_y, sample_list_x[node], sample_list_y[node], rr, ox, oy):
                ans.append(node)

        return ans

    def is_collision(self, base_x : float, base_y : float, target_x : float, target_y : float, radius : float, ox : list[float], oy : list[float]) -> BOOL:
        x = base_x
        y = base_y
        dx = target_x - base_x
        dy = target_y - base_y
        yaw = math.atan2(dy, dx)
        d = np.hypot(dx, dy)

        D = radius
        n_step = round(d / D)

        for i in range(n_step):
            _, dist = self.searchNearestFromList(x, y, ox, oy)
            if dist <= radius:
                return True # collision
            x += D * math.cos(yaw)
            y += D * math.sin(yaw)

        return False # no collision

    @staticmethod
    def get_unique_list(seq):
        seen = []
        return [x for x in seq if x not in seen and not seen.append(x)]

    def checkAddCoverage(self, q_new_x : float, q_new_y : float, visible_nodes : list[int]) -> BOOL:
        """
        半径delta内に他のノードがなければ追加してTrueを返す。
        """
        if len(visible_nodes) == 0: 
            self.sample_x.append(q_new_x)
            self.sample_y.append(q_new_y)
            return True
        return False

    def checkAddConnectivity(self, q_new_x : float, q_new_y : float, visible_nodes : list[int]) -> BOOL:
        """
        本当は union find とか使ってグラフの頂点をコンポーネントとして管理しないと...
        本当は二つのノードが属しているコンポーネントが異なる場合にだけ、q_newを使って接続する。
        本当は二つのノードが属しているコンポーネントが異なっていたら無条件にq_newを頂点に追加し、それらとのエッジも追加する。

        そうすると、障害物と干渉するんじゃない？って思ったけど、そもそもvisible_nodesを探すときに、干渉判定までやっておくのである。
        つまり、visible node とはサンプル点と直接つなぐことができるっていうこと！！

        ここをちゃんと実装すれば、checkAddInterfaceの暫定のところは消せるはず。なぜなら、それらがすでに同じグループにあれば、ここでTrueを返して、
        checkAddInterfaceまで行かないからである。
        """
        is_disconnected_node_found = False
        if len(visible_nodes) > 1:
            for i in range(len(visible_nodes)):
                for j in range(i+1, len(visible_nodes)):
                    pass
                    #is_disconnected_node_found = True

            if is_disconnected_node_found:
                return True

        return False

    def checkAddInterface(self, q_new_x : float, q_new_y : float, visible_nodes : list[int], rr : float, ox : list[float], oy : list[float]) -> BOOL:
        """
        本当はvisible nodeのうち最も近い２つを選んでそれらがエッジを持たない時に追加する。ここでは最も近いノードではなくて配列の0, 1を使っている。でも近いは近いはず。
        """
        if len(visible_nodes) > 1 :
            # 直接つながる場合は挿入しない。
            if not self.is_collision(self.sample_x[visible_nodes[0]], self.sample_y[visible_nodes[0]], self.sample_x[visible_nodes[1]], self.sample_y[visible_nodes[1]], rr, ox, oy):
                self.edges.append([visible_nodes[0], visible_nodes[1]])
                return True
            else:
                if not self.is_collision(self.sample_x[visible_nodes[0]], self.sample_y[visible_nodes[0]], q_new_x, q_new_y, rr, ox, oy):
                    if not self.is_collision(self.sample_x[visible_nodes[1]], self.sample_y[visible_nodes[1]], q_new_x, q_new_y, rr, ox, oy):
                        if len(visible_nodes) < 3: # 暫定でサンプルノードを追加すれば接続できる場合でも近くにたくさん(2個以上)ある場合は追加しない
                            # サンプルノードを経由した場合に干渉しない時はサンプルノードを追加する。
                            self.sample_x.append(q_new_x)
                            self.sample_y.append(q_new_y)
                            self.edges.append([len(self.sample_x)-1, visible_nodes[0]])
                            self.edges.append([len(self.sample_x)-1, visible_nodes[1]])
                            return True

        return False

    def addNode(self, ox : list[float], oy :list[float], rr : float, delta : float) -> None:
        """
        ox : obstacle の x座標 のリスト
        oy : obstacle の y座標 のリスト
        rr : obstacle の rr 以内は干渉と判定する閾値
        delta : 他のサンプル点との距離
        """
        max_x = max(ox)
        min_x = min(ox)
        max_y = max(oy)
        min_y = min(oy)

        is_added_sample = False
        is_sample_found = False
        sample_x = 0
        sample_y = 0

        for i in range(100): # 1個のノードを追加するのに100回までリトライする。
            # サンプル点を生成する
            sample_x = (random.random() * (max_x - min_x)) + min_x
            sample_y = (random.random() * (max_y - min_y)) + min_y

            # 一番近い干渉物を検索し、一番近い干渉物との距離が rr の外にあるかをチェック
            min_index, min_dist = self.searchNearestFromList(sample_x, sample_y, ox, oy)
            if min_dist < rr: # 干渉物に近ければ乱数生成からやり直し。
                continue
            else:
                # サンプル点候補が見つかった
                is_sample_found = True
                break

        if not is_sample_found:
            # 100回の試行でサンプル点の候補が見つからなかった
            return

        # サンプル点に対してdelta内にあるvisible nodes を検索
        visible_nodes = self.searchVisibleNode(self.sample_x, self.sample_y, sample_x, sample_y, delta, ox, oy, rr)
        if not self.checkAddCoverage(sample_x, sample_y, visible_nodes):
            # Github上の実装だと全てのvisibleなノードの組み合わせについて、同じコンポーネント（つまり接続されているか）をチェックして、同じコンポーネントにない同士をサンプル点を使って接続している。
            # ただし以下の実装では半径delta内に他のノードがあって、そのノード間を直接接続できれば接続する。
            # この場合、サンプルノードは追加しない。
            # ただし、ノード間をサンプルノードを経由することで接続できればサンプルノードも追加する。
            # ここは論文に忠実に従うと、visibleなノードが全て繋がっていた場合は、サンプル点から一番近い２つについて操作する。つまり処理を分けるべき。TODO 関数化してくくり出す。
            if not self.checkAddConnectivity(sample_x, sample_y, visible_nodes):
                if not self.checkAddInterface(sample_x, sample_y, visible_nodes, rr, ox ,oy):
                    # TODO ここでやるかどうかも検討が必要。隣接する頂点群(visible_nodes)の1個ずつについて、ある1個のvisible_nodes以外の隣接する頂点とother_nodesとを接続してみて、そのうちの最も長い距離がdelta * stretch factor よりも長くなれば、直接つなぐ。
                    pass

        self.edges = self.get_unique_list(self.edges)
        return
        
    def plot_graph(self):
        print("edges len : {}".format(len(self.edges)))
        for edge in self.edges:
            plt.plot([self.sample_x[edge[0]], self.sample_x[edge[1]]], [self.sample_y[edge[0]], self.sample_y[edge[1]]])
        # plot
        plt.plot(self.sample_x, self.sample_y, "ok")


def main():
    print("{} start !".format(__file__))

    sx = 10.0
    sy = 10.0
    gx = 50.0
    gy = 50.0
    robot_size = 5.0

    ox = []
    oy = []

    for i in range(60):
        ox.append(i)
        oy.append(0.0)
    for i in range(60):
        ox.append(60.0)
        oy.append(i)
    for i in range(61):
        ox.append(i)
        oy.append(60.0)
    for i in range(61):
        ox.append(0.0)
        oy.append(i)
    for i in range(40):
        ox.append(20.0)
        oy.append(i)
    for i in range(40):
        ox.append(40)
        oy.append(60.0 - i)

    if show_animation:
        plt.plot(ox, oy, ".k")
        plt.plot(sx, sy, "^r")
        plt.plot(gx, gy, "^c")
        plt.grid(True)
        plt.axis("equal")

    spars_planner = MySpars()
    for i in range(1000):
        print("adding node ... {}".format(i))
        spars_planner.addNode(ox, oy, 2.0, 6.0)

    spars_planner.plot_graph()

    if show_animation:
        #plt.plot(rx, ry, "-r")
        plt.pause(0.001)
        plt.show()

if __name__ == '__main__':
    main()