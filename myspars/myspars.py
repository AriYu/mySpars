"""
This is my SPARS algorithm, which only implement visible method.
"""

from ctypes.wintypes import BOOL
import math
import numpy as np
import random
import matplotlib.pyplot as plt

from typing import Tuple
from typing import List

show_animation = True

class Edge:
    def __init__(self, to : int, w : float) -> None:
        self.to = to # 隣接頂点番号
        self.w = w # 重み

class Node:
    def __init__(self, x : float, y : float) -> None:
        self.x = x
        self.y = y
        self.edges : List[Edge] = []

    def addEdge(self, edge) -> None:
        self.edges.append(edge)

    def addEdge(self, to : int, cost : float) -> None:
        edge = Edge(to, cost)
        self.edges.append(edge)

    def __str__(self) -> str:
        return str(self.x) + ", " + str(self.y)

class MySpars:

    def __init__(self) -> None:
        self.graph : List[Node] = []

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

    def searchVisibleNode(self, qnew : Node, delta : float, ox : float, oy : float, rr : float) -> Tuple[list[int], List[Node]]:
        """
        qnew : 新しいサンプル点
        delta : visibleの範囲
        ox : 障害物のx座標のリスト
        oy : 障害物のy座標のリスト
        rr : 障害物とどれくらい近いと干渉と判定するかの距離

        ans1 : visible な node の index
        ans2 : visible な node の コピー
        """
        ans_index : list[int] = []
        ans_nodes : List[Node] = []

        # qnewからdeltaの距離にあるNodeを検索
        for i, node in enumerate(self.graph):
            dist = np.hypot(node.x - qnew.x, node.y - qnew.y)
            if (dist < delta):
                if not self.isCollisionPath(qnew, node, ox, oy, rr): # そのうち、qnewと接続できるものを検索
                    ans_index.append(i)
                    ans_nodes.append(node)

        return (ans_index, ans_nodes)

    def isCollisionPath(self, p0 : Node, p1 : Node, ox : list[float], oy : list[float], rr : float) -> BOOL:
        """
        p0 と p1 間を rr の長さずつ、チェックし、ox, oy と干渉していないかをチェックする。干渉していればTrueを返す。
        """
        dx = p1.x - p0.x
        dy = p1.y - p0.y
        yaw = math.atan2(dy, dx)
        d = np.hypot(dx, dy)

        D = rr
        n_step = round(d / D)

        x = p0.x
        y = p0.y

        for i in range(n_step):
            _, dist = self.searchNearestFromList(x, y, ox, oy)
            if dist <= rr:
                return True # collide
            x += D * math.cos(yaw)
            y += D * math.sin(yaw)
        
        return False # no collide           

    def isCollision(self, p : Node, ox : list[float], oy : list[float], rr : float) -> BOOL:
        """
        半径rr内にox, oy があれば、True（Collide）を返す。
        """
        _, min_dist = self.searchNearestFromList(p.x, p.y, ox, oy)
        if min_dist < rr: 
            return True # collide
        else:
            return False # no collide

    def checkAddCoverage(self, qnew : Node, visible_nodes : List[Node]) -> BOOL:
        """
        半径delta内に他のノードがなければグラフに追加してTrueを返す。
        """
        if len(visible_nodes) == 0: 
            self.graph.append(qnew)
            return True
        return False

    def checkAddConnectivity(self, qnew : Node, visible_nodes : List[Node]) -> BOOL:
        """
        本当は union find とか使ってグラフの頂点をコンポーネントとして管理しないと...
        本当は二つのノードが属しているコンポーネントが異なる場合にだけ、q_newを使って接続する。
        本当は二つのノードが属しているコンポーネントが異なっていたら無条件にq_newを頂点に追加し、それらとのエッジも追加する。

        そうすると、障害物と干渉するんじゃない？って思ったけど、そもそもvisible_nodesを探すときに、干渉判定までやっておくのである。
        つまり、visible node とはサンプル点と直接つなぐことができるっていうこと！！

        ここをちゃんと実装すれば、checkAddInterfaceの暫定のところは消せるはず。なぜなら、それらがすでに同じグループにあれば、ここでTrueを返して、
        checkAddInterfaceまで行かないからである。
        """
        return False    

    def checkAddInterface(self, qnew : Node, visible_index : list[int], visible_nodes : List[Node], ox : list[float], oy : list[float], rr : float) -> BOOL:
        """
        本当はvisible nodeのうち最も近い２つを選んでそれらがエッジを持たない時に追加する。ここでは最も近いノードではなくて配列の0, 1を使っている。でも近いは近いはず。
        """
        if len(visible_nodes) > 1 :
            # 直接つながる場合は挿入しない。
            if not self.isCollisionPath(visible_nodes[0], visible_nodes[1], ox, oy, rr):
                v0_to = [edge.to for edge in self.graph[visible_index[0]].edges]
                v1_to = [edge.to for edge in self.graph[visible_index[1]].edges]
                if visible_index[1] not in v0_to:
                    # visble_index[0]のtoにvisible_index[1]が無ければ追加する。
                    self.graph[visible_index[0]].addEdge(visible_index[1], 0.0)
                if visible_index[0] not in v1_to:
                    # visble_index[1]のtoにvisible_index[0]が無ければ追加する。
                    self.graph[visible_index[1]].addEdge(visible_index[0], 0.0)
                return True
            else:
                if not self.isCollisionPath(visible_nodes[0], qnew, ox, oy, rr):
                    if not self.isCollisionPath(visible_nodes[1], qnew, ox, oy, rr):
                        if len(visible_nodes) < 3: # 暫定でサンプルノードを追加すれば接続できる場合でも近くにたくさん(2個以上)ある場合は追加しない
                            # サンプルノードを経由すれば干渉しない時はサンプルノードを追加する。
                            qnew.addEdge(visible_index[0], 0.0) # qnew -> visible_node[0]
                            qnew.addEdge(visible_index[1], 0.0) # qnew -> visible_node[1]
                            self.graph.append(qnew)
                            self.graph[visible_index[0]].addEdge(len(self.graph)-1, 0.0) # visible_node[0] -> qnew
                            self.graph[visible_index[1]].addEdge(len(self.graph)-1, 0.0) # visible_node[1] -> qnew
                            return True

        return False

    def checkAddPath(self, visible_index : list[int], stretch_factor : float, ox : list[float], oy : list[float], rr : float):
        """
        qnewのvisible_nodesについてそれの2個先と直接繋いでみて、その距離がストレッチファクターを考慮しても短いかどうかをチェックする
        直接繋いだ方がよければ直接つなぐ。
        """
        for index in visible_index:
            base_node = self.graph[index]
            base_node_edges = [edge.to for edge in base_node.edges]
            adjacent_nodes = [self.graph[edge.to] for edge in base_node.edges]
            for adj_node in adjacent_nodes:
                for edge in adj_node.edges: # 隣接点のエッジについて
                    if edge.to not in base_node_edges: # 隣接点のエッジのtoがbase_nodeのtoに含まれていない、つまり隣接点の隣接点とbase_nodeが直接繋がっていない場合
                        dist_on_graph = (np.hypot(base_node.x - adj_node.x, base_node.y - adj_node.y) + np.hypot(adj_node.x - self.graph[edge.to].x, adj_node.y - self.graph[edge.to].y)) *  stretch_factor
                        dist_direct = np.hypot(base_node.x - self.graph[edge.to].x, base_node.y - self.graph[edge.to].y)
                        if (dist_on_graph > dist_direct): # 直接繋いだ方が短くて、
                            if not self.isCollisionPath(base_node, self.graph[edge.to], ox, oy, rr): # 干渉しないならば、
                                self.graph[index].addEdge(edge.to, 0)
                                self.graph[edge.to].addEdge(index, 0)

    @staticmethod
    def sampler(center : Node, x_max : float, x_min : float, y_max : float, y_min : float, delta : float) -> Node:
        """
        - center を中心に半径delta以内にサンプルする
        refer iPad notes; Sample Uniform Near and ompl::base::RealVectorStateSampler::sampleUniformNear
        """
        ans = Node(0, 0)
        ans.x = random.uniform(np.max([x_min, center.x - delta]), np.min([center.x + delta, x_max]))
        ans.y = random.uniform(np.max([y_min, center.y - delta]), np.min([center.y + delta, y_max]))
        return ans

    def findCloseRepresentatives(self, qnew : Node, qrep : int, delta : float, max_x : float, min_x : float, max_y : float, min_y : float, ox : list[float], oy : list[float], rr : float):
        """
        - qnewのrepresentiveと異なるrepresentiveが見つかったら返却リストに入れる
        - workに対するrepresentiveが見つからなかったら、workをNodeとして追加してbreakして終了
        """
        near_sample_points = 2 # same to dimention
        close_representative = []
        for i in range(near_sample_points):
            while True:
                work = self.sampler(qnew, max_x, min_x, max_y, min_y, delta)
                if not self.isCollision(work, ox, oy, rr):
                    break
            candidates_index, candidates = self.searchVisibleNode(work, delta, ox, oy, rr)
            representative = -1
            for i, node in enumerate(candidates):
                if self.isCollisionPath(work, node, ox, oy, rr):
                    representative = candidates_index[i]
                    break
            if representative is not -1 and representative != qrep:
                if not representative in close_representative:
                    close_representative.append(representative)
            else:
                self.graph.append(work)
        return close_representative

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

        is_sample_found = False
        qnew = Node(0, 0)

        for i in range(100): # 1個のノードを追加するのに100回までリトライする。
            # サンプル点を生成する
            qnew.x = (random.random() * (max_x - min_x)) + min_x
            qnew.y = (random.random() * (max_y - min_y)) + min_y

            if self.isCollision(qnew, ox, oy, rr):
                continue # 新しく生成したサンプル点が干渉していたら再度乱数生成
            else:
                is_sample_found = True
                break

        if not is_sample_found:
            # 100回の試行でサンプル点の候補が見つからなかった
            return

        # サンプル点に対してdelta内にあるvisible nodes を検索
        visible_index, visible_nodes = self.searchVisibleNode(qnew, delta, ox, oy, rr)
        if not self.checkAddCoverage(qnew, visible_nodes):
            # Github上の実装だと全てのvisibleなノードの組み合わせについて、同じコンポーネント（つまり接続されているか）をチェックして、同じコンポーネントにない同士をサンプル点を使って接続している。
            # ただし以下の実装では半径delta内に他のノードがあって、そのノード間を直接接続できれば接続する。
            # この場合、サンプルノードは追加しない。
            # ただし、ノード間をサンプルノードを経由することで接続できればサンプルノードも追加する。
            # ここは論文に忠実に従うと、visibleなノードが全て繋がっていた場合は、サンプル点から一番近い２つについて操作する。つまり処理を分けるべき。
            if not self.checkAddConnectivity(qnew, visible_nodes):
                if not self.checkAddInterface(qnew, visible_index, visible_nodes, ox ,oy, rr):
                    representatives = self.findCloseRepresentatives(qnew, visible_index[0], delta, max_x, min_x, max_y, min_y, ox, oy, rr)
                    if len(representatives) != 0:
                        # TODO update_pair_points()
                        pass
                    self.checkAddPath(visible_index, 0.6, ox, oy, rr)
                    for rep in representatives:
                        self.checkAddPath(rep, 0.6, ox, oy, rr)        
        return
        
    def plotGraph(self):
        num_of_edges = 0
        node_x = [node.x for node in self.graph]
        node_y = [node.y for node in self.graph]
        for i, node in enumerate(self.graph):
            num_of_edges += len(node.edges)
            print("[{}] : {}".format(i, len(node.edges)))
            for edge in node.edges:
                plt.plot([node.x, self.graph[edge.to].x], [node.y, self.graph[edge.to].y])
        #plt.plot(node_x, node_y, "ok")
        print("Num of Nodes : {}".format(len(self.graph)))
        print("Num of Edges : {}".format(num_of_edges))

def main():
    random.seed(1)
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
    for i in range(10000):
        print("adding node ... {}".format(i))
        spars_planner.addNode(ox, oy, 2.0, 4.0)

    spars_planner.plotGraph()

    if show_animation:
        #plt.plot(rx, ry, "-r")
        plt.pause(0.001)
        plt.show()

if __name__ == '__main__':
    main()