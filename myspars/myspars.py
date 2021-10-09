"""
This is my SPARS algorithm, which only implement visible method.
"""

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

class UnionFind(object):
    """
    Union find
    """
    def __init__(self):
        """
        初期化
        """
        self.par : dict[int, int] = {}
        self.siz : dict[int, int] = {}

    def add(self, index : int):
        self.par[index] = -1
        self.siz[index] = 1

    def root(self, x : int) -> int:
        """
        根を求める
        """
        if self.par[x] == -1:
            return x # x が根の場合は x を返す
        else:
            self.par[x] = self.root(self.par[x])
            return self.par[x]

    def issame(self, x : int, y : int) -> bool:
        """
        x と y が同じグループに属するかどうか（根が一致するかどうか）
        """
        return (self.root(x) == self.root(y))

    def unite(self, x : int, y : int) -> bool:
        """
        x を含むグループと y を含むグループとを併合する
        """
        # x と y をそれぞれ根まで移動する
        x = self.root(x)
        y = self.root(y)

        # すでに同じグループの時は何もしない
        if (x == y):
            return False

        # union by size (y側のサイズが小さくなるようにする)
        if (self.siz[x] < self.siz[y]):
            self.swap(x, y)

        # y を x の子とする
        self.par[y] = x
        self.siz[x] += self.siz[y]
        return True

    def size(self, x : int) -> int:
        """
        x を含むグループのサイズ
        """
        return self.siz[self.root(x)]

    @staticmethod
    def swap(x : int, y : int):
        """
        x と y を入れ替える
        """
        tmp = x
        x = y
        y = tmp
        return

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

class CollisionChecker(object):
    """
    Collision checker for path planning
    """
    def __init__(self,  ox : list[float], oy :list[float], rr : float) -> None:
        """
        ox : The list for x of obstacles
        oy : The list for y of obstacles
        rr : The thresold to determine collide with obstacles
        """
        self.ox = ox
        self.oy = oy
        self.rr = rr
        self.max_x : float = max(ox)
        self.min_x : float = min(ox)
        self.max_y : float = max(oy)
        self.min_y : float = min(oy)

    def isCollision(self, p : Node) -> bool:
        """
        If there are ox or oy in radius rr, return True (colliding)
        """
        _, min_dist = searchNearestFromList(p.x, p.y, self.ox, self.oy)
        if min_dist < self.rr: 
            return True # collide
        else:
            return False # no collide


    def isCollisionPath(self, p0 : Node, p1 : Node) -> bool:
        """
        p0, p1 : the vertexes which compromize a line
        Check collision between the line and ox, oy by rr.
        If there is collision, return True.
        """
        dx = p1.x - p0.x
        dy = p1.y - p0.y
        yaw = math.atan2(dy, dx)
        d = np.hypot(dx, dy)

        D = self.rr
        n_step = round(d / D)

        x = p0.x
        y = p0.y

        for i in range(n_step):
            _, dist = searchNearestFromList(x, y, self.ox, self.oy)
            if dist <= self.rr:
                return True # collide
            x += D * math.cos(yaw)
            y += D * math.sin(yaw)
        
        return False # no collide           

class MySpars:

    def __init__(self, collision_checker : CollisionChecker) -> None:
        self.graph : List[Node] = []
        self.union_find: UnionFind = UnionFind()
        self.collision_checker = collision_checker

    def searchVisibleNode(self, qnew : Node, delta : float) -> Tuple[list[int], List[Node]]:
        """
        qnew : 新しいサンプル点
        delta : visibleの範囲

        ans1 : visible な node の index
        ans2 : visible な node の コピー

        qnewに対する visible node というのは干渉チェックまで終わっているnodeなのでqnewを経由すれば、干渉チェックなしに接続できる。
        """
        ans_index : list[int] = []
        ans_nodes : List[Node] = []

        # qnewからdeltaの距離にあるNodeを検索
        for i, node in enumerate(self.graph):
            dist = np.hypot(node.x - qnew.x, node.y - qnew.y)
            if (dist < delta):
                if not self.collision_checker.isCollisionPath(qnew, node): # そのうち、qnewと接続できるものを検索
                    ans_index.append(i)
                    ans_nodes.append(node)

        return (ans_index, ans_nodes)

    def checkAddCoverage(self, qnew : Node, visible_nodes : List[Node]) -> bool:
        """
        半径delta内に他のノードがなければグラフに追加してTrueを返す。
        """
        if len(visible_nodes) == 0: 
            self.graph.append(qnew)
            self.union_find.add(len(self.graph) - 1)
            return True
        return False

    def checkAddConnectivity(self, qnew : Node, visible_index : list[int]) -> bool:
        """
        qnewに対するvisible node に対して、ある visible node が他の visible node と同じグラフにあるかをチェックし、
        異なるグラフ上にある visible node があれば、qnewを追加して接続する。
        もう一つ重要な役割がある。それは、ここで visible node が同じグラフ上であれば、Falseを返すということ。
        つまり、ここで繋がるグラフが異なるグラフである場合に、この後の、checkAddInterfaceを呼ぶことになる。
        これにより、無限にショートカットパスを作らないようになる。（ショートカットパスを作ろうとするのは異なるグラフを接続した時だけ。）

        visible node が 1個しかない場合は、接続しない。なぜなら、飛び出るようなグラフになってしまうため。
        """
        if len(visible_index) > 1:
            disconnected : list[int] = []
            for i in range (len(visible_index)):
                for j in range(i+1, len(visible_index), 1):
                    if not self.union_find.issame(visible_index[i], visible_index[j]):
                        disconnected.append(visible_index[i])
                        disconnected.append(visible_index[j])
            if len(disconnected) > 0:
                self.graph.append(qnew)
                self.union_find.add(len(self.graph) - 1)
                for index in disconnected:
                    self.graph[len(self.graph) - 1].addEdge(index, 0)
                    self.union_find.unite(len(self.graph) - 1, index)
                return True
        return False    

    def checkAddInterface(self, qnew : Node, visible_index : list[int], visible_nodes : List[Node]) -> bool:
        """
        本当はvisible nodeのうち最も近い２つを選んでそれらがエッジを持たない時に追加する。ここでは最も近いノードではなくて配列の0, 1を使っている。でも近いは近いはず。
        """
        if len(visible_nodes) > 1 :
            # 直接つながる場合は挿入しない。
            if not self.collision_checker.isCollisionPath(visible_nodes[0], visible_nodes[1]):
                v0_to = [edge.to for edge in self.graph[visible_index[0]].edges]
                v1_to = [edge.to for edge in self.graph[visible_index[1]].edges]
                if visible_index[1] not in v0_to:
                    # visble_index[0]のtoにvisible_index[1]が無ければ追加する。
                    self.graph[visible_index[0]].addEdge(visible_index[1], 0.0)
                    self.union_find.unite(visible_index[1], visible_index[0])
                if visible_index[0] not in v1_to:
                    # visble_index[1]のtoにvisible_index[0]が無ければ追加する。
                    self.graph[visible_index[1]].addEdge(visible_index[0], 0.0)
                    self.union_find.unite(visible_index[0], visible_index[1])
                return True
            else:
                if not self.collision_checker.isCollisionPath(visible_nodes[0], qnew):
                    if not self.collision_checker.isCollisionPath(visible_nodes[1], qnew):
                        #if len(visible_nodes) < 3: # 暫定でサンプルノードを追加すれば接続できる場合でも近くにたくさん(2個以上)ある場合は追加しない
                        # サンプルノードを経由すれば干渉しない時はサンプルノードを追加する。
                        qnew.addEdge(visible_index[0], 0.0) # qnew -> visible_node[0]
                        qnew.addEdge(visible_index[1], 0.0) # qnew -> visible_node[1]
                        self.graph.append(qnew)
                        self.union_find.add(len(self.graph) - 1)
                        self.union_find.unite(len(self.graph) - 1, visible_index[0])
                        self.union_find.unite(len(self.graph) - 1, visible_index[1])
                        self.graph[visible_index[0]].addEdge(len(self.graph)-1, 0.0) # visible_node[0] -> qnew
                        self.graph[visible_index[1]].addEdge(len(self.graph)-1, 0.0) # visible_node[1] -> qnew
                        return True

        return False

    def checkAddPath(self, visible_index : int, stretch_factor : float):
        """
        qnewのvisible_nodesについてそれの2個先と直接繋いでみて、その距離がストレッチファクターを考慮しても短いかどうかをチェックする
        直接繋いだ方がよければ直接つなぐ。
        """
        base_node = self.graph[visible_index]
        base_node_edges = [edge.to for edge in base_node.edges]
        adjacent_nodes = [self.graph[edge.to] for edge in base_node.edges]
        for adj_node in adjacent_nodes:
            for edge in adj_node.edges: # 隣接点のエッジについて
                if edge.to not in base_node_edges: # 隣接点のエッジのtoがbase_nodeのtoに含まれていない、つまり隣接点の隣接点とbase_nodeが直接繋がっていない場合
                    dist_on_graph = (np.hypot(base_node.x - adj_node.x, base_node.y - adj_node.y) + np.hypot(adj_node.x - self.graph[edge.to].x, adj_node.y - self.graph[edge.to].y)) *  stretch_factor
                    dist_direct = np.hypot(base_node.x - self.graph[edge.to].x, base_node.y - self.graph[edge.to].y)
                    if (dist_on_graph > dist_direct): # 直接繋いだ方が短くて、
                        if not self.collision_checker.isCollisionPath(base_node, self.graph[edge.to]): # 干渉しないならば、
                            self.graph[visible_index].addEdge(edge.to, 0)
                            self.graph[edge.to].addEdge(visible_index, 0)
                            self.union_find.unite(visible_index, edge.to)

    def sampler(self, center : Node, delta : float) -> Node:
        """
        - center を中心に半径delta以内にサンプルする
        refer iPad notes; Sample Uniform Near and ompl::base::RealVectorStateSampler::sampleUniformNear
        """
        ans = Node(0, 0)
        ans.x = random.uniform(np.max([self.collision_checker.min_x, center.x - delta]), np.min([center.x + delta, self.collision_checker.max_x]))
        ans.y = random.uniform(np.max([self.collision_checker.min_y, center.y - delta]), np.min([center.y + delta, self.collision_checker.max_y]))
        return ans

    def findCloseRepresentatives(self, qnew : Node, qrep : int, delta : float):
        """
        - qnewのrepresentiveと異なるrepresentiveが見つかったら返却リストに入れる
        - workに対するrepresentiveが見つからなかったら、workをNodeとして追加してbreakして終了
        """
        near_sample_points = 2 # same to dimention
        close_representative = []
        for i in range(near_sample_points):
            while True:
                work = self.sampler(qnew, delta)
                if not self.collision_checker.isCollision(work):
                    break
            candidates_index, candidates = self.searchVisibleNode(work, delta)
            representative = -1
            for i, node in enumerate(candidates):
                if not self.collision_checker.isCollisionPath(work, node):
                    representative = candidates_index[i]
                    break
            if representative != -1 and representative != qrep:
                if not representative in close_representative:
                    close_representative.append(representative)
            else:
                self.graph.append(work)
                self.union_find.add(len(self.graph) - 1)
        return close_representative

    def addNode(self, delta : float) -> None:
        """
        delta : 他のサンプル点との距離
        """

        is_sample_found = False
        qnew = Node(0, 0)

        for i in range(100): # 1個のノードを追加するのに100回までリトライする。
            # サンプル点を生成する
            qnew.x = (random.random() * (self.collision_checker.max_x - self.collision_checker.min_x)) + self.collision_checker.min_x
            qnew.y = (random.random() * (self.collision_checker.max_y - self.collision_checker.min_y)) + self.collision_checker.min_y

            if self.collision_checker.isCollision(qnew):
                continue # 新しく生成したサンプル点が干渉していたら再度乱数生成
            else:
                is_sample_found = True
                break

        if not is_sample_found:
            # 100回の試行でサンプル点の候補が見つからなかった
            return

        # サンプル点に対してdelta内にあるvisible nodes を検索
        visible_index, visible_nodes = self.searchVisibleNode(qnew, delta)
        if not self.checkAddCoverage(qnew, visible_nodes):
            if not self.checkAddConnectivity(qnew, visible_index):
                if not self.checkAddInterface(qnew, visible_index, visible_nodes):
                    representatives = self.findCloseRepresentatives(qnew, visible_index[0], delta)
                    if len(representatives) != 0:
                        # TODO update_pair_points()
                        pass
                    self.checkAddPath(visible_index[0], 0.6)
                    for rep in representatives:
                        self.checkAddPath(rep, 0.6)        
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

    collision_checker = CollisionChecker(ox, oy, 2.0)
    spars_planner = MySpars(collision_checker)

    for i in range(5000):
        print("adding node ... {}".format(i))
        spars_planner.addNode(4.0)

    spars_planner.plotGraph()

    if show_animation:
        #plt.plot(rx, ry, "-r")
        plt.pause(0.001)
        plt.show()

if __name__ == '__main__':
    main()