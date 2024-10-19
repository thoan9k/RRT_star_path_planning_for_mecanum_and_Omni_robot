import random
import math
import time

import pygame


class RRTmap:
    def __init__(self, start, goal, mapdimensions, obsdim, obsnum, Mapwindowname='RRT path planning'):
        self.start = start
        self.goal = goal
        self.mapdimensions = mapdimensions
        self.maph, self.mapw = self.mapdimensions

        self.Mapwindowname = Mapwindowname
        pygame.display.set_caption(self.Mapwindowname)
        self.map = pygame.display.set_mode((self.mapw, self.maph))
        self.map.fill((255, 255, 255))
        self.nodeRadian = 2
        self.nodeThickness = 0
        self.edgeThickness = 1

        self.obstacles = []
        self.obsdim = obsdim
        self.obsnum = obsnum

        self.grey = (70, 70, 70)
        self.blue = (0, 0, 255)
        self.green = (0, 255, 0)
        self.red = (255, 0, 0)
        self.white = (255, 255, 255)

    def drawmap(self, obstacles):
        # vẽ điểm bắt đầu và điểm đích
        pygame.draw.circle(self.map, self.green, self.start, self.nodeRadian + 10, 0)
        pygame.draw.circle(self.map, self.red, self.goal, self.nodeRadian + 20, 1)
        # Vẽ các trướng ngại vật hình vuông
        self.drawObs(obstacles)

    def drawpath(self, coords):
        for coord in coords:
            pygame.draw.circle(self.map, self.red, coord, self.nodeRadian + 3, self.nodeThickness)

    def drawObs(self, obstacles):
        obstacleslist = obstacles.copy()
        while len(obstacleslist) > 0:  # vẽ các hình chữ nhật bên trong map đã tạo(nền trắng)
            obstacle = obstacleslist.pop(0)
            pygame.draw.rect(self.map, self.grey, obstacle)


class RRTgraph:
    def __init__(self, start, goal, mapdimensions, obsdim, obsnum, speed=0):
        (x, y) = start
        self.start = start
        self.goal = goal
        self.mapdimensions = mapdimensions
        self.maph, self.mapw = self.mapdimensions
        self.x = []
        self.y = []
        self.parent = []
        self.cost = []
        self.cost_2 = 0
        self.speed = speed
        # initialize the tree
        self.x.append(x)
        self.y.append(y)
        self.parent.append(0)
        self.cost.append(0)
        # the obstacles
        self.obstacles = []
        self.obsdim = obsdim
        self.obsnum = obsnum

        # path
        self.goalstate = None
        self.goalFlag = False
        self.path = []
        self.error = False

    def makerandomRect(self):
        uppercornerx = int(random.uniform(0, self.mapw - self.obsdim))
        uppercornery = int(random.uniform(0, self.maph - self.obsdim))
        return uppercornerx, uppercornery

    def getTrueObs(self, obs, size=-30):
        TOBS = []
        for ob in obs:
            TOBS.append(ob.inflate(size, size))
        return TOBS

    def reset(self, all_=False):
        if not all_:
            n = len(self.x) - 1
            self.x.pop(n)
            self.y.pop(n)
            self.parent.pop(n)
            self.cost.pop(n)
        else:
            self.x = []
            self.y = []
            self.parent = []
            self.cost = []
            (x, y) = self.start
            # initialize the tree
            self.x.append(x)
            self.y.append(y)
            self.parent.append(0)
            self.cost.append(0)


    def makeObs(self):

        obs = []
        for i in range(0, self.obsnum):
            rect = None
            startgoalcol = True
            while startgoalcol:
                upper = self.makerandomRect()
                rect = pygame.Rect(upper, (self.obsdim,     self.obsdim))
                if rect.collidepoint(self.start) or rect.collidepoint(self.goal):
                    startgoalcol = True
                else:
                    startgoalcol = False
            obs.append(rect)
        self.obstacles = obs.copy()
        return obs

    def add_node(self, n, x, y):
        self.x.insert(n, x)
        self.y.append(y)

    def remove_node(self, n):
        self.x.pop(n)
        self.y.pop(n)

    def add_edge(self, parent, child):
        self.parent.insert(child, parent)

    def remove_edge(self, n):
        self.parent.pop(n)

    def number_of_nodes(self):
        return len(self.x)

    def distance(self, n1, n2):
        (x1, y1) = (self.x[n1], self.y[n1])
        (x2, y2) = (self.x[n2], self.y[n2])
        dx = (float(x1) - float(x2)) ** 2
        dy = (float(y1) - float(y2)) ** 2
        return (dx + dy) ** 0.5

    def waypoints2path(self, num=5):
        oldpath = self.getPathcoords()
        path = []
        for i in range(0, len(self.path) - 1):
            # print(i)
            if i >= len(self.path):
                break
            x1, y1 = oldpath[i]
            x2, y2 = oldpath[i + 1]
            # print('---------')
            # print((x1, y1), (x2, y2))
            for i in range(0, num):
                u = i / num
                x = int(x2 * u + x1 * (1 - u))
                y = int(y2 * u + y1 * (1 - u))
                path.append((x, y))
                # print((x, y))

        return path
    def sample_envir(self):
        x = int(random.uniform(0, self.mapw))
        y = int(random.uniform(0, self.maph))
        return x, y

    def nearest(self, n):  # tìm node gần nhất
        dmin = self.distance(0, n)
        nnear = 0  # Lần đầu tiên là điểm thứ 0 -điểm start
        for i in range(0, n):
            if self.distance(i, n) < dmin:
                dmin = self.distance(i, n)
                nnear = i
        return nnear

    def isFree(self):
        n = self.number_of_nodes() - 1
        (x, y) = (self.x[n], self.y[n])
        obstacles = self.obstacles.copy()
        while len(obstacles) > 0:
            rect = obstacles.pop(0)
            if rect.collidepoint(x, y):
                self.remove_node(n)
                return False
        return True

    def crossObstacle_forstep(self, x1, x2, y1, y2, step_max=35):

        obs = self.obstacles.copy()
        dx = float(x2) - float(x1)
        dy = float(y2) - float(y1)
        theta = math.atan2(dy, dx)
        x, y = int(x1 + step_max * math.cos(theta)), int(y1 + step_max * math.sin(theta))

        while len(obs) > 0:
            rect = obs.pop(0)
            for i in range(0, 101):
                u = i / 100
                x_ = x1 * u + x * (1 - u)
                y_ = y1 * u + y * (1 - u)
                if rect.collidepoint(x_, y_):
                    return True
        return False

    def crossObstacle(self, x1, x2, y1, y2):
        obs = self.obstacles.copy()

        while len(obs) > 0:
            rect = obs.pop(0)
            for i in range(0, 101):
                u = i / 100
                x_ = x1 * u + x2 * (1 - u)
                y_ = y1 * u + y2 * (1 - u)
                if rect.collidepoint(x_, y_):
                    return True
        return False

    def connect(self, n1, n2):
        x1, y1 = self.x[n1], self.y[n1]
        x2, y2 = self.x[n2], self.y[n2]
        if self.crossObstacle_forstep(x1, x2, y1, y2):
            self.remove_node(n2)
            return False
        else:
            self.add_edge(n1, n2)
            cost_ = self.cost[n1] + self.distance(n1, n2)
            self.cost.append(cost_)
            return True

    def step(self, nnear, nrand, step_max=35):
        d = self.distance(nnear, nrand)
        x1, y1 = self.x[nnear], self.y[nnear]
        x2, y2 = self.x[nrand], self.y[nrand]
        # nếu d < stepmax thì đã thêm node ở phía trước rồi
        if d > step_max:

            dx = float(x2) - float(x1)
            dy = float(y2) - float(y1)
            theta = math.atan2(dy, dx)
            self.remove_node(nrand)  # xoá node này do lớn hơn step (xoá x và y)
            x, y = int(x1 + step_max * math.cos(theta)), int(y1 + step_max * math.sin(theta))
            if abs(x - self.goal[0]) < step_max and abs(y - self.goal[1]) < step_max:
                if self.crossObstacle(x, self.goal[0], y, self.goal[1]):
                    return
                self.add_node(nrand, self.goal[0], self.goal[1])
                self.goalstate = nrand
                self.goalFlag = True

            else:
                self.add_node(nrand, x, y)

    def path_to_goal(self):

        if self.goalFlag:
            self.path = []
            self.path.append(self.goalstate)
            time.sleep(0.02)
            # tìm node cha của điểm goal
            newPos = self.parent[self.goalstate]  # vị trí node cha
            time.sleep(0.02)
            # vòng lặp tìm node cha cho đến khi đến điểm start
            while newPos != 0:
                self.path.append(newPos)
                newPos = self.parent[newPos]
            self.path.append(0)
        return self.goalFlag

    def getPathcoords(self):
        pathcoords = []
        for node in self.path:
            # if len(self.x) < node:
            #     continue
            (x, y) = self.x[node], self.y[node]
            pathcoords.append((x, y))

        return pathcoords

    def getfinalcost(self):
        final_cost = 0.0
        for i in range(0, len(self.path) - 1):
            final_cost += self.distance(self.path[i], self.path[i + 1])
        if final_cost != 0:
            self.cost_2 = final_cost
        return final_cost

    def bias(self, ngoal):
        n = self.number_of_nodes()
        self.add_node(n, ngoal[0], ngoal[1])
        nnear = self.nearest(n)
        self.step(nnear, n)
        self.connect(nnear, n)
        return self.x, self.y, self.parent

    def expand(self):
        n = self.number_of_nodes()
        x, y = self.sample_envir()
        self.add_node(n, x, y)
        if self.isFree():
            xnearest = self.nearest(n)
            self.step(xnearest, n)
            self.connect(xnearest, n)

        return self.x, self.y, self.parent

    # def cost(self, n):
    #     ninit = 0
    #     n = n
    #     parent = self.parent[n]
    #     c = 0
    #     while n is not ninit:
    #         c = c + self.distance(n, parent)
    #         n = parent
    #         if n is not ninit:
    #             parent = self.parent[n]
    #     return c

    def rewire(self, map_, new_node, neighbors):
        if not neighbors:
            return

        # STEP 1: Tìm neighbor có chi phí nhỏ nhất đến new_node
        cost_min = float('inf')  # Khởi tạo với giá trị vô cực
        best_neighbor = None
        x_new, y_new = self.x[new_node], self.y[new_node]

        # Duyệt qua các neighbors để tìm đường đi có chi phí nhỏ nhất
        for i in neighbors:
            x_neighbor, y_neighbor = self.x[i], self.y[i]
            if not self.crossObstacle(x_new, x_neighbor, y_new, y_neighbor):
                new_cost = self.cost[i] + self.distance(new_node, i)
                if new_cost < cost_min:
                    cost_min = new_cost
                    best_neighbor = i  # Lưu lại neighbor tốt nhất

        if best_neighbor is not None:
            # xoá đường line cũ tới nốt mới
            pygame.draw.line(map_, (255, 255, 255), (self.x[-1], self.y[-1]),
                             (self.x[self.parent[-1]], self.y[self.parent[-1]]), 1)
            time.sleep(self.speed)
            # Cập nhật parent của new_node
            self.parent[new_node] = best_neighbor
            self.cost[new_node] = cost_min
            # Vẽ đường nối từ best_neighbor đến new_node
            pygame.draw.line(map_, (0, 0, 255), (self.x[best_neighbor], self.y[best_neighbor]), (x_new, y_new), 1)
            time.sleep(self.speed)
            pygame.display.update()
        # STEP 2: Tái cấu trúc cây
        for i in neighbors:
            # Tính toán chi phí mới nếu đi qua new_node đến neighbor i
            if i == best_neighbor:
                continue
            x_neighbor, y_neighbor = self.x[i], self.y[i]
            if not self.crossObstacle(x_new, x_neighbor, y_new, y_neighbor):
                new_cost = self.cost[new_node] + self.distance(new_node, i)
                # Nếu chi phí mới nhỏ hơn chi phí hiện tại của các neighbor, cập nhật parent
                if new_cost < self.cost[i]:
                    # xoá đường kết nối neighbor cũ
                    pygame.draw.line(map_, (255, 255, 255), (x_neighbor, y_neighbor),
                                     (self.x[self.parent[i]], self.y[self.parent[i]]), 1)
                    time.sleep(self.speed)
                    # kết nối neighbor đó với new_node
                    self.parent[i] = new_node
                    self.cost[i] = new_cost
                    # Vẽ đường nối từ new_node đến neighbor i
                    pygame.draw.line(map_, (0, 0, 255), (x_new, y_new), (x_neighbor, y_neighbor), 1)
                    time.sleep(self.speed)
                    pygame.display.update()

    # Tìm kiếm các node lân cận
    def near_neighbors(self, new_node, dmax=50):
        # # Số chiều không gian (2D)
        # d = 2
        #
        # # Hằng số gamma ban đầu có thể thử là tỷ lệ với kích thước bản đồ
        # gamma = 100  # Có thể thử từ 50-200 tùy vào kết quả

        #
        # radius = min(gamma * (math.log(new_node) / new_node) ** (1 / d), dmax)
        neighbors = []  # tìm lại neighbor khi gọi lại
        for i in range(len(self.x)):  # quét tất cả các node
            if self.distance(new_node, i) < dmax:
                neighbors.append(i)
        return neighbors
