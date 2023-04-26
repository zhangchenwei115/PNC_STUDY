import matplotlib.pyplot as plt
import math
show_animation = True
class Dijkstra:
    def __init__(self,ox,oy, resolution, robot_radius) -> None:
        self.min_x = None
        self.max_x = None
        self.min_y = None
        self.max_y = None
        self.x_width = None
        self.y_width = None
        self.resolution = resolution
        self.robot_radius = robot_radius        
        self.obs_map = None
        self.pengzhangx = []
        self.pengzhangy = []
        self.caculate_obs_map(ox,oy)
        
    def caculate_obs_map(self,ox,oy):
        self.min_x = min(ox)
        self.max_x = max(ox)
        self.min_y = min(oy)
        self.max_y = max(oy)
        print("min_x:", self.min_x)
        print("min_y:", self.min_y)
        print("max_x:", self.max_x)
        print("max_y:", self.max_y)        
        self.x_width = round((self.max_x - self.min_x) / self.resolution) 
        self.y_width = round((self.max_y - self.min_y) / self.resolution) 
        print("x_width:", self.x_width)
        print("y_width:", self.y_width)
        
        "障碍物膨胀，也就是需要把所有的node设置为false，因为机器人有步长。所以要障碍物膨胀，步长内的都设置成true"
        '表示不能穿过'
        self.obs_map = [[False for _ in range (self.x_width)] for _ in range (self.y_width)]
        
        for i in range (self.x_width):
            for j in range (self.y_width):
                for iox, ioy in zip(ox,oy):
                    iox = self.calc_xy_index(iox,self.min_x)
                    ioy = self.calc_xy_index(ioy,self.min_y)
                    d = math.hypot((i-iox),(j-ioy))
                    if d <= self.robot_radius:
                        self.obs_map[i][j] = True
                        self.pengzhangx.append(self.calc_xypositon(i,self.min_x))
                        self.pengzhangy.append(self.calc_xypositon(j,self.min_y))
                        break
                
        
    #index to xy
    def calc_xypositon(self,index,min):
        return index * self.resolution + min
    ##xy to index
    
        
    class Node: 
        def __init__(self,x,y,cost,parent_index) -> None:
            self.x = x
            self.y = y
            self.cost = cost
            self.parent_index = parent_index
        def __str__(self) -> str:
            return self.x + "," + self.y + "," +self.cost + "," + self.parent_index
    #
    def calc_xy_index(self,position,min):
        return round((position - min)/self.resolution)
     
    def calc_index(self,node):
        return node.y * self.x_width + node.x
    
    #检查是否在地图内以及是否是障碍物
    def verify_node(self, node):
        px = self.calc_position(node.x, self.min_x)
        py = self.calc_position(node.y, self.min_y)

        if px < self.min_x:
            return False
        if py < self.min_y:
            return False
        if px >= self.max_x:
            return False
        if py >= self.max_y:
            return False

        if self.obs_map[node.x][node.y]:
            return False

        return True

    #定义运动模型
    def get_motion_model(self):
        # dx, dy, cost
        motion = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1],
                  [-1, -1, math.sqrt(2)],
                  [-1, 1, math.sqrt(2)],
                  [1, -1, math.sqrt(2)],
                  [1, 1, math.sqrt(2)]]

        return motion
            
    def planning(self,sx,sy,gx,gy):
        """
        dijskra path search
        
        input:
            self.sx start x position [m]
            self.sy start y position [m]
            self.gx goal x position [m]
            self.gy goal y position [m]
        output:
            rx: x postion list of the final path;
            ry; y position list of the final path;
            
        """
        start_node = self.Node(self.calc_xy_index(sx,self.min_x),
                               self.calc_xy_index(sy,self.min_y),0,-1)
        goal_node =  self.Node(self.calc_xy_index(gx,self.min_x),
                               self.calc_xy_index(gy,self.min_y),0,-1)
        #创建两个dic，一个是open，一个是close，未被选中的放入dic，被选中的放入close
        open_set = dict()
        close_set = dict()
        open_set[self.calc_index(start_node)] = start_node
        while True:
            #找到openset里cost最小的点作为当前点
            print("size of openset",len(open_set))
            current_index = min(open_set,key=lambda o:open_set[o].cost)
            current_node = open_set[current_index]
            
            if show_animation:  #把当前点画出来
                plt.plot(self.calc_xypositon(current_node.x,self.min_x),
                         self.calc_xypositon(current_node.y,self.min_y), "xc")
           
                # for stopping simulation with the esc key.
                plt.gcf().canvas.mpl_connect(
                    'key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])
                if len(close_set.keys()) % 10 == 0:
                    plt.pause(0.001)
                    
            #判断退出条件
            if current_node.x == goal_node.x and current_node.y == goal_node.y:
                print("Find goal")
                goal_node.parent_index = current_node.parent_index
                goal_node.cost = current_node.cost
                break
            
            #将最小cost的点取出来放入close，并且从openset中删除
            del open_set[current_index]
            close_set[current_index] = current_node
            
            #开始扩展，扩展的方向是8个方向
            for i in range(8):
                motion = self.get_motion_model()[i]
                next_node = self.Node(current_node.x + motion[0],
                                      current_node.y + motion[1],
                                      current_node.cost + motion[2],
                                      current_index)
                next_node_id = self.calc_index(next_node)
                #因为是8个方向全部扩展，所以要判断是否在地图内，是否是障碍物，是否已经在close里
                if next_node_id in close_set:
                    continue
                if not self.verify_node(next_node):
                    continue
                #如果不在open里，就把这个点放入open，如果在的话，判读cost，如果小于原来的cost，就更新cost
                if next_node_id not in open_set:
                    open_set[next_node_id] = next_node
                else:
                    if open_set[next_node_id].cost >= next_node.cost:
                        open_set[next_node_id] = next_node
                        
            #回溯路径
        rx, ry = self.calc_final_path(goal_node, close_set)
        return rx, ry

    def calc_final_path(self, goal_node, closed_set):
        # generate final course
        rx, ry = [self.calc_position(goal_node.x, self.min_x)], [
            self.calc_position(goal_node.y, self.min_y)]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(self.calc_position(n.x, self.min_x))
            ry.append(self.calc_position(n.y, self.min_y))
            parent_index = n.parent_index

        return rx, ry

    def calc_position(self, index, minp):
        pos = index * self.resolution + minp
        return pos   
 
def main():
    #先画图
    #start and gooal postition
    sx = -0.0
    sy = -0.0
    gx = 50.0
    gy = 50.0
    grid_size = 2
    robot_radius = 1

    #set obstacle postion
    ox, oy = [], []
    for i in range(-10, 60):
        ox.append(i)
        oy.append(-10.0)
    for i in range(-10, 60):
        ox.append(60.0)
        oy.append(i)
    for i in range(-10, 61):
        ox.append(i)
        oy.append(60.0)
    for i in range(-10, 61):
        ox.append(-10.0)
        oy.append(i)
    for i in range(-10, 40):
        ox.append(20.0)
        oy.append(i)
    for i in range(0, 40):
        ox.append(40.0)
        oy.append(60.0 - i)
    dijkstra = Dijkstra(ox,oy,grid_size,robot_radius)
    rx,ry = dijkstra.planning(sx,sy,gx,gy)
    if show_animation:  # pragma: no cover
        plt.plot(ox, oy,".k")
        plt.plot(sx, sy, "og")
        plt.plot(gx, gy, "xb")
        plt.plot(dijkstra.pengzhangx,dijkstra.pengzhangy,".k")
        plt.grid(True)
        plt.axis("equal")
    if show_animation:  # pragma: no cover
        plt.plot(rx, ry, "-r")
        plt.pause(0.01)
        plt.show()

if __name__=="__main__":
    main()
