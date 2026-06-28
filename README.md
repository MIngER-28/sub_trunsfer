# 项目背景与目标
    在复杂的城市地铁管网中，最短距离（最少途经站）并不总是最优解。
    由于站内换乘通常需要较长等车时间，乘客往往更倾向于“尽量少换乘，在此基础上途径站尽量少”的路线。
    本项目以长沙地铁路线图为基础，输入起点和终点，系统将输出一条换乘次数最少的最优乘车路径。
# 核心数据结构设计
    地铁网络是一个典型的稀疏无向图。
    虽然站点总数众多，但每个站通常只与相邻的 2 到 4 个站点相连。
    因此，使用二维数组（邻矩阵）会造成极大的内存浪费，本项目采用 邻接表 
    (AdjacencyList) 进行图的存储。
# 核心结构体定义
## 边节点 (Edge)：
代表两站之间的一段轨道。不仅需要记录目标站点，还必须记录线路
号，这是判断是否发生换乘的核心依据。
## 顶点节点 (Station)：
代表地铁站，包含站点名称和指向相邻站点的链表头指针。

```c
// 边节点：记录连接关系与线路属性
typedef struct Edge {
    int to;             // 目标站点在数组中的索引
    int line;           // 属于几号线 (如 1, 2, 3)
    struct Edge* next;  // 指向下一条边的指针
} Edge;

// 站点节点：图的顶点
typedef struct {
    char name[50];      // 站点名称 (如 "五一广场")
    Edge* head;         // 邻接表头指针
} Station;
```
# 核心算法讲解
    带惩罚权重的 Dijkstra本项目的核心是对经典的 Dijkstra 
    短路径算法 进行了变种改造。
## 算法思路
    算法通过累加边权来寻找最短路。如果我们将
    邻两站的边权设为 1，求出的就是“最少途经站”路径。为了实
    “最少换乘优先”，我们引入了 “换乘惩罚权重 (Transfer
    Penalty)” 的概念。
### 基础代价
每乘坐一站，代价增加 1。
### 换乘代价
    当算法从站点 U 探索到站点 V 时，会检查当前边所在的线
    (current_line) 与到达站点 U 时乘坐的线路 (prev_lin[u]) 
    是否一致。如果一致，说明没有换乘，代价正常增加 1。
    如果不一致，说明发生了一次换乘。此时赋予一个极大的惩罚权重（
    如 1000）。总代价增加 1001。

    当 Dijkstra 在全图寻找总代价最小的路径时，
    它会自动避开产生 1000 惩罚的换乘行为，宁愿多坐几
    （代价增加几）也不愿换乘（代价增加一千）。只有当不换乘绝对
    法到达终点时，它才会接受这 1000 的惩罚。这就完美实现
    “最少换乘优先，其次最少停站”的规划逻辑。

    
### 第一部分：头文件与全局宏定义
```c
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#define MAX_STATIONS 200
#define INF 999999
#define TRANSFER_COST 1000
#define STATION_COST 1
```
### 第二部分：数据结构定义（图的邻接表）
```c
//代表站与站之间的一截轨道
typedef struct Edge {
    int to;
    int line;
    struct Edge* next;
} Edge;

//代表一个具体的地铁站
typedef struct {
    char name[50];
    Edge* head;
} Station;

Station network[MAX_STATIONS]; // 存储所有的地铁站
int num_stations = 0;          // 记录当前已经录入了多少个地铁站
```

### 第三部分：图的构建工具函数
1. 获取或创建站点ID
```c
int getStationId(const char* name) {
    // 遍历已有的站点，如果名字匹配，说明站点已存在，直接返回它的索引号
    for (int i = 0; i < num_stations; i++) {
        if (strcmp(network[i].name, name) == 0) return i;
    }
    // 如果循环结束没找到，说明是个新站。把它存入数组尾部。
    strcpy(network[num_stations].name, name); 
    network[num_stations].head = NULL; // 新站点目前还没有连接任何轨道，头指针设为空
    return num_stations++; // 返回新站的索引，并将总站点数加1
}
```
2. 添加相连的轨道（建边）
```C
void addEdge(const char* name1, const char* name2, int line) {
    int u = getStationId(name1);
    int v = getStationId(name2);

    // 建立 A -> B 的单向边
    Edge* e1 = (Edge*)malloc(sizeof(Edge)); 
    e1->to = v;
    e1->line = line;
    // 链表头插法
    e1->next = network[u].head;
    network[u].head = e1;

    // 建立 B -> A 的单向边 
    Edge* e2 = (Edge*)malloc(sizeof(Edge)); 
    e2->to = u; e2->line = line; e2->next = network[v].head;
    network[v].head = e2;
}
```
### 第四部分：核心寻路算法 (带惩罚的 Dijkstra)
1. 初始化战场
```C
void findPath(const char* start_name, const char* end_name) {
    int start = getStationId(start_name); 
    int end = getStationId(end_name);     

    int dist[MAX_STATIONS];
    int prev[MAX_STATIONS];
    int prev_line[MAX_STATIONS];
    bool visited[MAX_STATIONS];

    // 初始化：
    for (int i = 0; i < num_stations; i++) {
        dist[i] = INF; prev[i] = -1; prev_line[i] = -1; visited[i] = false;
    }
    dist[start] = 0;
```
2. Dijkstra 探索循环
```C    
// 外层循环：理论上最多遍历所有站点
    for (int i = 0; i < num_stations; i++) {
        int u = -1, min_dist = INF;
        
        // 寻找当前所有未访问站点中，代价(dist)最小的那一个，定为 u
        for (int j = 0; j < num_stations; j++) {
            if (!visited[j] && dist[j] < min_dist) {
                min_dist = dist[j];
                u = j;
            }
        }
        
        if (u == -1 || u == end) break; 
        visited[u] = true; 

        // 遍历站 u 连接的所有相邻轨道 (链表遍历)
        for (Edge* e = network[u].head; e != NULL; e = e->next) {
            int v = e->to; 
            int current_line = e->line;

            if (!visited[v]) { 
                int cost = STATION_COST;
                
                // 【核心逻辑】：判断是否换乘
                // 如果当前不是起点站，并且“去v的线路”和“来u的线路”不一样
                if (u != start && prev_line[u] != current_line) {
                    cost += TRANSFER_COST; 
                }

                // 如果 绕道u去v 的总代价，比之前发现的 去v的代价更小
                if (dist[u] + cost < dist[v]) {
                    dist[v] = dist[u] + cost;     
                    prev[v] = u;
                    prev_line[v] = current_line;
                }
            }
        }
    }
```
3. 路线回溯与输出
```C   
if (dist[end] == INF) { 
        printf("\n[Error] 未找到路线。\n");
        return;
    }

    int path[MAX_STATIONS], lines[MAX_STATIONS];
    int count = 0, curr = end; // 从终点开始往回找

    // 只要没回到起点（起点的 prev 是 -1），就往回倒推
    while (curr != -1) {
        path[count] = curr;       
        lines[count] = prev_line[curr]; 
        curr = prev[curr];
        count++;
    }

    // 因为是倒推的，所以要【倒序遍历】数组，才能正向打印出从起点到终点的路线
    for (int i = count - 2; i >= 0; i--) {
        int node = path[i];  
        int current_line = lines[i];   
        int next_line = (i > 0) ? lines[i-1] : -1; 

        printf("[ %s ]", network[node].name);

        if (next_line != -1 && next_line != current_line) {
            printf("  <-- 【 换乘至 %d 号线 】", next_line);
        }
        printf("\n");
    }
}
```
### 第五部分：主函数 (入口)
```C
int main() {
    buildChangshaGraph(); // 调用函数，把所有的地铁站和路线先录入到内存里
    
    char start[50], end[50];
    // ... 打印欢迎信息 ...
    
    printf("请输入起点站: ");
    scanf("%s", start); 
    printf("请输入终点站: ");
    scanf("%s", end);

    findPath(start, end);
    return 0;
}
```
