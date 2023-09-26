using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Threading;
using System.Diagnostics;
using OpenCvSharp;
using Priority_Queue;

//利用A*算法以及opencvsharp实现坦克坦克大战，
//其中A*算法用于敌人路径规划实现AI，
//opencvsharp实现图像的绘制，最后还实现那了地图自动生成。
namespace Aplanning
{
    class AStarPathfinding
    {
        // 定义地图尺寸
        private int width;
        private int height;

        // 定义地图，0表示可通行，1表示障碍物
        private int[,] map;

        //地图图像化
        Mat mapimage;
        //地图显示比例大小
        int scale = 4;
        //地图显示窗口
        string wind = "A*路径规划";

        //地图初始化数据
        private List<(int x, int y)> points = new List<(int x, int y)>();

        public AStarPathfinding()
        {
            Map mp = new Map();
            this.map = mp.map;
            this.points = mp.pointlist;
            this.width = map.GetLength(1);
            this.height = map.GetLength(0);
            this.mapimage= new Mat(new Size(width*scale, height*scale), MatType.CV_8UC3, new Scalar(0, 0, 0));
            show_init();
            show_route();
        }
        public AStarPathfinding(int[,] map)
        {
            this.map = map;
            this.width = map.GetLength(1);
            this.height = map.GetLength(0);
        }

        //定义地图类

        public class Map
        {
            public int[,] map = new int[200, 200];
            public int init_point = 10000;
            public List<(int x, int y)> pointlist = new List<(int x, int y)>();
            public Map()
            {
                Init();
            }
            public void Init()
            {
                List<(int x,int y)> start_point = new List<(int,int)> { 
                    (0,0),
                    (1,0),
                    (0,1),
                    (1,1),
                    (98,98),
                    (99,98),
                    (98,99),
                    (99,99),
                    (199,199),
                    (799,799),
                    (1399,799),
                    (499,499)

                };
                int[] random_points = new int[init_point];

                Random random = new Random();

                for (int i = 0; i < init_point;i++)
                {
                    int x, y;

                    do
                    {
                        x = random.Next(map.GetLength(0));
                        y = random.Next(map.GetLength(1));
                    } while (map[x, y] == 1 || start_point.Any(point => point == (y, x)));
                    //Console.WriteLine($"({y},{x})");
                    pointlist.Add((y, x));
                    map[x, y] = 1;

                }
                /*foreach (int point in random_points)
                {
                    int x = point / 100, y = point % 100;
                    pointlist.Add((y,x));
                    map[x,y] = 1;
                }

                /*for (int i = 0; i < map.GetLength(0); i++)
                {
                    for (int j = 0; j < map.GetLength(1); j++)
                    {
                        Console.Write($"{map[i, j]}");
                    }
                    Console.WriteLine("\n");
                }*/
               

            }

            private int GenerateRandomNumber(int min, int max)
            {
                Random random = new Random();
                return random.Next(min, max + 1);
            }
        }

        // 定义节点类
        public class Node
        {
            public int x, y; // 节点坐标
            public int Width = 2;  //坦克宽度
            public int Height = 2;//坦克高度
            public int g; // 从起点到当前节点的实际代价
            public int h; // 从当前节点到目标节点的估计代价
            public int f; // f = g + h，代表总代价
            public Node parent; // 父节点，用于回溯路径

            public Node(int x, int y)
            {
                this.x = x;
                this.y = y;
            }
            public override bool Equals(object obj)
            {
                // 如果 obj 为空或不是 Node 类型，返回 false
                if (obj == null || !(obj is Node))
                {
                    return false;
                }

                // 将 obj 转换为 Node 类型
                Node otherNode = (Node)obj;

                // 比较 X 和 Y 属性来判断节点是否相等
                return this.x == otherNode.x && this.y == otherNode.y;
            }

            public override int GetHashCode()
            {
                // 为了保持一致性，实现 GetHashCode 方法以匹配 Equals 方法的逻辑
                unchecked
                {
                    int hash = 17;
                    hash = hash * 23 + x.GetHashCode();
                    hash = hash * 23 + y.GetHashCode();
                    return hash;
                }
            }
        }

        public void show_init()
        {
            Cv2.NamedWindow(wind, WindowFlags.AutoSize);
            Cv2.MoveWindow(wind, 0, 0);
        }
        public void show_route()
        {
            mapimage.SetTo(Scalar.Black);
            foreach (var point in points)
            {
                Rect region = new Rect(point.x*scale,point.y*scale,scale, scale);
                mapimage.Rectangle(region,new Scalar(0,0,150), -1);
                
            }
            Cv2.ImShow(wind, mapimage);
            Cv2.WaitKey(1);
        }
        
        public void show_route(Node node)
        {
            Rect region = new Rect(node.x * scale, node.y * scale, scale, scale);
            mapimage.Rectangle(region, Scalar.Blue, -1);
            Cv2.ImShow(wind, mapimage);
            Cv2.WaitKey(1);
        }

        public void show_route(List<Node> nodes)
        {
            mapimage.SetTo(Scalar.Black);
            foreach (var point in points)
            {
                Rect region = new Rect(point.x * scale, point.y * scale, scale, scale);
                mapimage.Rectangle(region, new Scalar(0,0,150), -1);

            }
            foreach (var route in nodes)
            {
                Rect region = new Rect(route.x * scale, route.y * scale, scale, scale);
                mapimage.Rectangle(region, Scalar.Green, -1);
                Cv2.ImShow(wind, mapimage);
                Cv2.WaitKey(1);
            }
            Cv2.ImShow(wind, mapimage);
            Cv2.WaitKey(1);
        }
        // A*路径规划算法
        public List<Node> FindPath(Node start, Node goal)
        {
            List<Node> openList = new List<Node>();
            List<Node> closedList = new List<Node>();

            openList.Add(start);

            while (openList.Count > 0)
            {
                //Thread.Sleep(100);
                Node current = openList[0];

                Console.SetCursorPosition(0, 10);
                Console.WriteLine($"开始Oplist:\t{openList.Count}");
                Console.WriteLine($"开始Cllist:\t{closedList.Count}");
                // 找到最小f值的节点
                for (int i = 1; i < openList.Count; i++)
                {
                    if (openList[i].f < current.f)
                    {
                        current = openList[i];
                    }
                }
                Console.SetCursorPosition(0, 15);
                openList.Remove(current);
                Console.WriteLine($"移除了点:\t（{current.x},{current.y})");
                closedList.Add(current);
                //Console.WriteLine($"plist:\t{openList.Count}");
                if (current.x == goal.x && current.y == goal.y)
                {
                    // 找到路径，回溯
                    List<Node> path = new List<Node>();
                    while (current != null)
                    {
                        path.Add(current);
                        current = current.parent;
                    }
                    path.Reverse();
                    show_route(path);
                    return path;
                }

                // 获取当前节点的邻居
                List<Node> neighbors = GetNeighbors(current);

                foreach (Node neighbor in neighbors)
                {
                    if (closedList.Any(node => node.x == neighbor.x && node.y == neighbor.y) || map[neighbor.y, neighbor.x] == 1)//|| map[neighbor.x, neighbor.y] == 1
                    {
                        continue;
                    }

                    int tentativeG = current.g + 1;

                    if (!openList.Any(node => node.x == neighbor.x && node.y == neighbor.y) || tentativeG < neighbor.g)
                    {
                        neighbor.g = tentativeG;
                        neighbor.h = ManhattanDistance(neighbor, goal);
                        neighbor.f = neighbor.g + neighbor.h;
                        neighbor.parent = current;

                        if (!openList.Any(node => node.x == neighbor.x && node.y == neighbor.y))
                        {
                            Console.SetCursorPosition(0, 20);
                            Console.WriteLine($"添加了点:\t({neighbor.x},{neighbor.y})");
                            openList.Add(neighbor);
                            show_route(neighbor);
                            Console.WriteLine($"添加Oplist:\t{openList.Count}");
                        }
                    }
                }
                
            }

            // 没有找到路径
            return null;
        }
        public List<Node> FindPath1(Node start, Node goal)
        {
            //List<Node> openList = new List<Node>();
            //List<Node> closedList = new List<Node>();

            SimplePriorityQueue<Node> openList = new SimplePriorityQueue<Node>();
            HashSet<Node> closedList = new HashSet<Node>();

            openList.Enqueue(start, start.f);

            while (openList.Count > 0)
            {
                //Thread.Sleep(100);
                Node current =openList.Dequeue();
                closedList.Add(current);
               
                //Console.WriteLine($"plist:\t{openList.Count}");
                if (current.x == goal.x && current.y == goal.y)
                {
                    // 找到路径，回溯
                    List<Node> path = new List<Node>();
                    while (current != null)
                    {
                        path.Add(current);
                        current = current.parent;
                    }
                    path.Reverse();
                    show_route(path);
                    return path;
                }

                // 获取当前节点的邻居
                List<Node> neighbors = GetNeighbors(current);

                foreach (Node neighbor in neighbors)
                {
                    if (closedList.Contains(neighbor)|| map[neighbor.y, neighbor.x] == 1)//|| map[neighbor.x, neighbor.y] == 1
                    {
                        continue;
                    }

                    int tentativeG = current.g + 1;

                    bool node_exists = openList.Contains(neighbor);
                   
                    
                        
                    neighbor.g = tentativeG;
                    neighbor.h = ManhattanDistance(neighbor, goal);
                    neighbor.f = neighbor.g + neighbor.h;
                    neighbor.parent = current;
                   
                    if (node_exists )//&& neighbor.f < openList.GetPriority(neighbor)
                    {

                        if (neighbor.f < openList.GetPriority(neighbor))
                        {
                            openList.Remove(neighbor);
                            openList.Enqueue(neighbor, neighbor.f);
                            //Console.WriteLine("重置了");
                        }
                       
                        //openList.UpdatePriority(neighbor, neighbor.f);

                    }
                    else
                    {
                        openList.Enqueue(neighbor, neighbor.f);
                        //show_route(neighbor);
                        //Console.WriteLine("0");
                    }
                    
                    
                    
                }

            }

            // 没有找到路径
            return null;
        }
        // 计算曼哈顿距离
        private int ManhattanDistance(Node a, Node b)
        {
            int x = Math.Abs(a.x - b.x);
            int y = Math.Abs(a.y - b.y);
            //return (int)Math.Sqrt(x*x+y*y) ;
            return (int)Math.Sqrt(x*x+y*y);
        }
        private List<Node> GetNeighbors(Node node)
        {
            List<Node> neighbors = new List<Node>();

            int[] dx = { 1, -1, 0, 0 };
            int[] dy = { 0, 0, 1, -1 };

            for (int i = 0; i < 4; i++)
            {
                int newX = node.x + dx[i];
                int newY = node.y + dy[i];

                if (newX >= 0 && newX < width && newY >= 0 && newY < height)
                {
                    neighbors.Add(new Node(newX, newY));
                }
            }

            return neighbors;
        }

        // 获取邻居节点
        private List<Node> GetNeighbors1(Node node)
        {
            List<Node> neighbors = new List<Node>();

            int[] dx = { 1, -1, 0, 0 };
            int[] dy = { 0, 0, 1, -1 };

            for (int i = 0; i < 4; i++)
            {
                int newX = node.x + dx[i];
                int newY = node.y + dy[i];
                Node newnode = new Node(newX,newY);
                if (newX >= 0 && newX < width && newY >= 0 && newY < height)
                {
                    if(HasCollision(dx[i],dy[i],newnode,map))
                        neighbors.Add(newnode);
                }
            }

            return neighbors;
        }
        private bool HasCollision(int dx,int dy,Node node, int[,] map)
        {
            if (dx > 0) 
            {
                for(int i=node.y;i<node.y+node.Height;i++)
                {
                    if (node.x + node.Width - 1 >= map.GetLength(1)||map[i, node.x + node.Width - 1] == 1)
                        return false;
                }

            }
            if (dx < 0) 
            {
                for (int i = node.y; i < node.y + node.Height; i++)
                {
                    if ( node.x < 0||map[i,node.x] == 1)
                        return false;
                }
            }
            if (dy > 0) 
            {
                for (int i=node.x;i<node.x+node.Width;i++)
                {
                    if ( node.y + node.Height - 1 >= map.GetLength(0) || map[ node.y + node.Height - 1,i] == 1)
                        return false;
                }
            }
            if (dy < 0)
            {
                for (int i = node.x; i < node.x + node.Width; i++)
                {
                    if ( node.y < 0||map[ node.y,i] == 1)
                        return false;
                }
            }
            return true;
        }

    }
   

    class MazeGenerator
    {
        private static Random random = new Random();
        private string wind = "地图生成";
        private int scale = 8;
        public MazeGenerator()
        {
            int width = 100;  // 地图宽度
            int height = 100; // 地图高度
            int[,] maze = GenerateMaze(width, height);
            //PrintMaze(maze);
            show_init();
            show_image(maze);
            // 打印生成的迷宫地图
            

        }


        // 生成一个随机迷宫地图
        static int[,] GenerateMaze(int width, int height)
        {
            int[,] maze = new int[height, width];

            // 初始化地图，将所有格子都设置为障碍
            for (int i = 0; i < height; i++)
            {
                for (int j = 0; j < width; j++)
                {
                    maze[i, j] = 1;
                }
            }

            // 从随机起始点开始生成迷宫
            GenerateMazeRecursive(maze, random.Next(1, height - 1), random.Next(1, width - 1));

            // 添加起点和终点
            maze[1, 0] = 0; // 起点
            maze[height - 2, width - 1] = 0; // 终点

            return maze;
        }

        // 递归回溯算法生成迷宫
        static void GenerateMazeRecursive(int[,] maze, int x, int y)
        {
            int[] dx = { 1, -1, 0, 0 };
            int[] dy = { 0, 0, 1, -1 };

            int[] randomdirections = { 0, 1, 2, 3 };
            Shuffle(randomdirections);

            foreach (int direction in randomdirections)
            {
                int nx = x + 2* dx[direction];
                int ny = y + 2* dy[direction];

                if (IsInsideMaze(maze, nx, ny))
                {
                    if (maze[nx, ny] == 1)
                    {
                        maze[nx, ny] = 0; // 打通当前格子
                        
                        maze[x + dx[direction] , y + dy[direction]] = 0; // 打通中间格子
                        GenerateMazeRecursive(maze, nx, ny);
                    }
                }
            }
        }

        // 检查点是否在迷宫内
        static bool IsInsideMaze(int[,] maze, int x, int y)
        {
            int height = maze.GetLength(0);
            int width = maze.GetLength(1);
            return x >= 0 && x < height && y >= 0 && y < width;
        }

        // 打印迷宫地图
        static void PrintMaze(int[,] maze)
        {
            int height = maze.GetLength(0);
            int width = maze.GetLength(1);

            for (int i = 0; i < height; i++)
            {
                for (int j = 0; j < width; j++)
                {
                    Console.Write(maze[i, j] == 1 ? "1" : "0");
                }
                Console.WriteLine();
            }
        }
        public void show_init()
        {
            Cv2.NamedWindow(wind, WindowFlags.Normal);
            Cv2.MoveWindow(wind, 0, 0);
        }
        public void show_image(int[,] maze)
        {
            Mat image = new Mat(new Size(maze.GetLength(1)*scale,maze.GetLength(0)*scale), MatType.CV_8UC3, new Scalar(0, 0, 0));
            for (int x=0;x< maze.GetLength(1); x++)
            {
                for (int y=0;y< maze.GetLength(0);y++)
                {
                    if (maze[y,x]==1)
                    {
                        Rect region = new Rect(x * scale, y * scale, scale, scale);
                        image.Rectangle(region, new Scalar(100,50, 0), -1);
                    }
                }
            }
            Cv2.ImShow(wind,image);
            Cv2.WaitKey(10);

        }
        // 随机洗牌数组的元素顺序
        static void Shuffle(int[] array)
        {
            int n = array.Length;
            for (int i = n - 1; i > 0; i--)
            {
                int j = random.Next(0, i + 1);
                int temp = array[i];
                array[i] = array[j];
                array[j] = temp;
            }
        }
    }
    class MazeGenerator1
    {
        private static Random random = new Random();
        private string wind = "Prim算法生成迷宫";
        private int scale = 2;
        private int path_width = 8;
        public MazeGenerator1()
        {
            int width = 184;  // 地图宽度
            int height = 184; // 地图高度
            int[,] maze = GenerateMaze(width, height);

            // 打印生成的迷宫地图
            PrintMaze(maze);
            show_init();
            show_image(maze);
        }

        // 生成一个随机迷宫地图（Prim算法）
        private int[,] GenerateMaze(int width1, int height1)
        {
            int width = width1 / path_width;
            int height = height1 / path_width;
            int[,] maze = new int[height , width ];

            // 初始化地图，将所有格子都设置为障碍
            for (int i = 0; i < height; i++)
            {
                for (int j = 0; j < width; j++)
                {
                    maze[i, j] = 1;
                }
            }

            // 随机选择一个起始点，并将其标记为通道
            int startX = random.Next(1, height - 1);
            int startY = random.Next(1, width - 1);
            maze[startX, startY] = 0;

            // 创建一个用于Prim算法的边缘列表
            List<(int, int, int)> edges = new List<(int, int, int)>();
            //HashSet<(int, int)> edges1 = new HashSet<(int, int)>();

            // 添加起始点周围的边缘
            AddEdges(edges, maze, startX, startY);

            // 主要生成迷宫的过程
            while (edges.Count > 0)
            {
                // 随机选择一条边缘
                int randomIndex = random.Next(edges.Count);
                (int, int, int) edge = edges[randomIndex];
                int x = edge.Item1;
                int y = edge.Item2;
                int direction = edge.Item3;

                switch (direction)
                {
                    case 0:
                        maze[y, x] = 0;
                        maze[y, x - 1] = 0;
                        AddEdges(edges, maze, x, y);
                        edges.RemoveAt(randomIndex);
                        break;
                    case 1:
                        maze[y, x] = 0;
                        maze[y, x + 1] = 0;
                        AddEdges(edges, maze, x, y);
                        edges.RemoveAt(randomIndex);
                        break;
                    case 2:
                        maze[y, x] = 0;
                        maze[y - 1, x] = 0;
                        AddEdges(edges, maze, x, y);
                        edges.RemoveAt(randomIndex);
                        break;
                    case 3:
                        maze[y, x] = 0;
                        maze[y + 1, x] = 0;
                        AddEdges(edges, maze, x, y);
                        edges.RemoveAt(randomIndex);
                        break;
                    default:
                        break;
                }
                /*bool levHorizontal_sidesel = IsInsideMaze(maze, x + 2, y) && IsInsideMaze(maze, x - 2, y);
                bool vertical_sides = IsInsideMaze(maze, x, y + 2) && IsInsideMaze(maze, x, y - 2);
                bool level_value =levHorizontal_sidesel? maze[y, x + 2] == 0 && maze[y, x - 2] == 0:false;
                bool vertical_value = vertical_sides? maze[y + 2, x] == 0 && maze[y - 2, x] == 0:false;

                if (levHorizontal_sidesel&&level_value)
                {
                    AddEdges(edges, edges1, maze, x, y);
                    edges.RemoveAt(randomIndex);
                    edges1.Add((x,y));
                }
                else if (vertical_sides && vertical_value)
                {
                    AddEdges(edges, edges1, maze, x, y);
                    edges.RemoveAt(randomIndex);
                    edges1.Add((x, y));
                }
                else
                {
                    maze[y, x] = 0;
                    AddEdges(edges, edges1, maze, x, y);
                    edges.RemoveAt(randomIndex);
                    //Console.WriteLine("打通了y方向");
                }
                // 从边缘列表中移除已处理的边缘
                */
            }

            // 添加起点和终点
            maze[1, 0] = 0; // 起点
            maze[height - 2, width - 1] = 0; // 终点

            return Mazezoom(maze,width1,height1);
        }

        // 添加与给定点相邻的墙边缘到边缘列表
        static void AddEdges(List<(int, int, int)> edges, int[,] maze, int x, int y)
        {
            int[] dx = { 1, -1, 0, 0 };
            int[] dy = { 0, 0, 1, -1 };

            for (int i = 0; i < 4; i++)
            {
                int nx = x + 2 * dx[i];
                int ny = y + 2 * dy[i];

                if (IsInsideMaze(maze, nx, ny))
                {
                    if (maze[ny, nx] == 1)
                    {
                        if (!edges.Any(w => w.Item1 == nx && w.Item2 == ny))//&& !edges1.Contains((nx, ny))
                            edges.Add((nx, ny, i));
                    }
                }
            }
        }

        // 检查点是否在迷宫内
        static bool IsInsideMaze(int[,] maze, int x, int y)
        {
            int height = maze.GetLength(0);
            int width = maze.GetLength(1);
            return x >= 0 && x < height && y >= 0 && y < width;
        }
        public int[,] Mazezoom(int[,] maze,int width,int height)
        {
            int[,] mazezoom = new int[height,width];
            for (int x=0;x<maze.GetLength(1);x++)
            {
                for (int y=0;y<maze.GetLength(0);y++)
                {
                    for (int nx=x*path_width;nx<(x+1)*path_width;nx++)
                    {
                        for (int ny=y*path_width;ny<(y+1)*path_width;ny++)
                        {
                            mazezoom[ny, nx] = maze[y, x];
                        }
                    }
                }
            }
            return mazezoom;
        }
        public void show_init()
        {
            Cv2.NamedWindow(wind, WindowFlags.Normal);
            Cv2.MoveWindow(wind, 0, 0);
        }
        public void show_image(int[,] maze)
        {
            Mat image = new Mat(new Size(maze.GetLength(1) * scale, maze.GetLength(0) * scale), MatType.CV_8UC3, new Scalar(0, 0, 0));
            for (int x = 0; x < maze.GetLength(1); x++)
            {
                for (int y = 0; y < maze.GetLength(0); y++)
                {
                    if (maze[y, x] == 0)
                    {
                        Rect region = new Rect(x * scale, y * scale, scale, scale);
                        image.Rectangle(region, new Scalar(100, 50, 0), -1);
                    }
                }
            }
            Cv2.ImShow(wind, image);
            Cv2.WaitKey(0);
        }

        // 打印迷宫地图
        static void PrintMaze(int[,] maze)
        {
            int height = maze.GetLength(0);
            int width = maze.GetLength(1);

            for (int i = 0; i < height; i++)
            {
                for (int j = 0; j < width; j++)
                {
                    Console.Write(maze[i, j] == 1 ? "1" : "0");
                }
                Console.WriteLine("");
            }
        }
    }


    class Program
    {
        static void Main(string[] args)
        {
            Stopwatch stopwatch = new Stopwatch();
            
            AStarPathfinding pathfinder = new AStarPathfinding();
            AStarPathfinding.Node start = new AStarPathfinding.Node(0, 0);
            AStarPathfinding.Node goal = new AStarPathfinding.Node(199, 199);

            stopwatch.Start();
            List<AStarPathfinding.Node> path = pathfinder.FindPath1(start, goal);
            //Cv2.WaitKey(0);
            stopwatch.Stop();
            // 获取经过的时间
            TimeSpan elapsed = stopwatch.Elapsed;
            // 输出运行时间
            Console.WriteLine($"代码执行时间: {elapsed.TotalMilliseconds} 毫秒");
            if (path != null)
            {
                Console.WriteLine("找到路径：");
                foreach (AStarPathfinding.Node node in path)
                {
                    //Console.WriteLine($"({node.x}, {node.y})");
                }
            }
            else
            {
                Console.WriteLine("未找到路径。");

            }
            MazeGenerator maze = new MazeGenerator();
            MazeGenerator1 maze1 = new MazeGenerator1();
            

            Console.ReadLine();
        }
    }
}
