using System;
using System.IO;
using System.Collections;
using System.Collections.Generic;
using System.Linq;


//  A bit of driver code, just for testing and demonstration until we integrate our work
//Console.WriteLine("Running MazeNav with A* search algorithm");
//List<int> test_obstacles = new List<int>();
//test_obstacles.Add(144);
//test_obstacles.Add(146);
//test_obstacles.Add(30);
//test_obstacles.Add(130);
//MazeGraph mg = new MazeGraph(0, 145, 15, 15, test_obstacles);
//int[] arr = new int[] { 0, 145, 15, 15, 5, 30, 146 };
//Main(arr);

string[] parsed_args = Environment.GetCommandLineArgs();
int[] arr = new int[parsed_args.Length];

int ix;

for (int i = 0; i < parsed_args.Length; i++)
{
    int.TryParse(parsed_args[i], out ix);
    arr[i] = ix;
}


Main(arr);

static void Main(int[] args)
{
    Console.WriteLine("running main");
    // string[] parsed_args = Environment.GetCommandLineArgs();

    // Console.WriteLine(parsed_args);

    int number_obstacles = args.Length - 5; // First 4 arguments are source, goal, height, width, and then remainder are obstacles
    List<int> test_obstacles = new List<int>();
    for (int i = 0; i < number_obstacles; i++)
    {
        test_obstacles.Add(args[i + 5]);
    }

    MazeGraph mg = new MazeGraph(args[1], args[2], args[3], args[4], test_obstacles);

    mg.PrintPath();



}


// Re-write of Maze Navigator using Node class to make using A* algorithm simpler

public class Node
{
    public int node_id; // Integer ID for node
    public List<int> adj_list = new List<int>();   // List of adjacent nodes
    public double eucDist;  // Euclidean distance from goal node
    public int cost = 1000;    // Cost (number of grid spaces from source) to reach this node
    public double costDist; // Cost + Euclidean distance from goal
    public int parent;	// ID for parent node

}   // End class Node



public class MazeGraph
{
    static int N;   // Number of nodes, value is set by constructor function
    static int H, W;	// Height and width of maze, set by constructor function
    static List<Node> node_list;    // List of composite nodes
    static List<int> obst_list; // Obstacle list

    public static List<Node> path; //  This is the path discovered by the search algorithm
    public static List<String> step_list;   // List of steps for robot to take

    // Constructor
    // Expects arguments for source node, goal node, as well as height and width of maze
    public MazeGraph(int source, int goal, int h, int w, List<int> obstacles)
    {
        H = h;
        W = w;
        N = h * w;

        obst_list = obstacles;

        // Initialise node_list
        node_list = new List<Node>();
        Node n;
        for (int i = 0; i < N; i++)
        {
            n = new Node();
            n.node_id = i;
            n.eucDist = GetEucDist(i, source);
            n.costDist = n.cost + n.eucDist;

            node_list.Add(n);
        }
        BuildAdjacencies(); // Adds adjacencies to each node

        //path = new List<Node>();

        AStar(source, goal);
        //PrintPath();
    }

    // Add adjacent nodes to adj_list of each node
    // Called by parent class (MazeGraph) constructor

    public void BuildAdjacencies()
    {
        for (int i = 0; i < N; i++)
        {   // for each node
            if (obst_list.Contains(i))
            {   // Check node for obstacle
                continue;   // This node will not have any adjacencies, skip to next node
            }

            //	Left adjacency
            if (i % W != 0)
            {   // if not on left of board, add left adjacency
                if (obst_list.Contains(i - 1))
                {   // Check left node for obstacle
                }
                else {
                    node_list[i].adj_list.Add(i - 1); // Add left adjacency
                }
                
            }

            //	Right adjacency
            if (i % W != W - 1)
            {   // if not on right of board, add right adjacency
                if (obst_list.Contains(i + 1))
                {   // Check right node for obstacle
                }
                else {
                    node_list[i].adj_list.Add(i + 1); // Add right adjacency
                }
                
            }

            //	Lower adjacency
            if (i >= W)
            {   // if not on bottom row, add lower adjacency
                if (obst_list.Contains(i - W))
                {   // Check lower node for obstacle
                }
                else {
                    node_list[i].adj_list.Add(i - W); // Add lower adjacency
                }
                
            }

            //	Upper adjacency
            if (i < H * (W - 1))
            {   // if not on top row, add upper adjacency
                if (obst_list.Contains(i + W))
                {   // Check upper node for obstacle
                }
                else {
                    node_list[i].adj_list.Add(i + W); // Add upper adjacency
                }
                
            }
        }

        return;
    }   // End BuildAdjacencies function


    // Calculate Euclidean distance between node n and goal g
    public double GetEucDist(int n, int g)
    {
        double dist;
        int node_x, node_y;
        int goal_x, goal_y;

        goal_x = (g % W);
        goal_y = (int)(g / W);

        node_x = (n % W);
        node_y = (int)(n / W);

        dist = (Math.Pow(goal_x - node_x, 2) + Math.Pow(goal_y - node_y, 2));

        return dist;
    }   // End GetEucDist function


    // The following code pertains to operation of the A* algorithm

    public void AStar(int source, int goal)
    {
        Node n; // Current node
        Node a; // Adjacent node
        List<Node> activeNodes = new List<Node>();
        List<Node> visitedNodes = new List<Node>();

        n = node_list[source];
        n.cost = 0;
        n.costDist = n.cost + n.eucDist;
        activeNodes.Add(n); // Add starting node to active nodes

        //  Primary search loop
        while (activeNodes.Any())
        {
            n = activeNodes.OrderBy(x => x.costDist).First();   //  Order by cost + Euclidean distance (costDist)

            if (n.node_id == goal)  // If we have found goal node
            {
                Console.WriteLine("Found goal node");
                //  Now work backwards to determine path
                path = new List<Node>();
                n = node_list[goal];
                while (n.node_id != source) // Loop until you reach starting position
                {
                    path.Add(n);
                    //Console.WriteLine(n.node_id);
                    n = node_list[n.parent];

                }
                //path.Add(node_list[source]);    // Add source node, just comment this out if we don't want the source node included in path
                path.Reverse();     // Reverse path list such that it goes from source to goal
                return;
            }

            foreach (int adj_id in n.adj_list)  // Add each adjacent node to node list
            {
                a = node_list[adj_id];

                if (n.cost + 1 < a.cost)    // Update cost, costDist, and parent
                {
                    a.cost = n.cost + 1;
                    a.costDist = a.cost;
                    a.parent = n.node_id;
                }

                if (visitedNodes.Contains(a))   // If adjacent node has already been explored, we can skip this one
                {
                   continue;
                }
                activeNodes.Add(a);     // Add adjacent node to activeNodes
            }
            visitedNodes.Add(n);    // Add current node to list of visited nodes
            activeNodes.Remove(n);  // Remove current node from list of active nodes
        }

        // If we get through the while loop without returning (finding goal node)
        Console.WriteLine("No valid path found");

    }   //  End function AStar


    //  Print path, for testing
    public void PrintPath()
    {
        String outfile = "AStarOutput.txt";
        String[] step = new string[path.Count()];
        int index = 0;



        StreamWriter sw = new StreamWriter("C:\\ws\\catkin_ws\\src\\assignment3\\src\\AStarOutput.txt");
        foreach (Node n in path)
        {
            //Console.WriteLine(n.node_id);
            step[index] = n.node_id.ToString();
            sw.WriteLine(step[index]);
            // Console.WriteLine(step[index]);
            index++;

        }
        sw.Close();



        Console.WriteLine("Writing line");

        //File.WriteAllLines(outfile, step);

        return;
    }

}   // End class MazeGraph