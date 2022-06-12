using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class Pathfinding : MonoBehaviour
{

	public Transform seeker, target;
	Grid grid;

	void Awake()
	{
		grid = GetComponent<Grid>();
	}
	// in the update Method we added updates for BFS , UCS, and DFS so we can get the time of execution of each one of the search methods in milliseconds;
	void Update()
	{


		var b = new System.Diagnostics.Stopwatch();// for time counting for BFS;
		b.Start();
		FindPathBFS(seeker.position, target.position);
		b.Stop();
		Debug.Log($"Time OF Execution of the BFS SEARCH is {b.ElapsedMilliseconds} in ms");

		var d = new System.Diagnostics.Stopwatch(); // for time counting FOR DFS
		FindPathDFS(seeker.position, target.position);
		d.Stop();
		Debug.Log($"Time OF Execution of the DFS SEARCH is {d.ElapsedMilliseconds} in ms");

		var a = new System.Diagnostics.Stopwatch();// for time counting FOR A*;
		a.Start();
		FindPath(seeker.position, target.position);
		a.Stop();
		Debug.Log($"Time OF Execution of the A* SEARCH is {a.ElapsedMilliseconds} in ms");

		var u = new System.Diagnostics.Stopwatch();// for time counting FOR UCS;

		u.Start();
		FindPathUCS(seeker.position, target.position);
		u.Stop();
		Debug.Log($"Time OF Execution of the UCS SEARCH is {u.ElapsedMilliseconds} in ms");

		

	}

	// path finding section:

	void FindPath(Vector3 startPos, Vector3 targetPos)
	{
		Node startNode = grid.NodeFromWorldPoint(startPos);
		Node targetNode = grid.NodeFromWorldPoint(targetPos);

		List<Node> openSet = new List<Node>();
		HashSet<Node> closedSet = new HashSet<Node>();
		openSet.Add(startNode);

		while (openSet.Count > 0)
		{
			Node node = openSet[0];
			for (int i = 1; i < openSet.Count; i++)
			{
				if (openSet[i].fCost < node.fCost || openSet[i].fCost == node.fCost)
				{
					if (openSet[i].hCost < node.hCost)
						node = openSet[i];
				}
			}

			openSet.Remove(node);
			closedSet.Add(node);

			if (node == targetNode)
			{
				RetracePath(startNode, targetNode);
				return;
			}

			foreach (Node neighbour in grid.GetNeighbours(node))
			{
				if (!neighbour.walkable || closedSet.Contains(neighbour))
				{
					continue;
				}

				int newCostToNeighbour = node.gCost + GetDistance(node, neighbour);
				if (newCostToNeighbour < neighbour.gCost || !openSet.Contains(neighbour))
				{
					neighbour.gCost = newCostToNeighbour;
					neighbour.hCost = GetDistance(neighbour, targetNode);
					neighbour.parent = node;

					if (!openSet.Contains(neighbour))
						openSet.Add(neighbour);
				}
			}
		}
	}
	void FindPathBFS(Vector3 startp, Vector3 targetp)// In the BFS search we use as a structure a queue in order to travers or explore a path;
	{


		Node startn = grid.NodeFromWorldPoint(startp);
		Node targetn = grid.NodeFromWorldPoint(targetp);


		Queue<Node> queue = new Queue<Node>();
		HashSet<Node> closedset = new HashSet<Node>();

		queue.Enqueue(startn);

		while (queue.Count != 0){
			Node current_n = queue.Dequeue();
			if (current_n == targetn){
				RetracePathBFS(startn, targetn);
				return; }


			closedset.Add(current_n);// we add the current node tp the closed set;

			foreach (Node neighbour in grid.GetNeighbours(current_n)){
				if (!neighbour.walkable || closedset.Contains(neighbour)){
					continue; }
				if (neighbour.walkable || !queue.Contains(neighbour)){
					closedset.Add(neighbour);// we add the neighbour node to the closed set;
					neighbour.parent = current_n;// the current node become the parent;
					queue.Enqueue(neighbour);}
			}
		}

	}
	void FindPathDFS(Vector3 startp, Vector3 targetp)// in DFS we use as a data structure stacks;
	{


		Node startn = grid.NodeFromWorldPoint(startp);
		Node targetn = grid.NodeFromWorldPoint(targetp);

		Stack<Node> stack = new Stack<Node>();
		HashSet<Node> closedset = new HashSet<Node>();

		stack.Push(startn);

		while (stack.Count != 0){
			Node current_n = stack.Pop();
			if (current_n == targetn){

				RetracePathDFS(startn, targetn);
				return;
			}
			closedset.Add(current_n);
			foreach (Node neighbour in grid.GetNeighbours(current_n)){
				if (!neighbour.walkable || closedset.Contains(neighbour)){
					continue;
				}
				if (neighbour.walkable || !stack.Contains(neighbour)){
					closedset.Add(neighbour);
					neighbour.parent = current_n;
					stack.Push(neighbour);}
			}
		}
	}
	void FindPathUCS(Vector3 startp, Vector3 targetp)// in the UCS search we expend the sheepest path first; 
	{
		int i;
		Node start_n = grid.NodeFromWorldPoint(startp);
		Node target_n = grid.NodeFromWorldPoint(targetp);

		List<Node> Set = new List<Node>();
		HashSet<Node> closedset = new HashSet<Node>();

		Set.Add(start_n);

		while (Set.Count > 0){
			Node n = Set[0];
			for (i = 1; i < Set.Count; i++){
				if (Set[i].gCost < n.gCost || Set[i].gCost == n.gCost){
					n = Set[i];
				}
			}

			Set.Remove(n);
			closedset.Add(n);

			if (n == target_n){
				RetracePathUCS(start_n, target_n);
				return;
			}

			foreach (Node neighbour in grid.GetNeighbours(n)){
				if (!neighbour.walkable || closedset.Contains(neighbour)){
					continue;
				}

				int newCostToNeighbour = n.gCost + GetDistance(n, neighbour);
				if (newCostToNeighbour < neighbour.gCost || !Set.Contains(neighbour)){
					neighbour.gCost = newCostToNeighbour;
					neighbour.hCost = GetDistance(neighbour, target_n);
					neighbour.parent = n;

					if (!Set.Contains(neighbour))
						Set.Add(neighbour);
				}
			}
		}
	}
	// retrace path section;
	void RetracePath(Node startNode, Node endNode)
	{
		List<Node> path = new List<Node>();
		Node currentNode = endNode;

		while (currentNode != startNode)
		{
			path.Add(currentNode);
			currentNode = currentNode.parent;
		}
		path.Reverse();

		grid.path = path;

	}
	 void RetracePathBFS(Node startn, Node endNode)
	{
		List<Node> path = new List<Node>();
		Node current_n = endNode;

		while (current_n != startn)
		{
			path.Add(current_n);
			current_n = current_n.parent;

		}
		path.Reverse();
		grid.pathBFS = path;
	}

	
	void RetracePathDFS(Node startn, Node endNode)
	{
		List<Node> path= new List<Node>();
		Node current_n = endNode;

		while (current_n != startn)
		{
			path.Add(current_n);
			current_n = current_n.parent;

		}
		path.Reverse();
		grid.pathDFS = path;
	}
	
	void RetracePathUCS(Node startn, Node endNode)
	{
		List<Node> path = new List<Node>();
		Node current_n = endNode;

		while (current_n != startn)
		{
			path.Add(current_n);
			current_n = current_n.parent;

		}
		path.Reverse();
		grid.pathUCS = path;
	}


	int GetDistance(Node nodeA, Node nodeB)
	{
		int dstX = Mathf.Abs(nodeA.gridX - nodeB.gridX);
		int dstY = Mathf.Abs(nodeA.gridY - nodeB.gridY);

		if (dstX > dstY)
			return 14 * dstY + 10 * (dstX - dstY);
		return 14 * dstX + 10 * (dstY - dstX);
	}
}
