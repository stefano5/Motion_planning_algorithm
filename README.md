# Motion_planning_algorithm

This is a motion planning algorithm implementation.

Here are implementated two algorithms:
 * Forward value iteration
 * Backward value iteration
 

These algorithms computes the optimal paths in a oriented graph.

A short explanation of the Backward value iteration (may quite strange) is here: https://github.com/stefano5/Backward_Value_Iteration


## Problem

We have to find the optimal path in a oriented graph, eg:

![Screenshot_20211019_143913](https://user-images.githubusercontent.com/40228829/137910853-a24d1d0b-d259-4e74-af4e-dcf0c0d91c80.png)

Where (A, ..., G) are nodes, (A,B), (A,C), ... are oriented arcs with cost.

We want to go (e.g.) from the node D to A through the shortest (or less expansive) path.

## Backward and Forward Value Iteration

They are two similiar (but specular) algorithms that computes all the optimal paths from (or up to) a node.

Let's use them.

## Write the graph in a file

For each oriented graph you have, you must create a file where each row follow this syntax:
  * tail,head,cost    type: string,string,int     (Comma is a delimeter)

Each row is an arc and also a node declaration.


For example, the file for the previous graph is:

```
A,B,7
A,C,5
A,F,2
B,G,2
C,B,1
C,G,4
C,D,1
D,E,1
D,G,1
E,C,2
E,D,4
E,F,1
F,A,1
F,C,1
F,E,1
G,B,1
G,D,1
```

## How to use it
These algorithms required a N number of steps parameter. The found path will have N steps exactly.

In order to forget this parameter just add a new arc in your graph, that depends on which algorithm you want to use.
  * If you are using the Backward Value Iteration add an arc with null cost on the node you want to reach.
  * If you are using the Forward Value Iteration add an arc with null cost to the node you are starting from.

# Forward Value Iteration

Eg, if we want to reach the node A from the node D we will write:

```
#include "motion_planning_algorithms.cpp" // put it where you need

...

std::string graph_path = "/my/graph/path";

Graph graph(graph_path);

std::string starting_node = "D";
std::string arrival_node = "A";
int n_iteration n_iteration = 7; // tip: set it to the number of arcs

...

graph.compute_optimal_cost_forward(n_iteration, starting_node);  // compute all the optimal paths for each node from 'starting_node'

vector<Node> op =  graph.get_optimal_path_forward(arrival_node);  // get the optimal path to reach 'arrival_node' 

for (Node n:op) {
    cout << "\t" <<n.get_head() << endl;
    //move_robot_to(n.get_head());  // for example
}

```

The found optimal path will be:

```
Optimal path:
        D
        E
        F
        A
```


Insted, if we want to use the Backward Value Iteration we will write:


```
#include "motion_planning_algorithms.cpp" // put it where you need

...

std::string graph_path = "/my/graph/path";

Graph graph(graph_path);

std::string arrival_node = "D";   // in Backward Val.It. we'll go backward!
std::string starting_node = "A";
int n_iteration n_iteration = 7; // tip: set it to the number of arcs

...

graph.compute_optimal_cost_backward(n_iteration, arrival_node);  // compute all the optimal paths for each node to reach 'arrival_node'

vector<Node> op =  graph.get_optimal_path_backward(starting_node);  // get the optimal path from 'starting_node' 

for (Node n:op) {
    cout << "\t" <<n.get_head() << endl;
    //move_robot_to(n.get_head());  // for example
}
```

The optimal path will be:

```
Optimal path:
        D
        E
        F
        A
```


