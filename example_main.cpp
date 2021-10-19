#include <iostream>
#include <stdexcept>
#include <string>
#include <fstream>

#include "motion_planning_algorithms.cpp"


using namespace std;

int main(int argc, char **argv) {
    string name_file;
    uint16_t n_iteration;
    string name_xf;

    /////////////////////////////////////// input check

    if (argc == 3) {
        name_file = string(argv[1]);
        n_iteration = atoi(argv[2]);
    } else if (argc == 2) {
        if (atoi(argv[1]) == 0) {
            name_file = string(argv[1]);
            cout << "Enter N (number interations): " << endl;
            cin >> n_iteration;
        } else {
            n_iteration = atoi(argv[1]);
            cout << "Enter file name: " << endl;
            cin >> name_file;
        }
    } else if (argc == 1) {
        cout << "Enter file name: " << endl;
        cin >> name_file;
        cout << "Enter N (number interations): " << endl;
        cin >> n_iteration;
    } else {
        cerr << "Numero di parametri errati :(" << endl;
        cerr << "Parameter numbers is wrong :(" << endl;
        cerr << argv[0] << " <name file> <number iterations>" << endl;
        exit(-1);
    }

    cout << "File name: " << name_file << endl;
    cout << "Number of iteration given: " << n_iteration << endl;

    /////////////////////////////////////// choose starting node and arrival node

    cout << "Enter starting node: " << endl;
    string starting_node;
    //cin >> starting_node;
    starting_node = "6";
    starting_node = "D";
    cout << "[" << starting_node << "] is the starting node" << endl;
    cout << endl;

    string arrival_node;
    cout << "Chose which optimal path to see. Enter an arrival node: " << endl;
    //cin >> arrival_node;
    arrival_node = "1";
    arrival_node = "A";

    ///////////////////////////////////////
    ///////////////////////////////////////    MOTION PLANNING algorithm:     Instance class, compute optimal cost and show it
    ///////////////////////////////////////

    Graph graph(name_file);
    //graph.print_graph();  // if you need


    ///////////////////////////////////////
    ///////////////////////////////////////   FORWARD VALUE ITERATION
    ///////////////////////////////////////

    graph.compute_optimal_cost_forward(n_iteration, starting_node);

    ///// Show the optimal path
    cout << "Forward value iteration from [" << starting_node << "] to [" << arrival_node <<  "] is: " << endl;
    cout << graph._get_optimal_path_forward(arrival_node) << endl;
    vector<Node> op =  graph.get_optimal_path_forward(arrival_node);

    cout << "Optimal path:" << endl;
    for (Node n:op) {
        cout << "\t" <<n.get_head() << endl;
    }

    ///////////////////////////////////////
    ///////////////////////////////////////   BACKWARD VALUE ITERATION
    ///////////////////////////////////////
    //Switch nodes just
    //starting_node.swap(arrival_node);


    graph.compute_optimal_cost_backward(n_iteration, arrival_node);

    ///// Show the optimal path
    cout << "Backward value iteration from [" << starting_node << "] to [" << arrival_node <<  "] is: " << endl;
    cout << graph._get_optimal_path_backward(starting_node) << endl;

    op =  graph.get_optimal_path_backward(starting_node);

    cout << "Optimal path:" << endl;
    for (Node n:op) {
        cout << "\t" <<n.get_head() << endl;
    }

    graph.~Graph();

    /////////////////////////////////////// All done

    exit(EXIT_SUCCESS);
}

