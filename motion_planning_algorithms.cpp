
/*
 *
 *      Author: stefano
 *
 */

#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include "motion_planning_algorithms.h"

////////////////////////   Dependencies

int16_t API_S::get_size_file(std::string file_to_read) {
    std::streampos begin,end;
    std::ifstream myfile (file_to_read, std::ios::binary);
    if (myfile.is_open()) {
        begin = myfile.tellg();
        myfile.seekg (0, std::ios::end);
        end = myfile.tellg();
        myfile.close();
        return end - begin;
    } else return -1;
}

bool API_S::exist_file(std::string file_to_read) {
    return get_size_file(file_to_read) != -1;
}

std::vector<std::string> API_S::read_file(std::string file_to_read) {
    std::string file_raw;
    std::vector<std::string> all_file;
    std::ifstream file(file_to_read);

    if (file.is_open()) {
        while ( getline(file, file_raw) ) {
            all_file.push_back(file_raw);
        }
        file.close();
    } else {
        std::cerr << "read_file(" << file_to_read << ") <= file not found" << std::endl;
    }
    return all_file;
}

//////////////////////// end dependencies


//////////// Node functions
Node::Node() {
    node_t = node_c = "";
    cost = id = -1;
}

Node::Node(std::string node_t, std::string node_c, uint16_t cost) {
    this->node_t = node_t;
    this->node_c = node_c;
    this->cost = cost;
    id = -1;
}

std::string Node::to_string() {
    return "Tail: [" + node_c + "], head: [" + node_t +"], cost: " + std::to_string(cost) + "\n";
}

int Node::get_id() {
    return id;
}

void Node::set_id(int id) {
    this->id  = id;
}

int Node::get_cost() { 
    return cost; 
}

std::string Node::get_tail() { 
    return node_c; 
}

std::string Node::get_head() { 
    return node_t; 
}

//////////// Row functions

Row::Row(int cost, std::string node) {
    this->cost = cost;
    this->node = node;
    this->optimal_parent = "-";
}

//////////// G_opt_k functions

G_opt_k::G_opt_k(int k, int N) {
    this->k = k;
    row.reserve(N);
}

G_opt_k::~G_opt_k() {
    row.clear();
}

void G_opt_k::set_max_cost(int _max_cost) { 
    max_cost = _max_cost; 
}

std::string G_opt_k::get_node(int k) { // ritorna il nome del nodo dato
    return row.at(k).node;
}

void G_opt_k::new_node(std::string node, int cost) {
    row.push_back(Row(cost, node));
}

void G_opt_k::set_cost_node(int id_node, int cost) {
    row.at(id_node).cost = cost;
}

void G_opt_k::set_parent_node(int id_node, std::string parent) {
    row.at(id_node).optimal_parent = parent;
}

std::string G_opt_k::get_optimal_parent(int id_node) {
    return (row.at(id_node).optimal_parent);
}

int G_opt_k::get_cost_node(int id_node) {
    return (row.at(id_node).cost);
}

std::string G_opt_k::get_name_node(int id_node) {
    return row.at(id_node).node;
}


std::string G_opt_k::get_cost_node_print(int id_node) {
    if (row.at(id_node).cost == max_cost)
        return "inf";
    else
        return std::to_string(row.at(id_node).cost);
}

int G_opt_k::get_k() { 
    return k; 
}



//////////// Graph functions

/*
 *      raw_graph is a vector where each element is a row of the file (where the graph is stored)
 *
 * */
Graph::Graph(std::vector<std::string> raw_graph) {
    max_cost = 0;
    load_graph(raw_graph);
    find_name_nodes();
    class_not_ready = false;
}

Graph::Graph(std::string name_file_graph) {
    if (!API_S::exist_file(name_file_graph)) {
        std::cerr << "File [" << name_file_graph << "] doesn't exist" << std::endl;
        class_not_ready = true;
        return;
    } else class_not_ready = false;

    std::vector<std::string> raw_graph = API_S::read_file(name_file_graph);
    max_cost = 0;
    load_graph(raw_graph);
    find_name_nodes();
}

bool Graph::is_class_not_ready(std::string possible_error) {
    if (!class_not_ready) return false;
    std::cerr << "An error occured using this class. Check documentation" << std::endl;
    std::cerr << possible_error << std::endl;
    return true;
}

Graph::~Graph() {
    graph.clear();
    name_nodes.clear();
}

/*
 *      It use Backward Value Iteration to search optimal cost
 *          It gives us all the optimal cost (from all the nodes) to go in a single node
 *
 * */
void Graph::compute_optimal_cost_backward(int N, std::string node_xf) {
    if (is_class_not_ready("Suggestion: syntax file error or file was not uploaded")) return;

    /////////////////////////////////////// input check
    if (!exist_node(node_xf)) {
        std::cerr << "Error! Arrival node [" << node_xf << "] doesn't exist" << std::endl;
        print_graph();
        class_not_ready = true;
        return;
    } else class_not_ready = false;

    this->N = N;

    /////////////////////////////////////// initialize structure
    G_k.reserve(N + 1);
    for (int i=N; i >= 0; i-- ) {
        G_k.push_back( G_opt_k(i+1, N) );
    }

    for (int i=N; i >= 0; i-- ) {
        for (unsigned int j=0; j<name_nodes.size(); j++) {
            G_k.at(i).new_node(name_nodes.at(j), -1 );  // initial cost = -1, we are going to change it
            if (i == N) {   // initialize first row of the table
                if (!G_k.at(i).get_node(j).compare(node_xf)) {  // se è il nodo a cui vogliamo arrivare
                    G_k.at(i).set_cost_node(j, 0);   // ha un costo nullo
                } else {
                    G_k.at(i).set_cost_node(j, max_cost); // altrimenti massimo
                    G_k.at(i).set_max_cost(max_cost);
                }
            } else {
                G_k.at(i).set_cost_node(j, -1);
                G_k.at(i).set_max_cost(max_cost);
            }
        }
    }

    /////////////////////////////////////// start algorithm

    // NOTE                                                                                             Backward Value Iteration is here implemented
    for (int i=N-1; i >= 0; i--) {  // for each G_k
        //std::cout << "####################### k = " << (N-i)   << endl;


        for (unsigned int j=0; j<name_nodes.size(); j++) { // Tutti gli stati
            int optimal_cost = max_cost;
            std::string actual_node = G_k.at(i).get_node(j);
            //cout << "Guardero' tra tutti i nodi la stella uscente al nodo " << actual_node << endl;
            //cout << "We are going to see all the adiacent node " << actual_node << endl;

            for (Node n:graph) {
                //cout << n.get_tail() << " ---> " << n.get_head() << endl;
                if (!n.get_tail().compare(G_k.at(i).get_node(j))) { // qui ho la stella uscente del nodo che sto analizzando
                    //cout << "\t " << n.get_tail() << " ---> " << n.get_head()  << "  appartiene alla stella uscente al nodo " << actual_node << " con costo: " <<  n.get_cost()  << endl;
                    
                    // calcolo costo ottimo: 
                    int cost_to_go = G_k.at(i + 1).get_cost_node( get_id( n.get_head() ) );
                    int G_kp1 = n.get_cost() + cost_to_go;  //G_(k+1) (Xn) Xn stato n-esimo
                    //cout << "\t\tcost to go: " << cost_to_go << endl;
                    //cout << "\t\tarch cost: " << n.get_cost() << endl;
                    
                    
                    if (G_kp1 < optimal_cost) {
                        //cout << "\t\t\til costo ottimo è finito quindi salvo "<< n.get_head() << " come parent ottimo" <<endl;
                        G_k.at(i).set_parent_node(j, n.get_head());
                    }

                    optimal_cost = std::min(optimal_cost, G_kp1 );
                    G_k.at(i).set_cost_node(j, optimal_cost);
                    
                    // debug:
                    //cout << "\t\til costo ottimo è: " << optimal_cost << endl;
                    //cout << "\t\tpreso perchè (k+1) = " << to_string(i+1) << ". G_k(id:" << to_string(j) << "). G_k+1("<< n.get_head() << ") = " << G_k.at(i+1).get_cost_node(  get_id(n.get_head())) << " a cui sommo: " << n.get_cost() << endl;
                    //cout << "\t\tG_" << i << "(id_stato_k: " << get_id(n.get_tail()) << ") = min{"<< optimal_cost << ", (costo arco=)" << n.get_cost() << " + (G_k+1)=" << G_k.at(i+1).get_cost_node( get_id(n.get_head()) )   <<  " . G_k+1 punta allo stato " 
                    //    << n.get_head() << " che ha id " << get_id(n.get_head()) << endl;
                }
            }
        }
    }

    // Show table with optimal cost

    std::cout << "Table with optimal cost" << std::endl;
    std::cout << "\t";
    for (unsigned int j=0; j<name_nodes.size(); j++) {
        std::cout << name_nodes.at(j) + "\t";
    }
    std::cout << std::endl;
    for (int i=N; i >= 0; i-- ) {
        std::cout << "G_" << i << "\t";
        for (unsigned int j=0; j<name_nodes.size(); j++) {
            std::cout << G_k.at(i).get_cost_node_print(j) << "\t";
        }
        std::cout << std::endl;
    }

    //Show table with optimal parents 
    std::cout << "Table with optimal parents" << std::endl;
    std::cout << "\t";
    for (unsigned int j=0; j<name_nodes.size(); j++) {
        std::cout << name_nodes.at(j) + "\t";
    }
    std::cout << std::endl;
    for (int i=N; i >= 0; i-- ) {
        std::cout << "k = " << (N-i)<<"\t";
        for (unsigned int j=0; j<name_nodes.size(); j++) {
            std::cout << G_k.at(i).get_optimal_parent(j) << "\t";
        }
        std::cout << std::endl;
    }
}
/*
 *      It use Forward Value Iteration to search optimal cost
 *          It gives us all the optimal cost (from one nodes) to go in all the nodes
 *
 * */
void Graph::compute_optimal_cost_forward(int N, std::string node_xi) {
    if (is_class_not_ready("Suggestion: syntax file error or file was not uploaded")) return;

    /////////////////////////////////////// input check
    if (!exist_node(node_xi)) {
        std::cerr << "Error! Arrival node [" << node_xi << "] doesn't exist" << std::endl;
        print_graph();
        class_not_ready = true;
        return;
    } else class_not_ready = false;

    this->N = N;

    /////////////////////////////////////// initialize structure
    G_k.reserve(N);
    for (int i=0; i < N; i++ ) {
        G_k.push_back( G_opt_k(i, N) );
    }

    for (int i=0; i < N; i++ ) {
        for (unsigned int j=0; j<name_nodes.size(); j++) {
            G_k.at(i).new_node( name_nodes.at(j), -1 );  // initial cost = -1, we are going to change it
            if (i == 0) {   // initialize first row of the table
                if (!G_k.at(i).get_node(j).compare(node_xi)) {  // se è il nodo da cui vogliamo partire
                    G_k.at(i).set_cost_node(j, 0);   // ha un costo nullo
                } else {
                    G_k.at(i).set_cost_node(j, max_cost); // altrimenti massimo
                    G_k.at(i).set_max_cost(max_cost);
                }
            } else {
                G_k.at(i).set_cost_node(j, -1);
                G_k.at(i).set_max_cost(max_cost);
            }
        }
    }

    /////////////////////////////////////// start algorithm

    // NOTE                                                                                             Forward Value Iteration is here implemented
    for (int i=1; i < N; i++) {  // for each G_k
        //std::cout << "####################### k = " << i << std::endl;

        for (unsigned int j=0; j<name_nodes.size(); j++) { // Tutti gli stati
            int optimal_cost = max_cost;
            std::string actual_node = G_k.at(i).get_node(j);
            //cout << "Guardero' tra tutti i nodi la stella uscente al nodo " << actual_node << endl;
            //cout << "We are going to see all the adiacent node " << actual_node << endl;

            for (Node n:graph) {
                //std::cout << n.get_tail() << " ---> " << n.get_head() << std::endl;
                if (!n.get_head().compare(G_k.at(i).get_node(j))) { // qui ho la stella uscente del nodo che sto analizzando
                    //std::cout << "\t " << n.get_tail() << " ---> " << n.get_head()  << "  appartiene alla stella entrante al nodo " << actual_node << " con costo: " <<  n.get_cost()  << std::endl;

                    // calcolo costo ottimo: 
                    int cost_to_come = G_k.at(i - 1).get_cost_node( get_id( n.get_tail() ) );
                    int G_kp1 = n.get_cost() + cost_to_come;  //G_(k-1) (Xn) Xn stato n-esimo
                    //std::cout << "\t\tcost to go: " << cost_to_come << std::endl;
                    //std::cout << "\t\tarch cost: " << n.get_cost() << std::endl;

                    if (G_kp1 < optimal_cost) {
                        //cout << "\t\t\til costo ottimo è finito quindi salvo "<< n.get_head() << " come parent ottimo" <<endl;
                        G_k.at(i).set_parent_node(j, n.get_tail());
                    }

                    optimal_cost = std::min(optimal_cost, G_kp1 );
                    G_k.at(i).set_cost_node(j, optimal_cost);

                    // debug:
                    //cout << "\t\til costo ottimo è: " << optimal_cost << endl;
                    //cout << "\t\tpreso perchè (k+1) = " << to_string(i+1) << ". G_k(id:" << to_string(j) << "). G_k+1("<< n.get_head() << ") = " << G_k.at(i+1).get_cost_node(  get_id(n.get_head())) << " a cui sommo: " << n.get_cost() << endl;
                    //cout << "\t\tG_" << i << "(id_stato_k: " << get_id(n.get_tail()) << ") = min{"<< optimal_cost << ", (costo arco=)" << n.get_cost() << " + (G_k+1)=" << G_k.at(i+1).get_cost_node( get_id(n.get_head()) )   <<  " . G_k+1 punta allo stato " 
                    //    << n.get_head() << " che ha id " << get_id(n.get_head()) << endl;
                }
            }
        }
    }

    // Show table with optimal cost

    std::cout << "Table with optimal cost" << std::endl;
    std::cout << "\t";
    for (unsigned int j=0; j<name_nodes.size(); j++) {
        std::cout << name_nodes.at(j) + "\t";
    }
    std::cout << std::endl;
    for (int i=0; i < N; i++ ) {
        std::cout << "G_" << i << "\t";
        for (unsigned int j=0; j<name_nodes.size(); j++) {
            std::cout << G_k.at(i).get_cost_node_print(j) << "\t";
        }
        std::cout << std::endl;
    }

    //Show table with optimal parents 
    std::cout << "Table with optimal parents" << std::endl;
    std::cout << "\t";
    for (unsigned int j=0; j<name_nodes.size(); j++) {
        std::cout << name_nodes.at(j) + "\t";
    }
    std::cout << std::endl;
    for (int i=0; i < N; i++ ) {
        std::cout << "k = " << i<<"\t";
        for (unsigned int j=0; j<name_nodes.size(); j++) {
            std::cout << G_k.at(i).get_optimal_parent(j) << "\t";
        }
        std::cout << std::endl;
    }
}

/*
 *  Check if a name is a node name  or
 *  Check if a node exist
 * */
bool Graph::exist_node(std::string name) {
    if (is_class_not_ready("Suggestion: syntax file error or file was not uploaded")) return false;

    for (unsigned int j=0; j<name_nodes.size(); j++) {
        if (!name.compare(name_nodes.at(j))) return true;
    }
    return false;
}

/*
 *              GET OPTIMAL PATH  (EXAMPLE FUNCTION)
 *  Call it AFTER call "compute_optimal_cost_forward" function
 *  return the optimal path in a string just for print it
 *
 * */
std::string Graph::_get_optimal_path_forward(std::string arrival_node) {
    if (is_class_not_ready("Suggestion: file was not uploaded or you forgot to call 'compute_optimal_cost' before '_get_optimal_path()'")) return "Error, view log";
    std::string path_opt = arrival_node;
    std::string cost = G_k.at(G_k.size()-1).get_cost_node_print(get_id(arrival_node));

    std::string last_stored_node = "";

    for (int i=N-1; i >0; i--) {
        for (unsigned int j=0; j<name_nodes.size(); j++) {
            std::string j_th_node = name_nodes.at(j);
            std::string its_optimal_parent = G_k.at(i).get_optimal_parent(j);

            if (!j_th_node.compare(arrival_node)) {
                if (!its_optimal_parent.compare(last_stored_node)) {
                    std::cout << "We are already arrived" << std::endl;
                    return path_opt + " with total cost " + cost;
                }
                last_stored_node = its_optimal_parent;
                path_opt += " <-- " + its_optimal_parent;
                arrival_node = its_optimal_parent;
                break;
            }
        }
    }
    return path_opt + " with total cost " + cost;
}

std::vector<Node> Graph::reverse_vector(std::vector<Node> vect) {
    std::vector<Node> new_vec;

    for (int i=vect.size()-1; i >= 0; i--) {
        new_vec.push_back(vect.at(i));
    }
    return new_vec;
}

/*
 *              GET OPTIMAL PATH 
 *  Call it AFTER call "compute_optimal_cost_forward" function
 *  return the optimal path in a vector
 *
 * */
std::vector<Node> Graph::get_optimal_path_forward(std::string arrival_node) {
    if (is_class_not_ready("Suggestion: file was not uploaded or you forgot to call 'compute_optimal_cost' before 'get_optimal_path()'")) return std::vector<Node>();

    std::vector<Node> optimal_path;
    optimal_path.push_back(Node(arrival_node, arrival_node, 0));

    std::string last_stored_node = "";

    for (int i=N-1; i >0; i--) {
        for (unsigned int j=0; j<name_nodes.size(); j++) {
            std::string j_th_node = name_nodes.at(j);
            std::string its_optimal_parent = G_k.at(i).get_optimal_parent(j);

            if (!j_th_node.compare(arrival_node)) {
                if (!its_optimal_parent.compare(last_stored_node)) {
                    std::cout << "We are already arrived" << std::endl;
                    return reverse_vector(optimal_path);
                    //std::reverse(myvector.begin(),myvector.end());
                }
                last_stored_node = its_optimal_parent;
                optimal_path.push_back(Node(its_optimal_parent, its_optimal_parent, 0));
                arrival_node = its_optimal_parent;
                break;
            }
        }
    }
    return reverse_vector(optimal_path);
}

/*
 *              GET OPTIMAL PATH  (EXAMPLE FUNCTION)
 *  Call it AFTER call "compute_optimal_cost_backward" function
 *  return the optimal path in a string just for print it
 *
 * */
std::string Graph::_get_optimal_path_backward(std::string starting_node) {
    if (is_class_not_ready("Suggestion: file was not uploaded or you forgot to call 'compute_optimal_cost' before '_get_optimal_path_backward()'")) return "Error, view log";
    std::string path_opt = starting_node;
    std::string cost = G_k.at(1).get_cost_node_print(get_id(starting_node));

    std::string last_stored_node = "";

    for (int i=0; i < N; i++) {
        for (unsigned int j=0; j<name_nodes.size(); j++) {
            std::string j_th_node = name_nodes.at(j);
            std::string its_optimal_parent = G_k.at(i).get_optimal_parent(j);
            
            if (!j_th_node.compare(starting_node)) {  
                if (!its_optimal_parent.compare(last_stored_node)) {
                    std::cout << "We are already arrived" << std::endl;
                    return path_opt + " with total cost " + cost;
                }
                last_stored_node = its_optimal_parent;
                path_opt += " --> " + its_optimal_parent;
                starting_node = its_optimal_parent;
                break;
            }
        }
    }
    return path_opt + " with total cost " + cost;
}

/*
 *              GET OPTIMAL PATH 
 *  Call it AFTER call "compute_optimal_cost" function
 *  return the optimal path in a vector
 *
 * */
std::vector<Node> Graph::get_optimal_path_backward(std::string starting_node) {
    if (is_class_not_ready("Suggestion: file was not uploaded or you forgot to call 'compute_optimal_cost' before 'get_optimal_path()'")) return std::vector<Node>();

    std::vector<Node> optimal_path;
    optimal_path.push_back(Node(starting_node, starting_node, 0));

    std::string last_stored_node = "";

    for (int i=0; i < N; i++) { 
        for (unsigned int j=0; j<name_nodes.size(); j++) {
            std::string j_th_node = name_nodes.at(j);
            std::string its_optimal_parent = G_k.at(i).get_optimal_parent(j);

            if (!j_th_node.compare(starting_node)) { // 
                if (!its_optimal_parent.compare(last_stored_node)) {
                    std::cout << "We are already arrived" << std::endl;
                    return optimal_path;
                }
                last_stored_node = its_optimal_parent;
                optimal_path.push_back(Node(its_optimal_parent, its_optimal_parent, 0));
                starting_node = its_optimal_parent;
                break;
            }
        }
    }
    return optimal_path;
}

/*
 *  get node id by name
 * */
int Graph::get_id(std::string name_node) {
    if (is_class_not_ready("Suggestion: syntax file error")) return -1;
    for (unsigned int i=0; i<graph.size(); i++) {
        if (!graph.at(i).get_tail().compare(name_node)) 
            return graph.at(i).get_id();
    }
    return -1;
}

/*
 *  We need to get name nodes and set them an id
 *
 * */
void Graph::find_name_nodes() {
    // sort and remove duplicate names
    sort( name_nodes.begin(), name_nodes.end() );
    name_nodes.erase( unique( name_nodes.begin(), name_nodes.end() ), name_nodes.end() );

    //cout << "name nodes: " << endl;
    //for (int i=0; i<name_nodes.size(); i++) {
    //    cout << name_nodes.at(i)  <<endl;
    //}

    //cout << "Set id: " << endl;
    int id = 0;
    for (unsigned int i=0; i<graph.size(); i++) {
        //cout << "nodo " << graph.at(i).get_tail() << " ha id: " << graph.at(i).id << endl; 
        if (graph.at(i).get_id() == -1) {
            bool node_already_initialized = false;
            for (unsigned int j=0; j<i; j++) { // quando arrivo qui devo guardare i nodi su cui sono stato prima 
                // e vedere se il nodo che i-esimo che sto guardando adesso è già inizializzato precedentemente
                //cout << "\t cerco il nodo di partenza: " << graph.at(j).get_tail() << " vs " << graph.at(i).get_tail() << endl;
                if (!graph.at(j).get_tail().compare(graph.at(i).get_tail())) { // qui ho trovato che il nodo che sto guardando, l'ho già inizializzato!
                    //cout << "\t\t l'ho trovato, quindi non creo un nuovo id ma gli do il precedente" << endl;
                    graph.at(i).set_id(graph.at(j).get_id());
                    node_already_initialized = true;
                    break;
                } //else cout << "\t\t quindi non lo aggiorno"<< endl;

            }
            if (!node_already_initialized) {
                graph.at(i).set_id(id);
                id++;
            }
            //cout << "il nodo " << graph.at(i).get_tail() << " ha alla fine id: " << graph.at(i).id << endl; 
        }
    }

    // print id nodi
    //for (int i=0; i<graph.size(); i++) {
    //    cout << "nodo " << graph.at(i).get_tail() << " ha id: " << graph.at(i).get_id() << endl;
    //}
}

/*
 *  Just print the graph as you wrote it
 * */
void Graph::print_graph() {
    if (is_class_not_ready("Suggestion: syntax file error")) return;
    std::cout << "print_graph(): " << std::endl;
    for (Node n:graph) {
        std::cout << n.to_string();
    }
}

/*
 * Interpret graph
 * row file: tail,head,cost  type:  string,string,integer
 *
 * Example
 * row file:    A,B,5
 *              B,A,2
 *              B,C,10
 *              C,A,3
 *
 *  Your graph is like this:
 *                           
 *            5         10
 *        -------->   ----->
 *     (A)         (B)     (C)    
 *        <-------              
 *      ^     2             |
 *      |--------------------
 *                 3
 * */
void Graph::load_graph(std::vector<std::string> all_nodes) {
    for (unsigned int i=0; i<all_nodes.size(); i++) {
        // mi aspetto ogni riga del tipo: tail,head,costo
        int v1 = all_nodes.at(i).find(",");     // searching the comma after tail
        //cout << "tail: " << all_nodes.substr(0, v1) << endl;
        std::string tail = all_nodes.at(i).substr(0, v1);
        all_nodes.at(i).erase(0, v1 + 1);

        int v2 = all_nodes.at(i).find(",");     // searching the comma after head
        std::string head = all_nodes.at(i).substr(0, v2);
        all_nodes.at(i).erase(0, v2 + 1);

        int cost = stoi(all_nodes.at(i));
        max_cost = std::max(max_cost, cost);

        graph.push_back( Node(head, tail, cost) );
        name_nodes.push_back(head);
    }
    max_cost *= 10;
}


