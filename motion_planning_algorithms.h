
/*
 *
 *      Author: stefano
 *
 */

#ifndef _B_AND_F_VALUE_ITERATION_H
#define _B_AND_F_VALUE_ITERATION_H

////////////////////////   Dependencies

class API_S {
    public:
        static int16_t get_size_file(std::string file_to_read);
        static bool exist_file(std::string file_to_read);
        static std::vector<std::string> read_file(std::string file_to_read);
};

//////////////////////// end dependencies

class Node {
    private:
        std::string node_t, node_c;    // tail - > head    (arc)
        int cost;                       // arc's cost 
        int id;

    public:
        Node();
        Node(std::string node_t, std::string node_c, uint16_t cost);

        std::string to_string();
        int get_cost();
        std::string get_tail();
        std::string get_head();
        int get_id();
        void set_id(int id);
};

class Row {
    public:
        int cost;
        std::string node;
        std::string  optimal_parent;

        Row(int cost, std::string node);
};

class G_opt_k {
    private:
        int k;
        int max_cost;
    public:
        std::vector <Row> row;   // it has size = N
        G_opt_k(int k, int N);
        ~G_opt_k();

        void set_max_cost(int _max_cost);
        std::string get_node(int k);
        void new_node(std::string node, int cost);
        void set_cost_node(int id_node, int cost);
        void set_parent_node(int id_node, std::string parent);
        std::string get_optimal_parent(int id_node);
        int get_cost_node(int id_node);
        std::string get_name_node(int id_node);
        std::string get_cost_node_print(int id_node);
        int get_k();
};


class Graph {
    private:
        std::vector<Node> graph;
        std::vector<std::string> name_nodes;
        int max_cost;
        int N;
        std::vector<G_opt_k> G_k;
        bool class_not_ready;   // If this value is true you are wrong to use this class. Check documentation

    private:
        void find_name_nodes();
        void load_graph(std::vector<std::string> all_nodes);
        bool is_class_not_ready(std::string);
        std::vector<Node> reverse_vector(std::vector<Node> vect);

    public:
        Graph(std::vector<std::string> raw_graph);
        Graph(std::string name_file_graph);
        ~Graph();

        void compute_optimal_cost(int N, std::string node_xf);
        void compute_optimal_cost_backward(int N, std::string node_xf);
        void compute_optimal_cost_forward(int N, std::string node_xi);
        std::string _get_optimal_path_forward(std::string arrival_node);
        std::vector<Node> get_optimal_path_forward(std::string arrival_node);
        std::string _get_optimal_path_backward(std::string starting_node);
        std::vector<Node> get_optimal_path_backward(std::string starting_node);
        int get_id(std::string name_node);
        void print_graph();
        bool exist_node(std::string name);
};

#endif
