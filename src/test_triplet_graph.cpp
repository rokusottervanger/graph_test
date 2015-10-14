#include <stdio.h>
#include <iostream>

#include <triplet_graph/Graph.h>

int main(int argc, char** argv)
{
    // Instantiate graph
    triplet_graph::Graph graph;

    // Add nodes to graph
    int n1 = graph.addNode("node1");
    int n2 = graph.addNode("node2");
    int n3 = graph.addNode("node3");

    // Interconnect nodes using Edge2s
    {
        double l1 = 1.0;
        double l2 = 2.0;
        double l3 = 3.0;

        graph.addEdge2(n1,n2,l1);
        graph.addEdge2(n2,n3,l2);
        graph.addEdge2(n3,n1,l3);
    }

    // Interconnect nodes using an Edge3
    graph.addEdge3(n1,n2,n3);
}

// Retrieve image from sensor
// Obtain feature points
// Add nodes for all feature points
// for point in all feature points:
//      Add edge2s from point to all other feature points
//      add edge3s from point to all other feature points with all other feature points

// NEED AN EFFICIENT WAY TO CHECK IF AN EDGE EXISTS AND WHERE TO FIND IT, RIGHT?