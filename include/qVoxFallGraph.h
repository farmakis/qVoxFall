//##########################################################################
//#                                                                        #
//#                     CLOUDCOMPARE PLUGIN: qVoxFall                      #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 3 of the License.               #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#                 COPYRIGHT: THE UNIVERSITY OF NEWCASTLE                 #
//#                                                                        #
//##########################################################################

#ifndef Q_VOXFALL_GRAPH_HEADER
#define Q_VOXFALL_GRAPH_HEADER

#include <vector>
#include <cstdint>
#include <cstdlib>

#include <CCGeom.h>
#include "grid_graph.hpp"


class qVoxFallGraph
{
public:	

	/*  constructor, destructor  */

	qVoxFallGraph(Tuple3ui shape, uint8_t connectivity);
	 /* the destructor frees arrays of ... edges, first_edge, target, edge_status */
	~qVoxFallGraph();


	/* the 'get' methods */
	uint32_t GetNumNodes() const { return N; }
	uint32_t GetNumEdges() const { return E; }
    uint32_t GetNumReducedNodes() const { return rV; }
    uint32_t GetNumReducedEdges() const { return rE; }
    uint32_t GetNodeIndex(int i) { return comp_list[i]; }
	std::vector<uint32_t> GetEdges() const { return edges; }
	std::vector<uint32_t> GetFirstEdge() const { return first_edge; }
	std::vector<uint32_t> GetTarget() const { return target; }
	std::vector<uint32_t> GetNeighbors(uint32_t nodeIndex) const;
    uint32_t* GetLabels() const { return comp_assign; }
    uint32_t GetFirstNode(int i) const { return first_vertex[i]; }
    uint32_t GetLastNode(int i) const { return first_vertex[i + 1]; }

	/**  methods for manipulating parameters  **/
	/* Flags graph edges based on node matching 
	 * 'CUT' edges that connect nodes with different mask values */
	void FlagEdgesNodeMatch(const std::vector<bool>& mask);

    int Grid2Index(Tuple3i n);
	Tuple3i Index2Grid(unsigned index);

    /* update connected components */
    void compute_connected_components();

protected:
	/**  main graph  **/

	Tuple3ui shape; // number of nodes in each dimension (x, y, z)
    std::vector<uint32_t> edges; // edge list (2*E array)
    uint32_t N, E; // number of nodes, of edges

	/**  forward-star graph representation  **/
    /* - edges are numbered such as all edges originating from a same vertex
     * are consecutive;
     * - for each vertex, 'first_edge' indicates the first edge starting
     * from the vertex (or, if there are none, starting from the next vertex);
     * array of length N + 1, the first value is always zero and the last
     * value is always the total number of edges E
     * - for each edge, 'target' indicates its ending node */
	std::vector<uint32_t> first_edge, target; 

	/**  reduced graph  **/
	uint32_t rV, last_rV; // number of components (reduced vertices)
	uint32_t rE; // number of reduced edges
	/* assignment of each vertex to a component */
    uint32_t* comp_assign, *last_comp_assign;
    /* list the vertices of each components:
     * - vertices are gathered in 'comp_list' so that all vertices belonging
     * to a same components are consecutive
     * - for each component, 'first_vertex' indicates the index of its first
     * vertex in 'comp_list' */
    uint32_t *comp_list, *first_vertex;

	/* reduced connectivity
     * reduced edges represented with edges list (array of size twice the 
     * number of reduced edges, consecutive indices are linked components)
     * guarantees:
     * 1) starting component identifiers are smaller than ending components
     * 2) increasing order of starting and ending components identifiers
     *  (this eases some routines, like conversion to forward-star)
     * 3) each edge appears only once
     * 4) isolated components (not linked to any other component) are linked
     *  to themselves with epsilon reduced weight */
    uint32_t* reduced_edges;

	/* methods for setting and checking edge status */
    bool is_cut(uint32_t e) const // check if edge e is cut (active)
        { return edge_status[e] == CUT; }
    bool is_bind(uint32_t e) const // check if edge e is binding (inactive)
        { return edge_status[e] == BIND; }
    void cut(uint32_t e) // flag a cut (active) edge
        { edge_status[e] = CUT; }
    void bind(uint32_t e) // flag a binding (inactive) edge
        { edge_status[e] = BIND; }

	/* allocate memory and fail with error message if not successful */
    static void* malloc_check(size_t size)
    {
        void *ptr = malloc(size);
        if (!ptr){
            // std::cout << "Cut-pursuit: not enough memory." << std::endl;
            exit(EXIT_FAILURE);
        }
        return ptr;
    }

private:
	enum Edge_status : char // requires C++11 to ensure 1 byte
        {BIND, CUT};
    Edge_status* edge_status; // edge activation

};

#endif //Q_VOXFALL_GRAPH_HEADER