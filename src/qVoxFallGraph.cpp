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

#include "qVoxFallGraph.h"
#include "omp_num_threads.hpp"

#include <numeric>


qVoxFallGraph::qVoxFallGraph(Tuple3ui shape, uint8_t connectivity)
    : rV(1), last_rV(0), rE(0),
      comp_assign(nullptr), last_comp_assign(nullptr),
      comp_list(nullptr), first_vertex(nullptr),
      reduced_edges(nullptr)
{
    this->shape = shape;

    /* build the graph */
    constexpr size_t D = 3;
	uint32_t steps[3] = { shape.x, shape.y, shape.z };
	N = steps[0] * steps[1] * steps[2];
	E = num_edges_grid_graph(D, steps, connectivity);

    /* edge activation */
    edge_status = (Edge_status*) malloc_check(sizeof(Edge_status)*E);
    for (uint32_t e = 0; e < E; e++){ bind(e); }

	/* create edge list */
    edges.resize(2 * E);
	std::vector<uint8_t>  connectivities(E);
	edge_list_grid_graph(
        D, 
        steps, 
        connectivity, 
        edges.data(), 
        connectivities.data(), 
        (uint32_t)0, 
        (uint32_t)0, 
        (uint8_t)0, 
        false
    );

	/* Forward-star representation */
	first_edge.resize(N + 1);
	std::vector<uint32_t> reindex(E);
	edge_list_to_forward_star<uint32_t, uint32_t>(
		N,
		E,
		edges.data(),
		first_edge.data(),
		reindex.data()
	);
    
	/* Permute target vertices into forward-star order */
	target.resize(E);
	for (size_t e = 0; e < E; ++e)
	{
		target[reindex[e]] = edges[2 * e + 1];
	}
}


qVoxFallGraph::~qVoxFallGraph()
{
    free(edge_status);
    free(comp_assign);
    free(last_comp_assign);
    free(comp_list);
    free(first_vertex);
    // free(reduced_edges);
}


int qVoxFallGraph::Grid2Index(Tuple3i n)
{
	int i = n.x;
	int j = n.y;
	int k = n.z;

	int x = shape.x;
    int y = shape.y;
    int z = shape.z;
    
    int index = (i)+(j * x) + (k * x * y);
	return index;
}
    

Tuple3i qVoxFallGraph::Index2Grid(unsigned index)
{
	int x = shape.x;
    int y = shape.y;
    int z = shape.z;
    
    int k = std::floor(index / (y * x));
	int remain = index - (y * x * k);
	int j = std::floor(remain / x);
	int i = remain - (x * j);

	Tuple3i V(	static_cast<int>(i),
				static_cast<int>(j),
				static_cast<int>(k)	);
	return V;
}


std::vector<uint32_t> qVoxFallGraph::GetNeighbors(uint32_t index) const
{
    uint32_t first_nn = first_edge[index];
    uint32_t last_nn = first_edge[index + 1];

    std::vector<uint32_t> neighbours;
    neighbours.reserve(last_nn - first_nn);
    for (uint32_t i = first_nn; i < last_nn; ++i)
    {
        neighbours.push_back(target[i]);
    }
    return neighbours;
}


void qVoxFallGraph::FlagEdgesNodeMatch(const std::vector<bool>& mask)
{
    #pragma omp parallel for schedule(static) NUM_THREADS(E, N)
	for (uint32_t n = 0; n < N; n++){
		for (uint32_t e = first_edge[n]; e < first_edge[n + 1]; e++){
			if (mask[n] != mask[target[e]]){ cut(e); }
		}
	}
}


void qVoxFallGraph::compute_connected_components()
{
    /**Union-Find arrays **/
    std::vector<uint32_t> parent(N), rnk(N, 0u);
    std::iota(parent.begin(), parent.end(), 0u);

    /* find with path-halving */
    auto find = [&](uint32_t x) -> uint32_t {
        while (parent[x] != x) {
            parent[x] = parent[parent[x]];
            x = parent[x];
        }
        return x;
    };

    /* union by rank */
    auto unite = [&](uint32_t a, uint32_t b) {
        uint32_t ra = find(a), rb = find(b);
        if (ra == rb) { return; }
        if (rnk[ra] < rnk[rb]) { std::swap(ra, rb); }
        parent[rb] = ra;
        if (rnk[ra] == rnk[rb]) { rnk[ra]++; }
    };

    /* unite vertices connected by BIND edges */
    for (uint32_t v = 0; v < N; v++) {
        for (uint32_t e = first_edge[v]; e < first_edge[v + 1]; e++) {
            if (is_bind(e)) { unite(v, target[e]); }
        }
    }

    /* assign contiguous component ids */
    free(comp_assign);
    comp_assign = static_cast<uint32_t*>(malloc_check(sizeof(uint32_t) * N));

    std::vector<uint32_t> root_to_comp(N, UINT32_MAX);
    rV = 0;
    for (uint32_t v = 0; v < N; v++) {
        uint32_t root = find(v);
        if (root_to_comp[root] == UINT32_MAX) {
            root_to_comp[root] = rV++;
        }
        comp_assign[v] = root_to_comp[root];
    }

    /* build first_vertex (cumulative count) */
    free(first_vertex);
    first_vertex = static_cast<uint32_t*>(
        malloc_check(sizeof(uint32_t) * (rV + 1)));
    std::fill(first_vertex, first_vertex + rV + 1, 0u);
    for (uint32_t v = 0; v < N; v++) { first_vertex[comp_assign[v] + 1]++; }
    for (uint32_t rv = 0; rv < rV; rv++) {
        first_vertex[rv + 1] += first_vertex[rv];
    }

    /* build comp_list */
    free(comp_list);
    comp_list = static_cast<uint32_t*>(malloc_check(sizeof(uint32_t) * N));
    {
        std::vector<uint32_t> pos(rV);
        for (uint32_t rv = 0; rv < rV; rv++) { pos[rv] = first_vertex[rv]; }
        for (uint32_t v = 0; v < N; v++) {
            comp_list[pos[comp_assign[v]]++] = v;
        }
    }
};