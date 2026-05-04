// =============================================================================
//
//  This project implements a global router to connect components on a chip
//  while avoiding traffic jams (congestion). Here is a simple breakdown of 
//  the techniques used and what the methods in this code actually do
//
//  Grid & Edge Helpers (edgeIdFromPoints, getEdgeOverflow): 
//  Instead of using slow 2D matrices, the map is flattened into a 1D array for 
//  speed. These methods convert coordinates and check if an edge is over capacity.
//
//  Cost & History Tracking (updateSingleEdgeWeight, updateEdgeHistory...): 
//  These methods calculate how expensive a path is. 
//  If an edge is exactly full, we make it cost more so 
//  new wires avoid it. If an edge overflows, it gets a historical penalty 
//  that slowly fades away if the edge becomes clean later.
//
//  Multi-Pin Routing (computePinMST): 
//  For nets with 3 or more pins, we create a Minimum 
//  Spanning Tree. This connects the pins using the shortest possible 
//  distances rather than connecting them in a random, inefficient chain.
//
//  Pathfinding (rerouteSegmentAStar): 
//  This is the core A* search algorithm. It finds the cheapest path between 
//  two points. To make it incredibly fast, it uses a dynamic bounding box 
//  so it only searches a small local area, and it adds a small penalty for 
//  bending, which forces the router to prefer fast, straight lines.
//
//  Net Sorting (buildCongestedNetOrdering): 
//  Finds all the nets that are causing overlaps and sorts them by density, 
//  (cost squared divided by length). This guarantees the algorithm attacks the 
//  worst, most concentrated traffic jams first.
//
//  Rip-Up & Reroute (ripUpAndRerouteNetSegments, runSingleRRRIteration): 
//  Instead of ripping up an entire net, this only rips up the specific 
//  segments that are overflowing. To prevent infinite loops in the final 
//  minutes, it freezes minor overlaps so the 
//  router can focus on the remaining major blockages.
//
//  The Main Engine (solveRouting): 
//  Phase 1 routes the shortest nets first using fast Z-shapes to lock in 
//  clean paths. Phase 2 loops the Rip-Up and Reroute process for exactly 
//  5 minutes. It takes snapshots of the grid whenever it finds a better 
//  solution, guaranteeing we restore and output the absolute best result.
//
//  File I/O (readBenchmark, writeOutput):
//  Standard methods to parse the initial chip design file and write the final
//  calculated wire paths to the output file.
// =============================================================================

#include "global_router.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>
#include <algorithm>
#include <chrono>
#include <queue>
#include <cmath>
#include <climits>
#include <numeric>

static void updateSingleEdgeWeight(routingInst *rst, int eid);

// Global settings changed by command line arguments
static int gEnablePinOrdering = 0;
static int gEnableRRR         = 0;

// Variables used to control the A* weight (how greedy it is)
static int gHNum = 5; //numerator
static int gHDen = 4; //denominator
// weight = gHNum / gHDen. larger means faster(greedier) and smaller means more perfect-searching. 
// so think of it as like gears in cars, start out with big weight and as the board gets more
// organized, try to look for more perfectness.

// d and n
void setRoutingMode(int dFlag, int nFlag) {
  gEnablePinOrdering = (dFlag != 0) ? 1 : 0;
  gEnableRRR         = (nFlag != 0) ? 1 : 0;
}

// Flatten 2D coordinates (x,y) into a single 1D array number
static int edgeIdFromPoints(point a, point b, int gx, int gy) {
  // Always make sure 'a' is the bottom-left point to keep math consistent
  if (a.x == b.x && a.y > b.y) { point t = a; a = b; b = t; }
  if (a.y == b.y && a.x > b.x) { point t = a; a = b; b = t; }
  
  // Horizontal edge calculation
  if (a.y == b.y && b.x == a.x + 1) return a.y * (gx - 1) + a.x;
  // Vertical edge calculation
  if (a.x == b.x && b.y == a.y + 1) return gy * (gx - 1) + a.y * gx + a.x;
  
  return -1; // Return -1 if points don't form a valid straight edge
}

// Reverses the math: takes a 1D ID and gives back the 2D coordinates
static int edgePointsFromId(int eid, int gx, int gy, point *a, point *b) {
  if (!a || !b || eid < 0) return 0;
  int numH = gy * (gx - 1);
  
  if (eid < numH) {
    // It's a horizontal edge
    int y = eid / (gx - 1), x = eid % (gx - 1);
    a->x = x; a->y = y; b->x = x + 1; b->y = y;
    return 1;
  }
  // It's a vertical edge
  int vid = eid - numH;
  if (vid >= gx * (gy - 1)) return 0;
  int y = vid / gx, x = vid % gx;
  a->x = x; a->y = y; b->x = x; b->y = y + 1;
  return 1;
}

// Allocates blank memory arrays for the entire map grid
static void initEdgeCapsUtils(routingInst *rst) {
  // Total edges = horizontal roads + vertical roads
  rst->numEdges    = (rst->gy * (rst->gx - 1)) + (rst->gx * (rst->gy - 1));
  rst->edgeCaps    = (int*)malloc(sizeof(int) * rst->numEdges);
  rst->edgeUtils   = (int*)malloc(sizeof(int) * rst->numEdges);
  rst->edgeHistory = (int*)malloc(sizeof(int) * rst->numEdges);
  rst->edgeWeights = (int*)malloc(sizeof(int) * rst->numEdges);
  if (!rst->edgeCaps || !rst->edgeUtils || !rst->edgeHistory || !rst->edgeWeights) return;
  
  // Fill arrays with default values (empty roads)
  for (int e = 0; e < rst->numEdges; ++e) {
    rst->edgeCaps[e]    = rst->cap;
    rst->edgeUtils[e]   = 0;
    rst->edgeHistory[e] = 1; // Base history multiplier is 1
    rst->edgeWeights[e] = 1; // Base cost to travel is 1
  }
}

// Check overflow, usage - capacity. Returns 0 if there is no jam.
// overflow is when there is more usage than the capacity.
static inline int getEdgeOverflow(routingInst *rst, int eid) {
  int ov = rst->edgeUtils[eid] - rst->edgeCaps[eid];
  return ov > 0 ? ov : 0;
}

// Sets the travel cost for one specific road
// cost is 1 when there's no overflow, 2 when if it is full, and multiply by overflow
// if there is overflow with history of that route in the past.
static void updateSingleEdgeWeight(routingInst *rst, int eid) {
  int util = rst->edgeUtils[eid];
  int cap  = rst->edgeCaps[eid];
  int ov   = util - cap;
  
  if (ov > 0) {
    // Road is jammed. Multiply traffic by the road's history.
    rst->edgeWeights[eid] = 2 + (rst->edgeHistory[eid] * ov);
  } else if (util == cap && cap > 0) {
    // Road is full. Make it cost more to discourage new wires.
    rst->edgeWeights[eid] = 2; 
  } else {
    // Road is clear. Cost is normal.
    rst->edgeWeights[eid] = 1;
  }
}

// Loops through the entire map and updates the cost of every road
static void recomputeAllEdgeWeights(routingInst *rst) {
  for (int e = 0; e < rst->numEdges; ++e) updateSingleEdgeWeight(rst, e);
}

// Adds up every single traffic jam on the map to get the Total Overflow score
static int getTotalOverflow(routingInst *rst) {
  int tot = 0;
  for (int e = 0; e < rst->numEdges; ++e) tot += getEdgeOverflow(rst, e);
  return tot;
}

// Adjusts the bad history val the router holds against bad roads
static void updateEdgeHistoryFromCurrentOverflow(routingInst *rst, int iter) {
  for (int e = 0; e < rst->numEdges; ++e) {
    int ov = getEdgeOverflow(rst, e);
    if (ov > 0) {
      // Route is jammed right now. Increase its penalty for the future.
      rst->edgeHistory[e]++;
    } else if (rst->edgeHistory[e] > 1 && (iter % 3 == 0)) {
      // Every 3rd round, forgive the road slightly if it has empty space.
      if (rst->edgeCaps[e] - rst->edgeUtils[e] >= 1) {
          rst->edgeHistory[e]--; 
      }
    }
  }
}

// Connect 3+ pins together without wasting wire length
static std::vector<std::pair<int,int>> computePinMST(const net *n) {
  int k = n->numPins;
  std::vector<std::pair<int,int>> edges;
  edges.reserve(k - 1);
  if (k <= 1) return edges;

  std::vector<bool> inMST(k, false);
  std::vector<int>  minD(k, INT_MAX);
  std::vector<int>  parent(k, -1);
  minD[0] = 0; // Start at the first pin

  // Keep looping until all pins are connected to the tree
  for (int iter = 0; iter < k; ++iter) {
    int u = -1;
    // Find the closest unconnected pin
    for (int i = 0; i < k; ++i)
      if (!inMST[i] && (u == -1 || minD[i] < minD[u])) u = i;
    
    inMST[u] = true; // Mark it as connected
    if (parent[u] != -1) edges.push_back({parent[u], u}); // Save the wire
    
    const point& pu = n->pins[u];
    
    // Update the distances to the rest of the unconnected pins
    for (int v = 0; v < k; ++v) {
      if (inMST[v]) continue;
      int d = std::abs(pu.x - n->pins[v].x) + std::abs(pu.y - n->pins[v].y);
      if (d < minD[v]) { minD[v] = d; parent[v] = u; }
    }
  }
  return edges;
}

// Tracks position and cost during A* pathfinding
struct AStarNode {
  int x, y, gCost, fCost, dir;
  // Always pull the lowest costing node first
  bool operator>(const AStarNode& o) const { return fCost > o.fCost; }
};

// find the cheapest path from p1 to p2
static int rerouteSegmentAStar(segment *seg, point p1, point p2, routingInst *rst, bool isPhase1) {
  seg->p1 = p1; seg->p2 = p2;
  seg->numEdges = 0; seg->edges = NULL;
  if (p1.x == p2.x && p1.y == p2.y) return 1; // Already connected

  std::vector<int> margins;
  if (isPhase1) {
    // phase 1, search box is exactly the size of the distance. Forces fast Z-shapes.
    margins.push_back(0); 
  } else {
    // phaee 2, longer distances get a slightly bigger local search area.
    int dist = std::abs(p1.x - p2.x) + std::abs(p1.y - p2.y);
    int margin1 = std::max(2, std::min(12, dist / 4));
    margins.push_back(margin1);
    margins.push_back(margin1 + 6); // Try an even bigger box if the first one fails
  }

  // LUT for moving Up, Down, Right, Left
  static const int DX[] = {0, 0,  1, -1};
  static const int DY[] = {1, -1, 0,  0};

  for (size_t mi = 0; mi < margins.size(); ++mi) {
    const int mg = margins[mi];
    
    // Create the boundaries of the local search box
    int minX = std::max(0,           std::min(p1.x, p2.x) - mg);
    int maxX = std::min(rst->gx - 1, std::max(p1.x, p2.x) + mg);
    int minY = std::max(0,           std::min(p1.y, p2.y) - mg);
    int maxY = std::min(rst->gy - 1, std::max(p1.y, p2.y) + mg);

    int bbW  = maxX - minX + 1;
    int bbH  = maxY - minY + 1;
    int bbSz = bbW * bbH;
    
    // get local array index inside the search box
    auto bbI = [&](int x, int y) { return (x - minX) * bbH + (y - minY); };

    // Set up tracking arrays for costs and their parent coordinates
    std::vector<int> gCosts(bbSz, INT_MAX);
    struct Parent { int eid, px, py; };
    std::vector<Parent> par(bbSz, {-1, -1, -1});

    // The priority queue keeps the cheapest paths at the front
    std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> pq;
    
    gCosts[bbI(p1.x, p1.y)] = 0; // Starting point costs 0
    int h0 = (std::abs(p1.x - p2.x) + std::abs(p1.y - p2.y)) * gHNum / gHDen;
    pq.push({p1.x, p1.y, 0, h0, -1}); // push starting node

    bool found = false;
    while (!pq.empty()) {
      AStarNode cur = pq.top(); pq.pop(); // get cheapest path so far
      
      // we hit the target, stop
      if (cur.x == p2.x && cur.y == p2.y) { found = true; break; }
      
      // if we already found a cheaper way to this exact spot, skip
      int ci = bbI(cur.x, cur.y);
      if (cur.gCost > gCosts[ci]) continue;

      // try moving in all 4 directions
      for (int d = 0; d < 4; ++d) {
        int nx = cur.x + DX[d], ny = cur.y + DY[d];
        
        // stop if step goes outside our local search box boundaries
        if (nx < minX || nx > maxX || ny < minY || ny > maxY) continue;
        
        int eid = edgeIdFromPoints({cur.x, cur.y}, {nx, ny}, rst->gx, rst->gy);
        if (eid < 0 || eid >= rst->numEdges) continue;
        
        int ng = cur.gCost + rst->edgeWeights[eid]; // Add cost of new road
        int ni = bbI(nx, ny);
        
        // If the new path is cheaper than old paths to this spot, save it
        if (ng < gCosts[ni]) {
          gCosts[ni] = ng;
          par[ni] = {eid, cur.x, cur.y}; // record parents
          
          // Estimate remaining distance to target
          int h = (std::abs(nx - p2.x) + std::abs(ny - p2.y)) * gHNum / gHDen;
          
          // Add a penalty if we changed direction, force straight lines
          int bend_penalty = (cur.dir != -1 && cur.dir != d) ? 2 : 0;
          
          pq.push({nx, ny, ng, ng + h + bend_penalty, d});
        }
      }
    }

    if (!found) continue; // If the path is not found in small box, loop will try bigger box

    // Backtrack using the parent records to trace the final path
    std::vector<int> pathE;
    int cx = p2.x, cy = p2.y;
    while (!(cx == p1.x && cy == p1.y)) {
      int i = bbI(cx, cy);
      pathE.push_back(par[i].eid); // Save road ID
      int px = par[i].px, py = par[i].py;
      cx = px; cy = py; // Move backwards
    }
    std::reverse(pathE.begin(), pathE.end()); // Flip it so it goes from start to the target

    // save the finalized path into the memory
    seg->numEdges = (int)pathE.size();
    seg->edges = (int*)malloc(sizeof(int) * seg->numEdges);
    for (int i = 0; i < seg->numEdges; ++i) {
      seg->edges[i] = pathE[i];
      rst->edgeUtils[pathE[i]]++; // add our wire to the road traffic
      updateSingleEdgeWeight(rst, pathE[i]); // recalculate the road cost
    }
    return 1;
  }
  return 0; // when completely failed to find path
}

// Looks at the single wire to see if it is caught in a traffic jam
static int getNetCostAndCongestion(const net *n, routingInst *rst, bool &hasCong) {
  int total = 0;
  hasCong = false;
  for (int s = 0; s < n->nroute.numSegs; ++s) {
    for (int k = 0; k < n->nroute.segments[s].numEdges; ++k) {
      int eid = n->nroute.segments[s].edges[k];
      total += rst->edgeWeights[eid]; // add up the travel cost
      if (!hasCong && rst->edgeUtils[eid] > rst->edgeCaps[eid]) hasCong = true;
    }
  }
  return total;
}

// find problem wires and sort them so we fix the worst ones first
static std::vector<int> buildCongestedNetOrdering(routingInst *rst) {
  struct NetInfo { int id; double density; };
  std::vector<NetInfo> info;
  info.reserve(rst->numNets / 4);

  // scan all of the wires on the board
  for (int i = 0; i < rst->numNets; ++i) {
    bool cong;
    int cost = getNetCostAndCongestion(&rst->nets[i], rst, cong);
    if (cong) {
      // find the total length of this wire
      int num_edges = 0;
      for (int s = 0; s < rst->nets[i].nroute.numSegs; ++s) {
        num_edges += rst->nets[i].nroute.segments[s].numEdges;
      }
      
      // find density, (cost^2)/length. 
      // put the worst, most localized traffic jams to the top of the list
      double density = (num_edges > 0) ? ((double)cost * cost) / num_edges : cost;
      info.push_back({i, density});
    }
  }

  // sort list from highest density to lowest
  std::sort(info.begin(), info.end(),
    [](const NetInfo& a, const NetInfo& b) {
      return a.density > b.density;
    });

  // get the IDs to return
  std::vector<int> order;
  order.reserve(info.size());
  for (auto& ni : info) order.push_back(ni.id);
  return order;
}

// revmove the specific wire segment from the map
static void ripUpSegment(segment *seg, routingInst *rst) {
  for (int k = 0; k < seg->numEdges; ++k) {
    int eid = seg->edges[k];
    if (rst->edgeUtils[eid] > 0) rst->edgeUtils[eid]--; // lower traffic
    updateSingleEdgeWeight(rst, eid); // make road cheaper again!!!!
  }
  free(seg->edges);
  seg->edges    = NULL;
  seg->numEdges = 0;
}

// deletes out the entire wire from memory
static void freeNetRoute(net *n) {
  if (n->nroute.segments) {
    for (int s = 0; s < n->nroute.numSegs; ++s)
      if (n->nroute.segments[s].edges) free(n->nroute.segments[s].edges);
    free(n->nroute.segments);
    n->nroute.segments = NULL;
  }
  n->nroute.numSegs = 0;
}

// look at a single net, but only re-route the specific parts that are jammed
static int ripUpAndRerouteNetSegments(net *n, routingInst *rst, int iter) {
  for (int s = 0; s < n->nroute.numSegs; ++s) {
    segment *seg = &n->nroute.segments[s];
    
    // find the worst jam on this specific piece of wire
    int max_ov = 0;
    for (int k = 0; k < seg->numEdges; ++k) {
      int ov = getEdgeOverflow(rst, seg->edges[k]);
      if (ov > max_ov) max_ov = ov;
    }
    
    // Skip this one if it's clean
    if (max_ov == 0) continue;

    // If we spend too much time on the run (past 15 loops) 
    // and the jam is negligible (1 extra wire), ignore it to prevent infinite loops.
    if (iter > 15 && max_ov == 1) continue;

    point p1 = seg->p1, p2 = seg->p2;
    
    // redo and find a new A* path
    ripUpSegment(seg, rst);
    rerouteSegmentAStar(seg, p1, p2, rst, false); // false = allows dynamic search box
  }
  return 1;
}

// memory structures to hold the state of the map
struct SegSnap { point p1, p2; std::vector<int> edges; };
struct NetSnap { int numSegs; std::vector<SegSnap> segs; };
struct RoutingSnapshot {
  int              bestTOF;
  std::vector<int> edgeUtils;
  std::vector<NetSnap> nets;
};

// copy the entire current map state into a safe backup variable
static void takeSnapshot(routingInst *rst, RoutingSnapshot &snap) {
  snap.bestTOF = getTotalOverflow(rst);
  snap.edgeUtils.assign(rst->edgeUtils, rst->edgeUtils + rst->numEdges);
  snap.nets.resize(rst->numNets);
  for (int i = 0; i < rst->numNets; ++i) {
    const net &n = rst->nets[i];
    snap.nets[i].numSegs = n.nroute.numSegs;
    snap.nets[i].segs.resize(n.nroute.numSegs);
    for (int s = 0; s < n.nroute.numSegs; ++s) {
      const segment &seg        = n.nroute.segments[s];
      snap.nets[i].segs[s].p1   = seg.p1;
      snap.nets[i].segs[s].p2   = seg.p2;
      snap.nets[i].segs[s].edges.assign(seg.edges, seg.edges + seg.numEdges);
    }
  }
}

// delete the current map state and replace it with the backup
static void restoreSnapshot(routingInst *rst, const RoutingSnapshot &snap) {
  std::copy(snap.edgeUtils.begin(), snap.edgeUtils.end(), rst->edgeUtils);
  for (int i = 0; i < rst->numNets; ++i) {
    freeNetRoute(&rst->nets[i]);
    int ns = snap.nets[i].numSegs;
    rst->nets[i].nroute.numSegs = ns;
    if (ns == 0) continue;
    rst->nets[i].nroute.segments = (segment*)malloc(sizeof(segment) * ns);
    for (int s = 0; s < ns; ++s) {
      const SegSnap &ss = snap.nets[i].segs[s];
      rst->nets[i].nroute.segments[s].p1 = ss.p1;
      rst->nets[i].nroute.segments[s].p2 = ss.p2;
      int ne = (int)ss.edges.size();
      rst->nets[i].nroute.segments[s].numEdges = ne;
      if (ne > 0) {
        rst->nets[i].nroute.segments[s].edges = (int*)malloc(sizeof(int) * ne);
        std::copy(ss.edges.begin(), ss.edges.end(),
                  rst->nets[i].nroute.segments[s].edges);
      } else {
        rst->nets[i].nroute.segments[s].edges = NULL;
      }
    }
  }
  recomputeAllEdgeWeights(rst);
}

static int runSingleRRRIteration(routingInst *rst, int iter) {
  // get a sorted list of problem wires
  std::vector<int> order = buildCongestedNetOrdering(rst);
  
  // try to fix them one by one
  for (int netId : order)
    ripUpAndRerouteNetSegments(&rst->nets[netId], rst, iter);
    
  // update the past history penalties for the next round
  updateEdgeHistoryFromCurrentOverflow(rst, iter);
  
  // update the costs on the board
  recomputeAllEdgeWeights(rst);
  
  return 1;
}

// oranize the pins from bottom-left to top-right to make connecting them easier
static int applyPinOrderingToNet(net *n) {
  if (!n || !n->pins || n->numPins <= 2) return 1;
  point *ord = (point*)malloc(sizeof(point) * n->numPins);
  int   *vis = (int*)  malloc(sizeof(int)   * n->numPins);
  for (int i = 0; i < n->numPins; ++i) vis[i] = 0;

  int si = 0;
  for (int i = 1; i < n->numPins; ++i) {
    if (n->pins[i].x < n->pins[si].x ||
        (n->pins[i].x == n->pins[si].x && n->pins[i].y < n->pins[si].y)) si = i;
  }
  ord[0] = n->pins[si]; vis[si] = 1;
  point cur = ord[0];

  for (int k = 1; k < n->numPins; ++k) {
    int best = -1;
    for (int i = 0; i < n->numPins; ++i) {
      if (vis[i]) continue;
      if (best == -1) { best = i; continue; }
      int dc = std::abs(cur.x - n->pins[i].x)    + std::abs(cur.y - n->pins[i].y);
      int db = std::abs(cur.x - n->pins[best].x) + std::abs(cur.y - n->pins[best].y);
      if (dc < db) best = i;
    }
    ord[k] = n->pins[best]; vis[best] = 1; cur = ord[k];
  }
  for (int i = 0; i < n->numPins; ++i) n->pins[i] = ord[i];
  free(ord); free(vis);
  return 1;
}

// wrapper to sort the pins on every net
static int applyPinOrdering(routingInst *rst) {
  for (int i = 0; i < rst->numNets; ++i)
    if (!applyPinOrderingToNet(&rst->nets[i])) return 0;
  return 1;
}

// open the benchmark file and read all settings, capacities, and coordinates
int readBenchmark(const char *fileName, routingInst *rst) {
  FILE *fp = fopen(fileName, "r");
  if (!fp) return 0;
  char s1[128], s2[128];
  
  // read the board size (X and Y limits)
  if (fscanf(fp, "%127s %d %d", s1, &rst->gx, &rst->gy) != 3) { fclose(fp); return 0; }
  // read the default road capacity
  if (fscanf(fp, "%127s %d",    s1, &rst->cap)           != 2) { fclose(fp); return 0; }
  // read the total number of wires to place
  if (fscanf(fp, "%127s %127s %d", s1, s2, &rst->numNets)!= 3) { fclose(fp); return 0; }

  // read all pin coordinates for every wire
  rst->nets = (net*)malloc(sizeof(net) * rst->numNets);
  for (int i = 0; i < rst->numNets; ++i) {
    char nm[64]; int np;
    if (fscanf(fp, "%63s %d", nm, &np) != 2) { fclose(fp); return 0; }
    rst->nets[i].id = i; rst->nets[i].numPins = np;
    rst->nets[i].pins = (point*)malloc(sizeof(point) * np);
    for (int j = 0; j < np; ++j)
      fscanf(fp, "%d %d", &rst->nets[i].pins[j].x, &rst->nets[i].pins[j].y);
  }
  
  initEdgeCapsUtils(rst); // setup memory arrays
  
  // read any custom blockages (areas where road capacity is found to be lower than default assumed)
  int nBlk = 0;
  if (fscanf(fp, "%d", &nBlk) == 1)
    for (int k = 0; k < nBlk; ++k) {
      int x1, y1, x2, y2, cap;
      if (fscanf(fp, "%d %d %d %d %d", &x1, &y1, &x2, &y2, &cap) == 5) {
        int eid = edgeIdFromPoints({x1,y1},{x2,y2}, rst->gx, rst->gy);
        if (eid >= 0 && eid < rst->numEdges) rst->edgeCaps[eid] = cap;
      }
    }
  fclose(fp);
  return 1;
}

// Main.
int solveRouting(routingInst *rst) {
  if (gEnablePinOrdering) applyPinOrdering(rst);

  // clear all traffic before starting
  for (int e = 0; e < rst->numEdges; ++e) rst->edgeUtils[e] = 0;

  // phase 1, initial draft of routing.
  std::vector<int> initOrder(rst->numNets);
  std::iota(initOrder.begin(), initOrder.end(), 0);

  // Find Bounding Box.
  // To prevent A* from going haywire, we need to set boxes for how long they can go.
  // those boxes will be set using manhattan distances between pins so boxes can be different sizes.
  // Sort nets from smallest box to largest box, do the smaller ones first.
  std::sort(initOrder.begin(), initOrder.end(), [rst](int a, int b) {
    auto hpwl = [rst](int ni) {
      int mnX = INT_MAX, mxX = INT_MIN, mnY = INT_MAX, mxY = INT_MIN;
      for (int p = 0; p < rst->nets[ni].numPins; ++p) {
        int x = rst->nets[ni].pins[p].x, y = rst->nets[ni].pins[p].y;
        if (x < mnX) mnX = x;
        if (x > mxX) mxX = x;
        if (y < mnY) mnY = y;
        if (y > mxY) mxY = y;
      }
      return (mxX - mnX) + (mxY - mnY);
    };
    return hpwl(a) < hpwl(b);
  });

  // Loop through wires and draw the first draft paths using strict Z-shapes
  // Horizontal - Vertial - Horizontal. Make a rough draft before A*
  for (int ni : initOrder) {
    net *n = &rst->nets[ni];
    int k    = n->numPins;
    int segs = (k >= 2) ? (k - 1) : 0;
    n->nroute.numSegs = segs;
    if (segs == 0) continue;
    n->nroute.segments = (segment*)malloc(sizeof(segment) * segs);

    if (k == 2) {
      rerouteSegmentAStar(&n->nroute.segments[0], n->pins[0], n->pins[1], rst, true); // true = Phase 1 Mode
    } else {
      auto mst = computePinMST(n);
      for (int s = 0; s < segs; ++s)
        rerouteSegmentAStar(&n->nroute.segments[s],
                     n->pins[mst[s].first], n->pins[mst[s].second], rst, true);
    }
  }

  // Update map traffic
  recomputeAllEdgeWeights(rst);

  if (!gEnableRRR) return 1; // Stop if user only wanted Phase 1

  // phase 2: rip-up and reroute.
  using clk = std::chrono::steady_clock;
  const double MIN_SEC = 5.0  * 60.0;   // 5 Minutes Min
  const double MAX_SEC = 14.0 * 60.0;   // 14 Minutes Max (considering iteration)
  auto t0 = clk::now();

  int bestTOF      = getTotalOverflow(rst);
  int prevTOF      = bestTOF;
  int noImprove    = 0;
  int lowImproveCount = 0; 

  RoutingSnapshot bestSnap;
  takeSnapshot(rst, bestSnap);

  // Loop, broken when timer runs out or conditions are met
  for (int iter = 0; ; ++iter) {

    // Gradually make A* care more about distance as time goes on
    if      (iter < 3)  { gHNum = 3;  gHDen = 2;  }  
    else if (iter < 7)  { gHNum = 5;  gHDen = 4;  }  
    else if (iter < 12) { gHNum = 11; gHDen = 10; }   
    else                { gHNum = 1;  gHDen = 1;  }  

    // Run 1 cycle to fix jams. Stop if it throws a memory error.
    if (!runSingleRRRIteration(rst, iter)) return 0;

    int currTOF = getTotalOverflow(rst);
    double elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(
                       clk::now() - t0).count();

    // Check if we hit a new high score
    if (currTOF < bestTOF) {
      bestTOF = currTOF;
      takeSnapshot(rst, bestSnap); // Good, backup this result by doing snapshot.
      noImprove = 0;
    } else {
      ++noImprove; // this run didn't improve
    }

    // Stop if the board is 100% clean.
    if (currTOF == 0) break;           

    // Check if 5 minutes passed.
    if (elapsed >= MIN_SEC) {
      if (noImprove >= 3) break; // Quit if stuck on bad scores
      
      // If the score is less than 1% better, stop wasting time and quit.
      if (prevTOF > 0) {
        double ratio = (double)(prevTOF - currTOF) / (double)prevTOF;
        if (ratio < 0.01) {
            lowImproveCount++;
            if (lowImproveCount >= 2) break;
        } else {
            lowImproveCount = 0; 
        }
      }
    }

    // Hard quit before 15 minutes to save Q-score penalty
    if (elapsed >= MAX_SEC) break;
    
    prevTOF = currTOF;
  }

  // Final Safety Check: If our last round ruined the score, load the backup!
  if (getTotalOverflow(rst) > bestSnap.bestTOF)
    restoreSnapshot(rst, bestSnap);

  return 1;
}

// Formats paths into "(X,Y)-(X,Y)" strings for grading.
static int writeSegmentFromEdges(FILE *fp, const segment *seg, routingInst *rst) {
  if (seg->numEdges == 0) return 1;
  std::vector<point> pts;
  pts.push_back(seg->p1);
  point cur = seg->p1;
  
  // Follow array of roads to build point list
  for (int i = 0; i < seg->numEdges; ++i) {
    point a, b;
    edgePointsFromId(seg->edges[i], rst->gx, rst->gy, &a, &b);
    point nxt = (a.x == cur.x && a.y == cur.y) ? b : a;
    pts.push_back(nxt); cur = nxt;
  }
  
  // Compress lines to only show cornerss
  point rs = pts[0], prev = pts[0];
  for (size_t i = 1; i + 1 < pts.size(); ++i) {
    point c = pts[i], nx = pts[i + 1];
    if ((c.x - prev.x) != (nx.x - c.x) || (c.y - prev.y) != (nx.y - c.y)) {
      fprintf(fp, "(%d,%d)-(%d,%d)\n", rs.x, rs.y, c.x, c.y);
      rs = c;
    }
    prev = c;
  }
  fprintf(fp, "(%d,%d)-(%d,%d)\n", rs.x, rs.y, pts.back().x, pts.back().y);
  return 1;
}

// Loops over all nets and writes their coordinates into the .out file
int writeOutput(const char *outRouteFile, routingInst *rst) {
  FILE *fp = fopen(outRouteFile, "w");
  if (!fp) return 0;
  for (int i = 0; i < rst->numNets; ++i) {
    fprintf(fp, "n%d\n", rst->nets[i].id);
    for (int s = 0; s < rst->nets[i].nroute.numSegs; ++s)
      writeSegmentFromEdges(fp, &rst->nets[i].nroute.segments[s], rst);
    fprintf(fp, "!\n");
  }
  fclose(fp);
  return 1;
}

// Clean up arrays to prevent crashes
int release(routingInst *rst) {
  if (rst->nets) {
    for (int i = 0; i < rst->numNets; ++i) {
      freeNetRoute(&rst->nets[i]);
      if (rst->nets[i].pins) free(rst->nets[i].pins);
    }
    free(rst->nets);
  }
  if (rst->edgeCaps)    free(rst->edgeCaps);
  if (rst->edgeUtils)   free(rst->edgeUtils);
  if (rst->edgeHistory) free(rst->edgeHistory);
  if (rst->edgeWeights) free(rst->edgeWeights);
  return 1;
}