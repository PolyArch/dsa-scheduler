#include "dsa/dfg/ssdfg.h"

namespace dsa {
namespace dfg {
namespace pass {

inline void SliceOverlappedEdges(SSDfg *dfg) {

  struct EdgeSlicer {

    std::map<SSDfgEdge*, std::vector<SSDfgEdge*>> replace;

    void Visit(SSDfgNode *node) {
      for (auto value : node->values()) {
        std::vector<SSDfgEdge*> edges = value->edges();
        sort(edges.begin(), edges.end(), [] (SSDfgEdge *a, SSDfgEdge *b) {
          return a->l() != b->l() ? a->l() < b->l() : a->r() > b->r();
        });
        for (int i = 0, n = edges.size(); i < n; ++i) {
          int nxt = i + 1;
          if (nxt < n && edges[nxt]->l() <= edges[i]->r()) {
            ++nxt;
          }
          std::vector<int> points;
          for (int j = i; j < nxt; ++j) {
            points.push_back(edges[i]->l());
            points.push_back(edges[i]->r());
          }
          std::sort(points.begin(), points.end());
          int m = std::unique(points.begin(), points.end()) - points.begin();
          for (int j = 1; j < m; ++j) {
          }
        }

        for (int i = 0, n = value->edges().size(); i < n; ++i) {
          auto edgea = values->edges()[i];
          for (int j = i + 1; j < n; ++j) {
            auto edgeb = values->edges()[j];
          }
        }
      }
    }

  };


}


}
}
}