#include "dse.h"

CodesignInstance::CodesignInstance(SSModel* model) : _ssModel(*model) {
  verify();
  unused_nodes = std::vector<bool>(model->subModel()->node_list().size(), true);
  unused_links = std::vector<bool>(model->subModel()->link_list().size(), true);
}
