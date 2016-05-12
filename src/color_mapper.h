#ifndef __COLOR_MAPPER_H__
#define __COLOR_MAPPER_H__

#include <map>

class SbPDG_Node;


//basically, a global state for all objects
//no one said this project wasn't a hack!
class ColorMapper {
public:
    int colorOf(SbPDG_Node* item, bool reset =false);
    void setColor(int i, SbPDG_Node* item) {colorMap[item]=i;}
    void clear() {colorMap.clear();}
private:
    std::map<SbPDG_Node*,int> colorMap;
};
#endif
