#ifndef COLOR_MAPPER_H
#define COLOR_MAPPER_H

#include <map>

class DyPDG_Node;


//basically, a global state for all objects
//no one said this project wasn't a hack!
class ColorMapper {
public:
    int colorOf(DyPDG_Node* item, bool reset =false);
    void setColor(int i, DyPDG_Node* item) {colorMap[item]=i;}
    void clear() {colorMap.clear();}
private:
    std::map<DyPDG_Node*,int> colorMap;
};
#endif