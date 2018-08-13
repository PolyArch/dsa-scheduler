#ifndef __COLOR_MAPPER_H__
#define __COLOR_MAPPER_H__

#include <map>

class SbPDG_Node;

//Maintain mapping between SbPDG_Node's and colors
class ColorMapper {
public:
    int colorOf(SbPDG_Node* item, bool reset =false);
    void setColor(int i, SbPDG_Node* item) {colorMap[item]=i;}
    void clear() {colorMap.clear();}
private:
    std::map<SbPDG_Node*,int> colorMap;
};
#endif
