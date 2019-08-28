#ifndef __COLOR_MAPPER_H__
#define __COLOR_MAPPER_H__

#include <map>
#include <tuple>

class SSDfgNode;
class SSDfgValue;

//Maintain mapping between SSDfgNode's and colors
class ColorMapper {
public:
    int colorOf(SSDfgValue* node, bool reset =false);
    void setColor(std::tuple<int,int,int> i, SSDfgNode* node) {colorMap[node]=i;}
    void clear() {colorMap.clear();}
private:
    std::map<SSDfgNode*,std::tuple<int,int,int>> colorMap;
};
#endif
