#ifndef DATA_H_
#define DATA_H_

#include <mutex>
#include <string>
#include <vector>

typedef struct Data {
    std::mutex m;

    double timestamp;
    double battery;
    int wireless_signal;

    bool isBlink;
    bool isLeftWink;
    bool isRightWink;
    bool isLookingLeft;
    bool isLookingRight;

    std::string lowerFaceActionName;
    int lowerFaceAction;
    double lowerFaceActionPower;

    std::string upperFaceActionName;
    int upperFaceAction;
    double upperFaceActionPower;

    std::vector<int> cq;

} data_t;

#endif
