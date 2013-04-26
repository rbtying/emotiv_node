#include "Emotiv.h"
#include <map>

std::string eyebrow("Eyebrow"), furrow("Furrow"), smile("Smile"), clench("Clench");
std::string smirk_left("Smirk Left"), smirk_right("Smirk Right"), laugh("Laugh"), unknown("Unknown");
std::string blink("Blink"), neutral("Neutral"), wink_left("Wink Left"), wink_right("Wink Right"), horieye("Horieye");

std::string actionToName(EE_ExpressivAlgo_t action) {
    switch(action) {
        case EXP_BLINK:
            return blink;
        case EXP_NEUTRAL:
            return neutral;
        case EXP_WINK_LEFT:
            return wink_left;
        case EXP_WINK_RIGHT:
            return wink_right;
        case EXP_HORIEYE:
            return horieye;
        case EXP_EYEBROW:
            return eyebrow;
        case EXP_FURROW:
            return furrow;
        case EXP_SMILE:
            return smile;
        case EXP_CLENCH:
            return clench;
        case EXP_SMIRK_LEFT:
            return smirk_left;
        case EXP_SMIRK_RIGHT:
            return smirk_right;
        case EXP_LAUGH:
            return laugh;
    }
    return unknown;
}

void updateData(data_t *data, EmoStateHandle eState) {
    std::lock_guard<std::mutex> lk(data->m);
    double timestamp;
    data->timestamp = ES_GetTimeFromStart(eState);
    data->wireless_signal = static_cast<int>(ES_GetWirelessSignalStatus(eState));

    data->isBlink = ES_ExpressivIsBlink(eState);
    data->isLeftWink =  ES_ExpressivIsLeftWink(eState);
    data->isRightWink = ES_ExpressivIsRightWink(eState);
    data->isLookingLeft = ES_ExpressivIsLookingLeft(eState);
    data->isLookingRight = ES_ExpressivIsLookingRight(eState);

    EE_ExpressivAlgo_t upperFaceAction = ES_ExpressivGetUpperFaceAction(eState);
    EE_ExpressivAlgo_t lowerFaceAction = ES_ExpressivGetLowerFaceAction(eState);

    data->lowerFaceActionName = actionToName(lowerFaceAction);
    data->lowerFaceAction = static_cast<int>(lowerFaceAction);
    data->lowerFaceActionPower = ES_ExpressivGetLowerFaceActionPower(eState);

    data->upperFaceActionName = actionToName(upperFaceAction);
    data->upperFaceAction = static_cast<int>(upperFaceAction);
    data->upperFaceActionPower = ES_ExpressivGetUpperFaceActionPower(eState);

	int numc = ES_GetNumContactQualityChannels(eState);
    EE_EEG_ContactQuality_t cq[numc];
    ES_GetContactQualityFromAllChannels(eState, cq, numc);

    data->cq.clear();
    for (int i = 0; i < numc; i++) {
        data->cq.push_back(cq[i]);
    }
    int batt, maxbatt;
    ES_GetBatteryChargeLevel(eState, &batt, &maxbatt);
    data->battery = batt * 1.0 / maxbatt;
}

void emote(volatile bool *running, data_t *data) {
    EmoEngineEventHandle eEvent			= EE_EmoEngineEventCreate();
    EmoStateHandle eState				= EE_EmoStateCreate();
    unsigned int userID					= 0;
    const unsigned short composerPort	= 1726;
    int option = 0;
    int state  = 0;
    std::string input;

    bool connected = false;
    if(EE_EngineConnect() == EDK_OK)
    {
        connected = true;
        std::cout << "Start receiving EmoState !\n" << std::endl;
    }
    else
    {
        connected = false;
        std::cout <<"EmoEngine start up failed\n" << std::endl;
    }

    bool writeHeader = true;

    while (connected && *running)
    {
        state = EE_EngineGetNextEvent(eEvent);
       // std::cout << "state : " << state << std::endl;
        // New event needs to be handled
        if (state == EDK_OK)
        {
             EE_Event_t eventType = EE_EmoEngineEventGetType(eEvent);
             EE_EmoEngineEventGetUserId(eEvent, &userID);

             // Log the EmoState if it has been updated
             if (eventType == EE_EmoStateUpdated)
             {                 
                  EE_EmoEngineEventGetEmoState(eEvent, eState);
                  const float timestamp = ES_GetTimeFromStart(eState);

                  std::cout << timestamp <<" New EmoState from user " << userID << std::endl;

                  /* logEmoState(std::cout, userID, eState, writeHeader); */

                  updateData(data, eState);
                  writeHeader = false;
              }
         }
         else if (state != EDK_NO_EVENT)
         {
             std::cout << "Internal error in Emotiv Engine!" << std::endl;
             break;
         }
    }

    EE_EngineDisconnect();
    EE_EmoStateFree(eState);
    EE_EmoEngineEventFree(eEvent);
}

#if  0
void logEmoState(std::ostream& os, unsigned int userID, EmoStateHandle eState, bool withHeader) {

    // Create the top header
    if (withHeader) {
        os << "Time,";
        os << "UserID,";
        os << "Wireless Signal Status,";
        os << "Blink,";
        os << "Wink Left,";
        os << "Wink Right,";
        os << "Look Left,";
        os << "Look Right,";
        os << "Eyebrow,";
        os << "Furrow,";
        os << "Smile,";
        os << "Clench,";
        os << "Smirk Left,";
        os << "Smirk Right,";
        os << "Laugh,";
        os << "Short Term Excitement,";
        os << "Long Term Excitement,";
        os << "Engagement/Boredom,";
        os << "Cognitiv Action,";
        os << "Cognitiv Power,";
        os << std::endl;
    }

    // Log the time stamp and user ID
    os << ES_GetTimeFromStart(eState) << ",";
    os << userID << ",";
    os << static_cast<int>(ES_GetWirelessSignalStatus(eState)) << ",";

    // Expressiv Suite results
    os << ES_ExpressivIsBlink(eState) << ",";
    os << ES_ExpressivIsLeftWink(eState) << ",";
    os << ES_ExpressivIsRightWink(eState) << ",";

    os << ES_ExpressivIsLookingLeft(eState) << ",";
    os << ES_ExpressivIsLookingRight(eState) << ",";

    std::map<EE_ExpressivAlgo_t, float> expressivStates;

    EE_ExpressivAlgo_t upperFaceAction = ES_ExpressivGetUpperFaceAction(eState);
    float			   upperFacePower  = ES_ExpressivGetUpperFaceActionPower(eState);

    EE_ExpressivAlgo_t lowerFaceAction = ES_ExpressivGetLowerFaceAction(eState);
    float			   lowerFacePower  = ES_ExpressivGetLowerFaceActionPower(eState);

    expressivStates[ upperFaceAction ] = upperFacePower;
    expressivStates[ lowerFaceAction ] = lowerFacePower;

    os << expressivStates[ EXP_EYEBROW     ] << ","; // eyebrow
    os << expressivStates[ EXP_FURROW      ] << ","; // furrow
    os << expressivStates[ EXP_SMILE       ] << ","; // smile
    os << expressivStates[ EXP_CLENCH      ] << ","; // clench
    os << expressivStates[ EXP_SMIRK_LEFT  ] << ","; // smirk left
    os << expressivStates[ EXP_SMIRK_RIGHT ] << ","; // smirk right
    os << expressivStates[ EXP_LAUGH       ] << ","; // laugh

    // Affectiv Suite results
    os << ES_AffectivGetExcitementShortTermScore(eState) << ",";
    os << ES_AffectivGetExcitementLongTermScore(eState) << ",";

    os << ES_AffectivGetEngagementBoredomScore(eState) << ",";

    // Cognitiv Suite results
    os << static_cast<int>(ES_CognitivGetCurrentAction(eState)) << ",";
    os << ES_CognitivGetCurrentActionPower(eState);

    os << std::endl;
}

#endif
