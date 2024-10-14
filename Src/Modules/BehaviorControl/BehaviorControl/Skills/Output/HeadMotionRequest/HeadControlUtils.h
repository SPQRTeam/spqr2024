#pragma once

#include <map>
#include <array>
#include "Tools/Math/Angle.h"

#include <iostream>

class ScanStateHandler {
public:
    enum class ScanState {
        TopLeft = 0,
        CenterBottomToRight = 1,
        CenterBottomToLeft = 2,
        TopRight = 3,
    };
    std::map<ScanState, std::array<Angle, 2>> stateToPanTilt = {
        {ScanState::CenterBottomToRight, {0_deg, 23_deg}},
        {ScanState::CenterBottomToLeft, {0_deg, 23_deg}},
        {ScanState::TopLeft, {50_deg, 0_deg}},
        {ScanState::TopRight, {-50_deg, 0_deg}},
    };

    ScanStateHandler(Angle stateCompleteThreshold = 5_deg) : currentState(ScanState::CenterBottomToRight), stateCompleteThreshold(stateCompleteThreshold) {}

    void reset() {
        currentState = ScanState::CenterBottomToRight;
    }

    ScanState getCurrentState() const {
        return currentState;
    }

    std::array<Angle, 2> getPanTilt(){
        return stateToPanTilt[currentState];
    }

    void transition(Angle current_pan, Angle current_tilt) {
        std::array<Angle, 2> currentPanTilt = getPanTilt();
        if (std::abs(Angle::normalize(current_pan - currentPanTilt[0])) < stateCompleteThreshold && 
            std::abs(Angle::normalize(current_tilt - currentPanTilt[1])) < stateCompleteThreshold)
            currentState = scanTransitions.at(currentState);

    }

private:

    std::map<ScanState, ScanState> scanTransitions = {
        {ScanState::CenterBottomToRight, ScanState::TopRight},
        {ScanState::TopRight, ScanState::CenterBottomToLeft},
        {ScanState::CenterBottomToLeft, ScanState::TopLeft},
        {ScanState::TopLeft, ScanState::CenterBottomToRight}
    };

    ScanState currentState;
    Angle stateCompleteThreshold;
};