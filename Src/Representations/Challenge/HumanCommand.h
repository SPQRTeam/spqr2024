/**
 * @file HumanCommand.h
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Streams/Enum.h"

/**
 * @class HumanCommand
 *
 */

STREAMABLE(HumanCommand, COMMA public BHumanCompressedMessageParticle<HumanCommand>
{
    ENUM(CommandBody,
    {,
        BaseCommandBody,
        GoToPosition,
        Dribble,
        GoToBallAndDribble,
        Kick,
        Spazza,
        Pass,
        AskForTheBall,
        Turn,
        SearchTheBall,
        Stop,
    });

    ENUM(CommandHead,
    {,
        BaseCommandHead,
        LookAtTheBall,
        LookForward,
        LookLeft,
        LookRight,
        Scan,
    });
    
    ENUM(Strategy,
    {,
        Default,
        Passages,
    });

    // create a function to map the enum to a string
    static const char* CommandBody2String(CommandBody commandBody)
    {
        switch (commandBody)
        {
            case CommandBody::BaseCommandBody: return "BaseCommandBody";
            case CommandBody::GoToPosition: return "GoToPosition";
            case CommandBody::Dribble: return "Dribble";
            case CommandBody::GoToBallAndDribble: return "GoToBallAndDribble";
            case CommandBody::Kick: return "Kick";
            case CommandBody::Spazza: return "Spazza";
            case CommandBody::Pass: return "Pass";
            case CommandBody::AskForTheBall: return "AskForTheBall";
            case CommandBody::Turn: return "Turn";
            case CommandBody::SearchTheBall: return "SearchTheBall";
            case CommandBody::Stop: return "Stop";
        }
        return "Unknown";
    };

    static const char* CommandHead2String(CommandHead commandHead)
    {
        switch (commandHead)
        {
            case CommandHead::BaseCommandHead: return "BaseCommandHead";
            case CommandHead::LookAtTheBall: return "LookAtTheBall";
            case CommandHead::LookForward: return "LookForward";
            case CommandHead::LookLeft: return "LookLeft";
            case CommandHead::LookRight: return "LookRight";
            case CommandHead::Scan: return "Scan";
        }
        return "Unknown";
    };

    static const char* Strategy2String(Strategy strategy)
    {
        switch (strategy)
        {
            case Strategy::Default: return "Default";
            case Strategy::Passages: return "Passages";
        }
        return "Unknown";
    },

    (CommandBody)(CommandBody::BaseCommandBody) commandBody,
    (CommandHead)(CommandHead::BaseCommandHead) commandHead,
    (Strategy) strategy,
    (float) x,
    (float) y,

    (bool) defending,
    
});

