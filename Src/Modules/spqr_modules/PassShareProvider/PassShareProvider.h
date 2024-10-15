// TODO PORT THIS

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/spqr_representations/PassShare.h"

MODULE(PassShareProvider,
{,

 PROVIDES(PassShare),

});

class PassShareProvider : public PassShareProviderBase
{
private:


public:
    void update(PassShare& ps);
};
