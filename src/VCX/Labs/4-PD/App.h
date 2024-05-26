#pragma once

#include <vector>

#include "Engine/app.h"
#include "Labs/4-PD/CaseMassSpring.h"
#include "Labs/Common/UI.h"

namespace VCX::Labs::PD {
    class App : public Engine::IApp {
    private:
        Common::UI _ui;

        CaseMassSpring _caseMassSpring;

        std::size_t _caseId = 0;

        std::vector<std::reference_wrapper<Common::ICase>> _cases = { _caseMassSpring };

    public:
        App();

        void OnFrame() override;
    };
}
