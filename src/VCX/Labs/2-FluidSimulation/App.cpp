#include "Assets/bundled.h"
#include "Labs/2-FluidSimulation/App.h"

namespace VCX::Labs::FluidSimulation {

    App::App():
        _ui(Labs::Common::UIOptions {}),
        _casefluid({ Assets::ExampleScene::Fluid }) {
    }

    void App::OnFrame() {
        _ui.Setup(_cases, _caseId);
    }
}
