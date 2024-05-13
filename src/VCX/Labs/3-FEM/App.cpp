#include "Assets/bundled.h"
#include "Labs/3-FEM/App.h"

namespace VCX::Labs::FEM {

    App::App():
        _ui(Labs::Common::UIOptions {}),
        _casedeform() {
    }

    void App::OnFrame() {
        _ui.Setup(_cases, _caseId);
    }
}
