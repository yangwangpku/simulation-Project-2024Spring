#include "Assets/bundled.h"
#include "Labs/4-PD/App.h"

namespace VCX::Labs::PD {

    App::App():
        _ui(Labs::Common::UIOptions {}){
    }

    void App::OnFrame() {
        _ui.Setup(_cases, _caseId);
    }
}
