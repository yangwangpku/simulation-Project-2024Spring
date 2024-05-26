#include "Assets/bundled.h"
#include "Labs/4-PD/App.h"

int main() {    
    using namespace VCX;
    return Engine::RunApp<Labs::PD::App>(Engine::AppContextOptions {
        .Title         = "VCX-sim Labs 4: Projective Dynamics",
        .WindowSize    = {1024, 768},
        .FontSize      = 16,
        .IconFileNames = Assets::DefaultIcons,
        .FontFileNames = Assets::DefaultFonts,
    });
}
