#include "Assets/bundled.h"
#include "Labs/2-FluidSimulation/App.h"

int main() {
    using namespace VCX;
    return Engine::RunApp<Labs::FluidSimulation::App>(Engine::AppContextOptions {
        .Title         = "VCX-sim Labs 2: Fluid Simulation",
        .WindowSize    = {512, 384},
        .FontSize      = 16,
        .IconFileNames = Assets::DefaultIcons,
        .FontFileNames = Assets::DefaultFonts,
    });
}
