#include "draw.h"

bool draw(Viewer &viewer){
    
    if (!GlobalState::must_be_updated()) return false;
    GlobalState::deactivate_update();
    GlobalState gs = GlobalState::instance();

    M_Assert((&viewer == &(gs.getViewer())), "The draw function must be called with the global viewer.");
    gs.draw();

    return false;
}