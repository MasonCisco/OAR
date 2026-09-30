#include <SDL2/SDL.h>
#include <cstdio>

int main() {
    if(SDL_Init(SDL_INIT_GAMECONTROLLER) < 0) {
        std::printf("SDL init failed: %s\n", SDL_GetError());
        return 1;
    }

    SDL_GameController* pad = nullptr;
    for(int i = 0; i < SDL_NumJoysticks(); ++i){
        if(SDL_IsGameController(i)){
            pad = SDL_GameControllerOpen(i);
            break;
        }
    }
    if(!pad) {
        std::printf("No controller found\n");
        return 1;
    }
    
    while (true){
        int lx = SDL_GameControllerGetAxis(pad, SDL_CONTROLLER_AXIS_LEFTX);
        int ly = SDL_GameControllerGetAxis(pad, SDL_CONTROLLER_AXIS_LEFTY); 
        int rx = SDL_GameControllerGetAxis(pad, SDL_CONTROLLER_AXIS_RIGHTX);
        int ry = SDL_GameControllerGetAxis(pad, SDL_CONTROLLER_AXIS_RIGHTY);
        int lt = SDL_GameControllerGetAxis(pad, SDL_CONTROLLER_AXIS_TRIGGERLEFT);
        int rt = SDL_GameControllerGetAxis(pad, SDL_CONTROLLER_AXIS_TRIGGERRIGHT);
        int a = SDL_GameControllerGetButton(pad, SDL_CONTROLLER_BUTTON_A);
        int b = SDL_GameControllerGetButton(pad, SDL_CONTROLLER_BUTTON_B);

        std::printf("LX:%6d LY:%6d RX:%6d RY:%6d LT:%6d RT:%6d A:%d B:%d\n", lx, ly, rx, ry, lt, rt, a, b);
        std::fflush(stdout);
        SDL_Delay(50);
    }

}
