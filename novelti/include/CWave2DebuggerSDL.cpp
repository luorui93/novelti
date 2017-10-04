#ifndef CWAVE2_DEBUGGER_SDL_H_
#define CWAVE2_DEBUGGER_SDL_H_

#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include <stdio.h>
#include <string>
#include <cmath>
#include <algorithm> 

#include <CWave2.h>
#include <CWave2Debugger.h>

namespace cwave {
    
/*class CWave2DebuggerSDLEventHandler {
public: 
    CWave2DebuggerSDLEventHandler() {};
    void onPointClick(int cx, int cy);
}*/

class CWave2DebuggerSDL : public CWave2Debugger {

typedef int (*onPointClickType)(int, int);

public:

    struct Margins {
        int top;
        int right;
        int bottom;
        int left;
    };
    
    enum TextAlignment {
        ALIGN_LEFT_TOP,    ALIGN_CENTER_TOP,    ALIGN_RIGHT_TOP,
        ALIGN_LEFT_CENTER, ALIGN_CENTER_CENTER, ALIGN_RIGHT_CENTER,
        ALIGN_LEFT_BOTTOM, ALIGN_CENTER_BOTTOM, ALIGN_RIGHT_BOTTOM
    };
    
    enum Fonts {
        FONT_GRID  = 0,
        FONT_POINT = 1,
        FONT_QMARK = 2,
        FONTS_NUM  = 3
    };
    
    enum Layers {
        LAYER_MAP   = 0,
        LAYER_DIST  = 1,
        LAYER_BEAM  = 2,
        LAYER_TOP   = 3,
        LAYERS_NUM  = 4
    };
    
    SDL_Window*     win_;
    SDL_Renderer*   rend_;
    TTF_Font*       fonts_[FONTS_NUM];
    int             cell_size_;
    int             width_;
    int             height_;
    int             win_width_;
    int             win_height_;
    int             ybase_;
    bool            quit_;
    Margins         margins_;
    CompoundMap&    map_;
    SDL_Texture*    layers_[LAYERS_NUM];
    onPointClickType    onPointClick_;
    
    
    CWave2DebuggerSDL(CompoundMap& map, double map_size_coef) :
        map_(map)
    {
        quit_ = false;
        width_ = map.width();
        height_ = map.height();
        margins_ = {30, 30, 30, 30};
        onPointClick_ = NULL;
        
        if( SDL_Init( SDL_INIT_VIDEO ) < 0 )  {
            printf( "SDL could not initialize! SDL Error: %s\n", SDL_GetError() );
        } else {
            //Set texture filtering to linear
            if( !SDL_SetHint( SDL_HINT_RENDER_SCALE_QUALITY, "1" ) ) {
                printf( "Warning: Linear texture filtering not enabled!" );
            }
            
            ///for(i = 0; i < SDL_GetNumVideoDisplays(); ++i) {
            int i=0;
            SDL_DisplayMode disp;
            if (SDL_GetCurrentDisplayMode(i, &disp) != 0) {
                    printf("Could not get display mode for video display #%d: %s", i, SDL_GetError());
            } else {
                //Calculate cell size, window size etc
                double ws = double(disp.w -margins_.left -margins_.right);
                double hs = double(disp.h -margins_.top -margins_.bottom);
                if (ws/hs > double(map.width())/double(map.height())) //height is the limit
                    cell_size_ =(int)floor((map_size_coef*disp.h -margins_.top -margins_.bottom)/map.height());
                else 
                    cell_size_ =(int)floor((map_size_coef*disp.w -margins_.left -margins_.right)/map.width());
                win_width_  = margins_.left + map.width()*cell_size_  + margins_.right;
                win_height_ = margins_.top  + map.height()*cell_size_ + margins_.bottom;
                ybase_ = win_height_-margins_.top; //TODO double check
                
                win_ = SDL_CreateWindow( "CWave2 testing", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, win_width_, win_height_, SDL_WINDOW_SHOWN );
                if( win_ == NULL ) {
                    printf( "Window (size=(%d,%d)) could not be created! SDL Error: %s\n", win_width_, win_height_, SDL_GetError() );
                } else {
                    SDL_SetWindowPosition(win_, 0, 0);
                    rend_ = SDL_CreateRenderer( win_, -1, SDL_RENDERER_ACCELERATED );
                    if( rend_ == NULL ) {
                        printf( "Renderer could not be created! SDL Error: %s\n", SDL_GetError() );
                    } else {
                        SDL_SetRenderDrawColor( rend_, 0xFF, 0xFF, 0xFF, 0xFF ); //Initialize renderer color
                        //Initialize SDL_ttf
                        if( TTF_Init() == -1 ) {
                            printf( "SDL_ttf could not initialize! SDL_ttf Error: %s\n", TTF_GetError() );
                        }
                        for (int k=0; k<LAYERS_NUM; k++)
                            layers_[k] = createLayer(win_width_, win_height_);
                        
                        fonts_[FONT_GRID]  = TTF_OpenFont("fonts/LiberationMono-Regular.ttf", 14);
                        //fonts_[FONT_POINT] = TTF_OpenFont("/home/sd/ws/src/cwave/fonts/LiberationMono-Regular.ttf", 9);
                        fonts_[FONT_POINT] = TTF_OpenFont("fonts/OpenSans-CondLight.ttf", 9);
                        fonts_[FONT_QMARK] = TTF_OpenFont("fonts/LiberationMono-Regular.ttf", 25);
                                //if (font_ == NULL)
                                //    printf( "Failed to load font! SDL_ttf Error: %s\n", TTF_GetError() );
                        drawMap();
                        render();
                        
                    }
                }
            }
        }
    }
    
    ~CWave2DebuggerSDL() {
        for (int font_id=0; font_id<sizeof(fonts_[0]); font_id++) {
            TTF_CloseFont( fonts_[font_id] );
            fonts_[font_id] = NULL;
        }
        for (int k=0; k<LAYERS_NUM; k++)
             SDL_DestroyTexture(layers_[k]);
        SDL_DestroyRenderer( rend_ );
        SDL_DestroyWindow( win_ );
        win_ = NULL;
        rend_ = NULL;
        TTF_Quit();
        SDL_Quit();
    }
    
    SDL_Texture* createLayer(int width, int height) {
        SDL_Texture* layer = SDL_CreateTexture(rend_, SDL_PIXELFORMAT_RGBA8888, SDL_TEXTUREACCESS_TARGET, width, height);
        SDL_SetTextureBlendMode(layer, SDL_BLENDMODE_BLEND);
        SDL_SetRenderTarget(rend_, layer);
        SDL_SetRenderDrawColor( rend_, 0xFF, 0xFF, 0xFF, 0x00 );
        SDL_RenderClear( rend_ );
        return layer;
    }
    
    void render() {
        SDL_SetRenderTarget(rend_, NULL);
        SDL_SetRenderDrawBlendMode(rend_, SDL_BLENDMODE_BLEND);
        for (int k=0; k<LAYERS_NUM; k++) {
            SDL_RenderCopy(rend_, layers_[k],  NULL, NULL);
            SDL_RenderPresent( rend_ ); 
        }
    }
    
    int ptx(int cell_x) { return margins_.left + cell_x*cell_size_; }
    int pty(int cell_y) { return ybase_ - cell_y*cell_size_; }
    
    void processClick(SDL_Event& e) {
        int x,y;
        SDL_GetMouseState(&x, &y);
        printf("Mouse click at (%d,%d)\n", x,y);
        if (x>=margins_.left  && x< win_width_-margins_.right && 
            y>=margins_.top  && x<win_height_-margins_.bottom
        ) {
            int cx = (int)round((x+cell_size_/2-margins_.left)/cell_size_);
            int cy = (int)round((win_height_-y+cell_size_/2-margins_.top)/cell_size_);
            //printf("Mouse click on the map Point=(%d,%d)\n", cx,cy);
            if (onPointClick_ != NULL)
                onPointClick_(cx,cy);
        }
    }
    
    void mainLoop() {
        SDL_Event e;
        while( !quit_ ) {
            while( SDL_PollEvent( &e ) != 0 ) {
                //printf("    >>>>>>event type=%d \n", e.type);
                switch (e.type) {
                    case SDL_MOUSEBUTTONUP: 
                        if (e.button.clicks==1)
                            processClick(e); 
                        break;
                    case SDL_QUIT:          
                        quit_ = true;
                        //printf("--------------------------------------------------------------------\n");
                        break;
                }
            }
            //printf("quit_=%d \n", quit_?1:0);
            SDL_Delay(40);
        }
    }
    
    void drawMap() {
        SDL_SetRenderTarget(rend_, layers_[LAYER_MAP]);
        SDL_SetRenderDrawColor( rend_, 0xFF, 0xFF, 0xFF, 0xFF );
        SDL_RenderClear( rend_ );
        drawGrid();
        drawMapOnly(); 
    }
    
    void drawGrid() {
        for (int x=0; x<=width_; x++) {
            if (x%5==0) {
                SDL_SetRenderDrawColor( rend_, 0x00, 0x00, 0x00, 0xFF );
                writeText(ptx(x), ybase_-(-7), to_string(x), fonts_[FONT_GRID], ALIGN_CENTER_TOP);
            } else if (x%5==1) {
                SDL_SetRenderDrawColor( rend_, 0xAA, 0xAA, 0xAA, 0xFF );
            }
            #ifndef MAPPIC
            SDL_RenderDrawLine( rend_, 
                ptx(x), ybase_-(x%5 ? 0 : -7), 
                ptx(x), pty(height_) );
           
            if (x%5==0) {
                SDL_RenderDrawLine( rend_, 
                    ptx(x)-1, ybase_-(x%5 ? 0 : -7), 
                    ptx(x)-1, pty(height_) );
                SDL_RenderDrawLine( rend_, 
                    ptx(x)+1, ybase_-(x%5 ? 0 : -7), 
                    ptx(x)+1, pty(height_) );
            }
            #endif
            if (x!=width_ && x%2==0)
                writeText(ptx(x)+cell_size_/2+2, pty(height_), to_string(x), fonts_[FONT_GRID], ALIGN_CENTER_BOTTOM);
        }
        
        for (int y=0; y<=height_; y++) {
            if (y%5==0) {
                SDL_SetRenderDrawColor( rend_, 0x55, 0x55, 0x55, 0xFF );
                writeText(margins_.left-7, pty(y), to_string(y), fonts_[FONT_GRID], ALIGN_RIGHT_CENTER);
            } else if (y%5==1) {
                SDL_SetRenderDrawColor( rend_, 0xAA, 0xAA, 0xAA, 0xFF );
            }
            #ifndef MAPPIC
            SDL_RenderDrawLine( rend_, 
                margins_.left+(y%5 ? 0 : -7),   pty(y),  
                ptx(width_),                    pty(y));
            if (y%5==0) {
                SDL_RenderDrawLine( rend_, 
                    margins_.left+(y%5 ? 0 : -7),   pty(y)-1,  
                    ptx(width_),                    pty(y)-1);
                SDL_RenderDrawLine( rend_, 
                    margins_.left+(y%5 ? 0 : -7),   pty(y)+1,  
                    ptx(width_),                    pty(y)+1);
            }
            if (y!=height_)
                writeText(ptx(width_)+3, pty(y)-cell_size_/2, to_string(y), fonts_[FONT_GRID], ALIGN_LEFT_CENTER);
            #endif
        }
    }
    
    void drawMapOnly() {
        for (int x=0; x<=width_-1; x++)
            for (int y=0; y<=height_-1; y++) {
                if (map_.isPixelOccupied(x,y)) {
                    SDL_Rect fillRect = { ptx(x), pty(y)-cell_size_, cell_size_, cell_size_ };
                    SDL_SetRenderDrawColor( rend_, 0x77, 0x77, 0x77, 0xFF );
                    SDL_RenderFillRect( rend_, &fillRect );
                }}
    }

    void writeText(int x, int y, string text, TTF_Font* font, TextAlignment alignment) {
        int w,h;
        if(TTF_SizeText(font, text.c_str(),&w,&h)) {
            // perhaps print the current TTF_GetError(), the string can't be rendered...
        } else {
            int dx, dy;
            switch (alignment) {
                case ALIGN_LEFT_TOP:        dx=0,    dy=0;   break;
                case ALIGN_CENTER_TOP:      dx=-w/2, dy=0;   break;
                case ALIGN_RIGHT_TOP:       dx=-w,   dy=0;   break;
                case ALIGN_LEFT_CENTER:     dx=0,    dy=-h/2; break;
                case ALIGN_CENTER_CENTER:   dx=-w/2, dy=-h/2; break;
                case ALIGN_RIGHT_CENTER:    dx=-w,   dy=-h/2; break;
                case ALIGN_LEFT_BOTTOM:     dx=0,    dy=-h;    break;
                case ALIGN_CENTER_BOTTOM:   dx=-w/2, dy=-h;    break;
                case ALIGN_RIGHT_BOTTOM:    dx=-w,   dy=-h;    break;
            }
            
            SDL_Color gridTextColor = {0, 0, 0};
            SDL_Surface* srfMsg = TTF_RenderText_Solid(font, text.c_str(), gridTextColor); 
            SDL_Texture* Message = SDL_CreateTextureFromSurface(rend_, srfMsg); //convert into texture
            SDL_Rect Message_rect({x+dx,y+dy,w,h});
            SDL_RenderCopy(rend_, Message, NULL, &Message_rect);
            SDL_DestroyTexture(Message);
            ///SDL_DestroySurface(Message);
           /// SDL_DestroyColor(gridTextColor);
        }
    }

    
    
    void markDist(int x, int y, int val, bool is_nbp, bool overlap, bool writeDist) {
        int w = (int)round(min(11.0, 2.0*cell_size_/3));
        int h = (int)round(min(11.0, 2.0*cell_size_/3));
        SDL_Rect fillRect = { margins_.left+x*cell_size_-w/2, ybase_-y*cell_size_-h/2, w,h };
        char c = val%10==0 ? 0xAA : 0xDD;
        if (overlap)
            c=0x88;
        if (is_nbp)
            SDL_SetRenderDrawColor( rend_, 0xFF, 0, 0, 0xFF );
        else
            SDL_SetRenderDrawColor( rend_, c, c, 0xFF, 0xFF );
        SDL_RenderFillRect( rend_, &fillRect );
        if (writeDist)
            writeText(ptx(x)+1, pty(y), to_string(val), fonts_[FONT_POINT], ALIGN_CENTER_CENTER);
        //render();
    }
    
    void markDistWave(int x, int y, int val, bool is_nbp, bool overlap, bool writeDist) {
        int w = (int)round(min(11.0, 2.4*cell_size_/3));
        int h = (int)round(min(11.0, 2.4*cell_size_/3));
        if (val%10==0 || val%10==1) {
            SDL_Rect fillRect = { margins_.left+x*cell_size_-w/2, ybase_-y*cell_size_-h/2, w,h };
            SDL_SetRenderDrawColor( rend_, 0xAA, 0xAA, 0xFF, 0xFF );
            SDL_RenderFillRect( rend_, &fillRect );
        }
        //render();
    }
    
    void markStar(int x, int y) {
        SDL_SetRenderTarget(rend_, layers_[LAYER_DIST]);
        int w = 8*cell_size_/10;
        int h = 8*cell_size_/10;
        SDL_Rect rect = { ptx(x)-w/2, pty(y)-h/2, w, h };
        SDL_SetRenderDrawColor( rend_, 0xEE, 0xEE, 0x00, 0xFF );
        SDL_RenderFillRect( rend_, &rect );
        SDL_SetRenderDrawColor( rend_, 0x00, 0x00, 0x00, 0xFF );
        SDL_RenderDrawRect( rend_, &rect );
    }
    
    void selectPoint(int x, int y, bool is_nbp, bool is_checked) {
        int w = is_nbp ? 0.7*cell_size_ : cell_size_;
        int h = is_nbp ? 0.7*cell_size_ : cell_size_;
        SDL_Rect rect = {ptx(x)-w/2, pty(y)-h/2, w, h};
        if (is_nbp)
            SDL_SetRenderDrawColor( rend_, 0xFF, 0x00, 0x00, 0xFF );
        else
            SDL_SetRenderDrawColor( rend_, 0x00, 0x00, 0xFF, 0xFF );
        if (!is_checked) {//draw cross
            SDL_RenderDrawLine( rend_, ptx(x)-w/2, pty(y)-h/2,   ptx(x)+w/2, pty(y)+h/2 );
            SDL_RenderDrawLine( rend_, ptx(x)-w/2, pty(y)+h/2, ptx(x)+w/2, pty(y)-h/2 );
        } else {
            SDL_RenderDrawRect(rend_, &rect);
        }
    }
    
    void onBeamGrow(CWave2& cw, Star& s, Beam& b) {
        SDL_SetRenderTarget(rend_, layers_[LAYER_BEAM]);
        
        SDL_SetRenderDrawColor( rend_, 0xFF, 0xFF, 0xFF, 0x00 );
        SDL_RenderClear( rend_ );
        
        SDL_SetRenderDrawColor( rend_, 0xFF, 0x00, 0x00, 0xFF ); //red
        //SDL_RenderDrawLine( rend_, 10, 10, 500, 500);
        Point p = s.octToPoint(b.start.oct,100*b.start.dx,100*b.start.dy);
        SDL_RenderDrawLine( rend_, ptx(s.c.x), pty(s.c.y), ptx(p.x),  pty(p.y));
        
        SDL_SetRenderDrawColor( rend_, 0x00, 0x00, 0xFF, 0xFF ); //blue
        p = s.octToPoint(b.end.oct,100*b.end.dx,100*b.end.dy);
        SDL_RenderDrawLine( rend_, ptx(s.c.x), pty(s.c.y), ptx(p.x),  pty(p.y));
        
        render();
    }
    
    void onStarAdded(CWave2& cw, Star& s) {
        markStar(s.c.x, s.c.y);
        print(cw);
        render();
    }
    
    void onStarRemoved(CWave2& cw, Star& s) {
        
    }
    
    void onVisitPair(CWave2& cw, Walker& w, bool reg, bool nbp){
        SDL_SetRenderTarget(rend_, layers_[LAYER_TOP]);
        SDL_SetRenderDrawColor( rend_, 0xFF, 0xFF, 0xFF, 0x00 );
        SDL_RenderClear( rend_ );

        Point p = w.star.octToPoint(w.pt.oct, w.pt.x, w.pt.y);
        selectPoint(p.x, p.y, false, reg);
        w.pt.x++;
        Point p2 = w.star.octToPoint(w.pt.oct, w.pt.x, w.pt.y);
        w.pt.x--;
        selectPoint(p2.x, p2.y, true, nbp);
        render();
    }
    
    void onCheckPixel(CWave2& cw, Point& p) {
        SDL_SetRenderTarget(rend_, layers_[LAYER_TOP]);
        SDL_SetRenderDrawColor( rend_, 0x55, 0x55, 0x55, 0xFF );;
        writeText(ptx(p.x)+cell_size_/2, pty(p.y)-cell_size_/2, "?", fonts_[FONT_QMARK], ALIGN_CENTER_CENTER);
        render();
    }
    
    void onSetPointDist(CWave2& cw, Point& p, int val, bool is_nbp) {
        SDL_SetRenderTarget(rend_, layers_[LAYER_DIST]);        
        markDist(p.x, p.y, val, is_nbp, false, true);
        render();
    }
    
    void onSetPointDist(CWave2& cw, Point& p, int val, bool is_nbp, bool overlap, bool to_mark) {
        if (to_mark && !overlap) {
            SDL_SetRenderTarget(rend_, layers_[LAYER_DIST]);        
            markDist(p.x, p.y, val, is_nbp, overlap, true);
            render();
        }
    }
    
    void printStarBeams(Star& s) {
        int beams=0;
        for (forward_list<Beam>::iterator b=s.beams.begin();b!=s.beams.end();++b) {
            printf("          %.2d  start: oct=%d d=(%d,%d) p=(%d,%d), eps=%d\n", ++beams,
                    b->start.oct, b->start.dx, b->start.dy, b->start.x, b->start.y, b->start.eps);
            printf("                end: oct=%d d=(%d,%d) p=(%d,%d), eps=%d\n",
                    b->end.oct,   b->end.dx,   b->end.dy,   b->end.x,   b->end.y,   b->end.eps);
        }
    }    
    
    
    void print(CWave2& cw) {
        printf("==== CWave w,h=(%d,%d), dist=%d, max_dist=%d ====\nproc:\n", 
                 cw.map.width(), cw.map.height(), cw.dist, cw.max_dist);
        for (int k=0; k<cw.proc.size(); k++) {
            char empty = cw.proc[k].beams.empty() ? '=' : '>';
            #ifdef CWAVE2_FLOAT_STARS
                double d = cw.proc[k].dist;
            #else
                double d = -11111.11111;
            #endif
            printf("  %s  (%d,%d) r=%d dist=%f idist=%d beams%c0:\n",
                    k==cw.pr?"pr":"  ", cw.proc[k].c.x, cw.proc[k].c.y, cw.proc[k].r, d, cw.proc[k].idist, empty);
            printStarBeams(cw.proc[k]);
        }
        
        printf("\nadded[1]:\n");
        for (int k=0; k<cw.a[1]; k++) {
            char empty = cw.added[1][k].beams.empty() ? '=' : '>';
            #ifdef CWAVE2_FLOAT_STARS
                double d = cw.added[1][k].dist;
            #else
                double d = -11111.11111;
            #endif
            printf("      (%d,%d) r=%d dist=%f idist=%d beams%c0:\n",
                    cw.added[1][k].c.x, cw.added[1][k].c.y, cw.added[1][k].r, d, cw.added[1][k].idist, empty);
            printStarBeams(cw.added[1][k]);
        }

        printf("\nadded[2]:\n");
        for (int k=0; k<cw.a[2]; k++) {
            char empty = cw.added[2][k].beams.empty() ? '=' : '>';
            #ifdef CWAVE2_FLOAT_STARS
                double d = cw.added[2][k].dist;
            #else
                double d = -11111.11111;
            #endif
            printf("      (%d,%d) r=%d dist=%f idist=%d beams%c0:\n",
                    cw.added[2][k].c.x, cw.added[2][k].c.y, cw.added[2][k].r, d, cw.added[2][k].idist, empty);
            printStarBeams(cw.added[2][k]);
        }
        
        printf("\n\n\n\n");
    }
    
    /*void print(CWave2& cw) {
        printf("==== CWave w,h=(%d,%d), dist=%d, max_dist=%d ====\n", 
                 cw.map.width(), cw.map.height(), cw.dist, cw.max_dist);
        for (forward_list<Star>::iterator si=cw.stars.begin(); si!=cw.stars.end();++si) {
            char mark1 =  si==cw.i ? 'i' : ' ';
            char mark2 =  si==cw.p ? 'p' : ' ';
            char ins0  = si==cw.insert[0] ? '0' : ' ';
            char ins1  = si==cw.insert[1] ? '1' : ' ';
            char ins2  = si==cw.insert[2] ? '2' : ' ';
            char empty = si->beams.empty() ? '=' : '>';
            #ifdef CWAVE2_FLOAT_STARS
                printf("%c%c%c%c%c  (%d,%d) r=%d dist=%f beams%c0:\n",
                       mark1,mark2,ins0,ins1,ins2, si->c.x, si->c.y, si->r, si->dist, empty);
            #else
                char mark3 =  si==cw.ins ? 'I' : ' ';
                printf("%c%c%c  (%d,%d) r=%d dist=%d beams%c0:\n",
                       mark1,mark2,mark3, si->c.x, si->c.y, si->r, si->dist, empty);
            #endif
                
            printStarBeams(*si);
        }
        printf("\n\n\n\n");
    }*/
    
    void onBoundaryAddPixel(CWave2& cw, Star& s, Boundary& b) {
        //if (b.tip.x==0) return;
        Point p1 = s.octToPoint(b.oct, b.x, b.y);
        Point p2 = s.octToPoint(b.oct, b.x+1, b.y + (b.was_incremented ? 1 : 0));
        //drawTiltedRect(ptx(p1.x), pty(p1.y), ptx(p2.x), pty(p2.y), 0.15*cell_size_);
    }
    
    
    
    void onTipUpdate(CWave2& cw, Star& s, Boundary& b) {
        SDL_SetRenderTarget(rend_, layers_[LAYER_BEAM]);
        float wreg = 0.85*cell_size_;
        float wnbp = 0.7*cell_size_;
        int dreg = int(wreg/(2*sqrt(2)));
        int dnbp = int(wnbp/(2*sqrt(2)));
        Point p0 = s.octToPoint(b.oct, b.tip.x-1, b.tip.y);
        Point p1 = s.octToPoint(b.oct, b.tip.x, b.tip.y);
        Point p2 = s.octToPoint(b.oct, b.tip.x+1, b.tip.y);
        if (!b.tip.offline) {
            SDL_SetRenderDrawColor( rend_, 0x00, 0x00, 0xFF, 0xFF );
            drawTiltedRect(ptx(p1.x)-dreg, pty(p1.y)-dreg, ptx(p1.x)+dreg, pty(p1.y)+dreg, wreg/2);
            if (b.tip.nbp) {
                SDL_SetRenderDrawColor( rend_, 0xFF, 0x00, 0x00, 0xFF );
                drawTiltedRect(ptx(p2.x)-dnbp, pty(p2.y)-dnbp, ptx(p2.x)+dnbp, pty(p2.y)+dnbp, wnbp/2);
            }
        } else {
            SDL_SetRenderDrawColor( rend_, 0x00, 0x00, 0xFF, 0xFF );
            drawTiltedRect(ptx(p0.x)-2*dreg, pty(p0.y), ptx(p0.x)+2*dreg, pty(p0.y), 2);
            drawTiltedRect(ptx(p0.x), pty(p0.y)-2*dreg, ptx(p0.x), pty(p0.y)+2*dreg, 2);
            if (b.tip.nbp) {
                SDL_SetRenderDrawColor( rend_, 0xFF, 0x00, 0x00, 0xFF );
                drawTiltedRect(ptx(p1.x)-dnbp, pty(p1.y)-dnbp, ptx(p1.x)+dnbp, pty(p1.y)+dnbp, wnbp/2);
            }
        }
        render();
    }
    
    void drawTiltedRect(int x1, int y1, int x2, int y2, float d) {
       //printf(">>>>>>>>>>>>> %d, %d, %d, %d\n", x1, y1, x2, y2);
        int dx=x2-x1, dy=y2-y1;
        float l = sqrt(dx*dx+dy*dy);
        SDL_RenderDrawLine( rend_, int(x1-d*dy/l), int(y1+d*dx/l),  int(x2-d*dy/l), int(y2+d*dx/l) );
        SDL_RenderDrawLine( rend_, int(x2-d*dy/l), int(y2+d*dx/l),  int(x2+d*dy/l), int(y2-d*dx/l) );
        SDL_RenderDrawLine( rend_, int(x2+d*dy/l), int(y2-d*dx/l),  int(x1+d*dy/l), int(y1-d*dx/l) );
        SDL_RenderDrawLine( rend_, int(x1+d*dy/l), int(y1-d*dx/l),  int(x1-d*dy/l), int(y1+d*dx/l) );
        render();
    }
    

    
    void displayDistMap(CompoundMap& map, bool displayTextOnPoints) { 
        int dist;
        for (int y=1; y<map.height(); y++) {
            ///printf("    DEBUG: y=%d", y); fflush(stdout);
            for (int x=1; x<map.width(); x++) {
                dist = map.getPoint(x,y);
                if (dist != MAP_POINT_UNEXPLORED) {
                    markDistWave(x,y,dist,dist%2, false, displayTextOnPoints);
                }
            }
        }
    }

    void displayResults(CompoundMap& map, int x, int y, bool displayTextOnPoints) { 
        ///printf("    DEBUG: displayResults 1\n");
        SDL_SetRenderTarget(rend_, layers_[LAYER_DIST]);
        SDL_SetRenderDrawColor( rend_, 0xFF, 0xFF, 0xFF, 0x00 );
        SDL_RenderClear( rend_ );
        ///printf("    DEBUG: displayResults 10\n");
        displayDistMap(map, displayTextOnPoints);
        selectPoint(x, y, false, true);
        render();
    }
    
    void clear() {
        SDL_SetRenderTarget(rend_, layers_[LAYER_DIST]);
        SDL_SetRenderDrawColor( rend_, 0xFF, 0xFF, 0xFF, 0x00 );
        SDL_RenderClear( rend_ );
        SDL_SetRenderTarget(rend_, layers_[LAYER_BEAM]);
        SDL_SetRenderDrawColor( rend_, 0xFF, 0xFF, 0xFF, 0x00 );
        SDL_RenderClear( rend_ );
        SDL_SetRenderTarget(rend_, layers_[LAYER_TOP]);
        SDL_SetRenderDrawColor( rend_, 0xFF, 0xFF, 0xFF, 0x00 );
        SDL_RenderClear( rend_ );
        
        render();
    }
};

}

#endif