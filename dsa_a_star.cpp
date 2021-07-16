#include <iostream>
#include <vector>
#include <list>
#include <stdlib.h>
//#include <Windows.h>
//#include <GL/gl.h>
#include "SDL2/SDL.h"
#include <math.h>

using namespace std;

#pragma comment(lib, "SDL2/lib/x64/SDL2.lib")
#pragma comment(lib, "SDL2/lib/x64/SDL2main.lib")
//#pragma comment(lib, "opengl32.lib")

#define map_w 20
#define map_h 20
#define inf 9999

SDL_Window *window;
SDL_Renderer *renderer;
SDL_Rect *grid = new SDL_Rect[map_w*map_h];


struct node
{
  bool obstacle;
  bool visited;
  float local_goal;
  float global_goal;
  int x;
  int y;
  vector<node*> neighbour;
  node* parent;
};
struct node *nodes = new node[map_w * map_h];
struct node *nodestart = nullptr;
struct node *nodeend = nullptr;

void init()
{
  for(int x=0;x<map_w;x++)
  {
    for(int y=0;y<map_h;y++)
    {
      nodes[y*map_h + x].obstacle = false;
      nodes[y*map_h + x].visited = false;
      nodes[y*map_h + x].x = x;
      nodes[y*map_h + x].y = y;
      nodes[y*map_h + x].parent = nullptr;
      //(*(nodes + i*map_h +j)).parent = nullptr;
      grid[y*map_h + x].x = x*30;
      grid[y*map_h + x].y = y*30;
      grid[y*map_h + x].w = 28;
      grid[y*map_h + x].h = 28;
    }
  }

  for(int x=0;x<map_w;x++)
  {
    for(int y=0;y<map_h;y++)
    {
      if(y>0)
        nodes[y*map_w+ x].neighbour.push_back(&nodes[(y-1)*map_w + (x+0)]);
      if(y<(map_w-1))
        nodes[y*map_w+ x].neighbour.push_back(&nodes[(y+1)*map_w + (x+0)]);
      if(x>0)
        nodes[y*map_h+ x].neighbour.push_back(&nodes[(y+0)*map_w + (x-1)]);
      if(x<(map_h-1))
        nodes[y*map_h+ x].neighbour.push_back(&nodes[(y+0)*map_w + (x+1)]);
    }
  }

  nodestart = &nodes[(map_h / 2) * map_w + 1];
  nodeend = &nodes[(map_h / 2) *   map_w + map_w-2];
}

void astar()
{
  for(int x=0;x<map_w;x++)
  {
    for(int y=0;y<map_h;y++)
    {
      nodes[y*map_w + x].visited = false;
      nodes[y*map_w + x].local_goal = inf;
      nodes[y*map_w + x].global_goal = inf;
      nodes[y*map_w + x].parent = nullptr;
    }
  } 

  auto distance = [](node* a, node* b) // For convenience
  {
    return sqrtf((a->x - b->x)*(a->x - b->x) + (a->y - b->y)*(a->y - b->y));
  };

  auto heuristic = [distance](node* a, node* b) // So we can experiment with heuristic
  {
    return distance(a, b);
  };
  
  node *nodeCurrent = nodestart;
  nodestart->local_goal = 0.0f;
  nodestart->global_goal = heuristic(nodestart, nodeend);

  list<node*> list_to_test_nodes;
  list_to_test_nodes.push_back(nodestart);

  while (!list_to_test_nodes.empty())
  {
    list_to_test_nodes.sort([](const node* lhs, const node* rhs){ return lhs->local_goal < rhs->global_goal; } );

    while(!list_to_test_nodes.empty() && list_to_test_nodes.front()->visited)
      list_to_test_nodes.pop_front();

    if (list_to_test_nodes.empty())
      break;
      
    nodeCurrent = list_to_test_nodes.front();
    nodeCurrent->visited = true;

    for (auto nodeNeighbour : nodeCurrent->neighbour)
    {
      if (!nodeNeighbour->visited && nodeNeighbour->obstacle == 0)
        list_to_test_nodes.push_back(nodeNeighbour);

      float PossiblyLowerGoal = nodeCurrent->local_goal + distance(nodeCurrent, nodeNeighbour);

      if (PossiblyLowerGoal < nodeNeighbour->local_goal)
      {
        nodeNeighbour->parent = nodeCurrent;
        nodeNeighbour->local_goal = PossiblyLowerGoal;
        nodeNeighbour->global_goal = nodeNeighbour->local_goal + heuristic(nodeNeighbour, nodeend);
      }
    }
  }
}

void render()
{
  for(int x=0;x<map_w;x++)
  {
    for(int y=0;y<map_h;y++)
    {
      for(auto n:nodes[y*map_h +x].neighbour)
      {
        SDL_SetRenderDrawColor(renderer, 128, 0, 0, 255);
        SDL_RenderDrawLine(renderer,nodes[y*map_h + x].x * 30+15, nodes[y*map_h + x].y * 30+15, n->x*30 +15,n->y*30 +15);
      }
    }
  }
  for(int x=0;x<map_w;x++)
  {
    for(int y=0;y<map_h;y++)
    {
      if(&nodes[y*map_h + x] == nodeend)
        SDL_SetRenderDrawColor(renderer,177,156,217,255);//(renderer,173,216,230,255);
      else if(&nodes[y*map_h + x] == nodestart)
        SDL_SetRenderDrawColor(renderer,255,192,203,255);
      else if(nodes[y*map_h +x].obstacle)
        SDL_SetRenderDrawColor(renderer,128,128,128,255);
      else
        SDL_SetRenderDrawColor(renderer, 128, 0, 0, 255);

      SDL_RenderFillRect(renderer, &grid[y*map_h + x]);

      if (nodeend != nullptr)
      {
        node *p = nodeend;
        while (p->parent != nullptr)
        {
          SDL_SetRenderDrawColor(renderer,212,117,55,255);
          SDL_RenderDrawLine(renderer, p->x * 30+15, p->y * 30+15, p->parent->x*30 +15, p->parent->y*30 +15);
        // Set next node to this node's parent
          p = p->parent;
        }
      }
    }
  }
  SDL_RenderPresent(renderer); 
}




int main(int argc, char *argv[]) 
{

  SDL_Init(SDL_INIT_EVERYTHING);

  window = SDL_CreateWindow("An SDL2 window", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, 600, 600, SDL_WINDOW_OPENGL);
  renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);

  if (window == NULL)
  {
    printf("Could not create window: %s\n", SDL_GetError());
    return 1;
  }

  init();

  bool running = true;
  while (running) 
  {
    SDL_Event event;
    while (SDL_PollEvent(&event)) 
    {
      if (event.type == SDL_QUIT) 
      {
        running = false;
        break;
      }
      bool ctrl = (KMOD_CTRL & SDL_GetModState());
      if(event.type == SDL_MOUSEBUTTONUP)
      {
        if(event.button.button == SDL_BUTTON_RIGHT && event.button.clicks == 1 )
        {
          int x = event.button.x / 30;
          int y = event.button.y / 30;
          nodes[y*map_h + x].obstacle = !(nodes[y*map_h + x].obstacle);
        }
        if(event.button.button == SDL_BUTTON_LEFT && ctrl == 0)
        {
          int x = event.button.x / 30;
          int y = event.button.y / 30;
          nodestart = &nodes[y*map_h + x];
        }
        if(event.button.button == SDL_BUTTON_LEFT && ctrl == 1)
        {
          int x = event.button.x / 30;
          int y = event.button.y / 30;
          nodeend = &nodes[y*map_h + x];
        }
      }
    }

    astar();
    render();
  }
  SDL_Quit();
  return 0;
}
