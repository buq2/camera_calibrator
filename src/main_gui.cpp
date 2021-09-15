#include <GL/glew.h>
#include "gui.hh"
#include "imgui.h"

void Gui()
{

}

int main(int argc, char *argv[]) {
  GuiWindow conf("win");

  conf.SetAutoResize(false);
  conf.Initialize();
  while (!conf.Draw([&]() { Gui(); })) {
  }

  return 0;
}
