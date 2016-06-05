#include "Plotter.h"
#include <stdlib.h>
#include <windows.h>
#include "../Core/Utils.hpp"

void Benchmark::Plotter::GeneratePlotsFromRawData()
{
	SetCurrentDirectory(Core::benchmarkFolder.c_str());
	system("gnuplot script.plt");
	SetCurrentDirectory("..\\..\\");
}
