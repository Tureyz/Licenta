#include "Plotter.h"
#include <stdlib.h>
#include <windows.h>
#include "../Core/Utils.hpp"

void Benchmark::Plotter::GeneratePlotsFromRawData()
{
	SetCurrentDirectory((LPCWSTR)Core::BENCHMARK_FOLDER.c_str());
	system("gnuplot script.plt");
	system("gnuplot script2.plt");
	SetCurrentDirectory((LPCWSTR)"..\\");
}
