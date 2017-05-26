#include "Cube.h"

Rendering::Models::Cube::Cube(Managers::ModelManager *modelManager, Managers::ISimulationManager *simulationManager) : Model(modelManager, simulationManager)
{
	SetObjectType(Simulation::PhysicsObjectType::OBJ_CUBE);
}

Rendering::Models::Cube::~Cube()
{
}