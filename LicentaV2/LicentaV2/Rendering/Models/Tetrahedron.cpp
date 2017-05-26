#include "Tetrahedron.h"

Rendering::Models::Tetrahedron::Tetrahedron(Managers::ModelManager *modelManager, Managers::ISimulationManager *simulationManager) : Model(modelManager, simulationManager)
{
	SetObjectType(Simulation::PhysicsObjectType::OBJ_TETRAHEDRON);
}

Rendering::Models::Tetrahedron::~Tetrahedron()
{
}