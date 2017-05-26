#include "Cylinder.h"

Rendering::Models::Cylinder::Cylinder(Managers::ModelManager *modelManager, Managers::ISimulationManager *simulationManager) : Model(modelManager, simulationManager)
{
	SetObjectType(Simulation::PhysicsObjectType::OBJ_CYLINDER);
}

Rendering::Models::Cylinder::~Cylinder()
{

}