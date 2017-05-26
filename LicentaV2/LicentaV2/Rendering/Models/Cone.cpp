#include "Cone.h"

Rendering::Models::Cone::Cone(Managers::ModelManager *modelManager, Managers::ISimulationManager *simulationManager) : Model(modelManager, simulationManager)
{
	SetObjectType(Simulation::PhysicsObjectType::OBJ_CONE);
}

Rendering::Models::Cone::~Cone()
{

}