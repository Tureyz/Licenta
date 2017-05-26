#include "Sphere.h"

Rendering::Models::Sphere::Sphere(Managers::ModelManager *modelManager, Managers::ISimulationManager *simulationManager) : Model(modelManager, simulationManager)
{
	SetObjectType(Simulation::PhysicsObjectType::OBJ_SPHERE);	
}

Rendering::Models::Sphere::~Sphere()
{
}