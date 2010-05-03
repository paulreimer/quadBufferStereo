#include "SoftBodyPhysics.h"

//--------------------------------------------------------------
void
PhysicsEngine::setup()
{
	int s=32,h=6+12,r=31;
	//TRACEDEMO
	btSoftBody*		psb=btSoftBodyHelpers
	::CreatePatch(m_softBodyWorldInfo,
				  btVector3(-s,h,-s),
				  btVector3(+s,h,-s),
				  btVector3(-s,h,+s),
				  btVector3(+s,h,+s),
				  r,r,
				  1+2+4+8,true);
	
	psb->getCollisionShape()->setMargin(0.5);
	btSoftBody::Material* pm=psb->appendMaterial();
	pm->m_kLST		=	0.4;
	pm->m_flags		-=	btSoftBody::fMaterial::DebugDraw;
	psb->generateBendingConstraints(2,pm);
	psb->setTotalMass(150);
	m_dynamicsWorld->addSoftBody(psb);	
	
	m_dynamicsWorld->setDrawFlags(fDrawFlags::Nodes);
}

//--------------------------------------------------------------
void
PhysicsEngine::destroy()
{}

//--------------------------------------------------------------
void
PhysicsEngine::update()
{
	if(m_drag)
		m_node->m_v = btVector3(0,0,0);
}

//--------------------------------------------------------------
void
PhysicsEngine::draw()
{
}
