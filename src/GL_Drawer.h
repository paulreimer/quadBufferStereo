/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
#pragma once

class btCollisionShape;
class btShapeHull;
#include "LinearMath/btAlignedObjectArray.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btIDebugDraw.h"

#include "BulletCollision/CollisionShapes/btShapeHull.h"

/// OpenGL shape drawing
class GL_Drawer
: public btIDebugDraw
{
protected:
	struct ShapeCache
	{
	struct Edge { btVector3 n[2];int v[2]; };
	ShapeCache(btConvexShape* s) : m_shapehull(s) {}
	btShapeHull					m_shapehull;
	btAlignedObjectArray<Edge>	m_edges;
	};
	//clean-up memory of dynamically created shape hulls
	btAlignedObjectArray<ShapeCache*>	m_shapecaches;
	unsigned int						m_texturehandle;
	bool								m_textureenabled;
	bool								m_textureinitialized;
	

	ShapeCache*							cache(btConvexShape*);

	int m_debugMode;

public:
	GL_Drawer();

	virtual ~GL_Drawer();

	///drawOpenGL might allocate temporary memoty, stores pointer in shape userpointer
	virtual void		drawOpenGL(btScalar* m, const btCollisionShape* shape, const btVector3& color,int	debugMode,const btVector3& worldBoundsMin,const btVector3& worldBoundsMax);
	virtual void		drawShadow(btScalar* m, const btVector3& extrusion,const btCollisionShape* shape,const btVector3& worldBoundsMin,const btVector3& worldBoundsMax);
	
	bool		enableTexture(bool enable) { bool p=m_textureenabled;m_textureenabled=enable;return(p); }
	bool		hasTextureEnabled() const
	{
		return m_textureenabled;
	}
	
	void	drawLine(const btVector3& from,const btVector3& to,const btVector3& fromColor, const btVector3& toColor);
	void	drawLine(const btVector3& from,const btVector3& to,const btVector3& color);
	void	drawContactPoint(const btVector3& PointOnB,const btVector3& normalOnB,btScalar distance,int lifeTime,const btVector3& color);
	
	void	reportErrorWarning(const char* warningString);
	
	void	draw3dText(const btVector3& location,const char* textString);
	
	void	setDebugMode(int debugMode);
	int		getDebugMode() const
	{ return m_debugMode;}

	static void		drawCylinder(float radius,float halfHeight, int upAxis);
	void			drawSphere(btScalar r, int lats, int longs);
	static void		drawCoordSystem();	
};

void OGL_displaylist_register_shape(btCollisionShape * shape);
void OGL_displaylist_clean();
