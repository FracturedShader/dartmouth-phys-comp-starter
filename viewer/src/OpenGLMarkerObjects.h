//#####################################################################
// Opengl interactive object
// Copyright (c) (2018-), Bo Zhu, boolzhu@gmail.com
// This file is part of SLAX, whose distribution is governed by the LICENSE file.
//#####################################################################
#ifndef __OpenGLMarkerObjects_h__
#define __OpenGLMarkerObjects_h__
#include "glm.hpp"
#include "Mesh.h"
#include "OpenGLObject.h"

class OpenGLBackground : public OpenGLObject
{public:typedef OpenGLObject Base;
	Box<2> box;	
	std::string tex_name;
	real depth;
	bool use_fbo_tex;

	OpenGLBackground();

	void Set_Box(const Vector2& min_corner,const Vector2& max_corner){box=Box<2>(min_corner,max_corner);}
	void Set_Texture(const std::string& _tex_name){use_vtx_tex=true;tex_name=_tex_name;}
	void Set_Depth(const real _depth){depth=_depth;}
	void Set_Fbo(){}
	virtual void Initialize();
	virtual void Display() const;
};

class OpenGLAxes : public OpenGLObject
{public:typedef OpenGLObject Base;
	real axis_length=(real).5;
	bool use_2d_display=false;

	OpenGLAxes(){name="axes";polygon_mode=PolygonMode::Wireframe;}

	virtual void Initialize();
	virtual void Display() const;
};

class OpenGLPoint : public OpenGLObject
{public:typedef OpenGLObject Base;using Base::color;
	Vector3 pos=Vector3::Zero();
	GLfloat point_size=16.f;

	OpenGLPoint(){name="point";color=OpenGLColor::Red();polygon_mode=PolygonMode::Fill;}
	
	virtual void Initialize();
	virtual void Update_Data_To_Render();
	virtual void Display() const;
};

class OpenGLTriangle : public OpenGLObject
{public:typedef OpenGLObject Base;using Base::color;using Base::line_width;
	ArrayF<Vector3,3> vtx;

	OpenGLTriangle(){name="triangle";color=OpenGLColor::Red();polygon_mode=PolygonMode::Fill;}
	
	virtual void Initialize();
	virtual void Update_Data_To_Render();
	virtual void Display() const;
};

class OpenGLCircle : public OpenGLObject
{public: typedef OpenGLObject Base;
	Vector3 pos=Vector3::Zero();
	real radius=(real).1;
	glm::mat4 model=glm::mat4(1.f);

	int n=16;
	Array<Vector3> vtx;

	OpenGLCircle(){name="circle";color=OpenGLColor::Green();polygon_mode=PolygonMode::Fill;}

	virtual void Initialize();
	virtual void Update_Data_To_Render();
	virtual void Display() const;
	virtual void Update_Model_Matrix();
};

#endif