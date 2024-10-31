#ifndef __GridFluidDriver_h__
#define __GridFluidDriver_h__
#include <random>
#include "Common.h"
#include "Driver.h"
#include "Particles.h"
#include "OpenGLMesh.h"
#include "OpenGLCommon.h"
#include "OpenGLWindow.h"
#include "OpenGLViewer.h"
#include "OpenGLMarkerObjects.h"
#include "OpenGLParticles.h"

class GridFluidDriver : public Driver, public OpenGLViewer
{using Base=Driver;
	double dt=.02;
	double t = 0;
	GridFluid fluid;
	double v_rescale=(double).05;
	bool add_particle=false;

	OpenGLSegmentMesh* opengl_vectors=nullptr;							////vector field for velocity
	OpenGLPolygon* opengl_polygon=nullptr;								////a rectangle for domain boundary
	std::vector<OpenGLSolidCircle*> opengl_circles;							////passive particles
	Hashset<int> invis_particles;
	OpenGLColoredTriangleMesh* opengl_mesh=nullptr;						////density field

	//////////////////////////////////////////////////////////////////////////
	////specify the following flags to control visualization
	bool draw_velocity=true;
	bool draw_density=true;
	bool draw_particles=true;
	bool use_shaders=true;
	//////////////////////////////////////////////////////////////////////////

	GLuint px_width = 256;
	GLuint px_height = px_width;
	GLuint smoke_px_width = 2048;
	GLuint smoke_px_height = smoke_px_width;
	float px_dx = 1.0f / px_width;

	std::shared_ptr<OpenGLFbos::OpenGLFbo> boundary_mask_fbo;
	std::shared_ptr<OpenGLFbos::OpenGLFbo> u_fbos[2];
	std::shared_ptr<OpenGLFbos::OpenGLFbo> den_fbos[2];
	std::shared_ptr<OpenGLFbos::OpenGLFbo> div_u_fbo;
	std::shared_ptr<OpenGLFbos::OpenGLFbo> p_fbos[2];
	std::shared_ptr<OpenGLFbos::OpenGLFbo> vor_fbo;

	std::shared_ptr<OpenGLShaderProgram> clear_shader;
	std::shared_ptr<OpenGLShaderProgram> fill_shader;
	std::shared_ptr<OpenGLShaderProgram> copy_shader;

	std::shared_ptr<OpenGLShaderProgram> apply_source_shader;
	std::shared_ptr<OpenGLShaderProgram> advect_shader;
	std::shared_ptr<OpenGLShaderProgram> div_u_shader;
	std::shared_ptr<OpenGLShaderProgram> poisson_iter_shader;
	std::shared_ptr<OpenGLShaderProgram> p_correct_u_shader;
	std::shared_ptr<OpenGLShaderProgram> calc_vor_shader;
	std::shared_ptr<OpenGLShaderProgram> vor_conf_shader;

	OpenGLUVMesh* uv_quad;
	OpenGLTexturedTriangleMesh* smoke_mesh = nullptr;

	static glm::vec3 HSVToRGB(double h, double s, double v)
	{
		const double c = v * s;
		const double x = c * (1.0 - std::abs(fmod(h / 60.0, 2.0) - 1.0));
		const double m = v - c;

		const int base_idx = static_cast<int>(h / 120.0);
		const int shift = static_cast<int>(h / 60) % 2;

		glm::vec3 rgb{m, m, m};

		rgb[(base_idx + shift) % 3] += static_cast<float>(c);
		rgb[(base_idx + 1 - shift) % 3] += static_cast<float>(x);

		return rgb;
	}

	std::shared_ptr<OpenGLFbos::OpenGLFbo> CreatePosFbo(const std::string& name)
	{
		auto fbo = OpenGLFbos::Get_Fbo(name, 2);

		fbo->Resize(px_width, px_height);

		return fbo;
	}

	void RenderToFBO(std::shared_ptr<OpenGLFbos::OpenGLFbo>& fbo, std::function<void()> render_fn)
	{
		glViewport(0, 0, fbo->width, fbo->height);
		fbo->Bind();

		render_fn();

		fbo->Unbind();
		glViewport(0, 0, opengl_window->win_w, opengl_window->win_h);
	}

	void RenderToFBONoDepth(std::shared_ptr<OpenGLFbos::OpenGLFbo>& fbo, std::function<void()> render_fn)
	{
		RenderToFBO(fbo, [&]()
			{
				glDisable(GL_DEPTH_TEST);
				render_fn();
				glEnable(GL_DEPTH_TEST);
			});
	}

	void RenderToFBONoDepthNoBounds(std::shared_ptr<OpenGLFbos::OpenGLFbo>& fbo, std::function<void()> render_fn)
	{
		RenderToFBONoDepth(fbo, [&]()
			{
				glEnable(GL_SCISSOR_TEST);
				glScissor(1, 1, fbo->width - 2, fbo->height - 2);
				render_fn();
				glDisable(GL_SCISSOR_TEST);
			});
	}

	Vector2 ToFluidUV(const Vector2& world_pos)
	{
		return { world_pos[0] / 2, world_pos[1] };
	}

public:
	virtual void Initialize()
	{
		fluid.Initialize();
		fluid.Initialize_Visualization_Particles();
		OpenGLViewer::Initialize();
	}

	////initialize visualization data, called in OpenGLViewer::Initialize()
	virtual void Initialize_Data()
	{
		////initialize vector field (write all vectors as segments)
		opengl_vectors=Add_Interactive_Object<OpenGLSegmentMesh>();
		opengl_vectors->mesh.elements.resize(fluid.node_num);
		opengl_vectors->mesh.vertices->resize(fluid.node_num*2);

		for(int i=0;i<fluid.particles.Size();i++){
			Add_Solid_Circle(i);}

		for(int i=0;i<fluid.node_num;i++){
			opengl_vectors->mesh.elements[i]=Vector2i(i*2,i*2+1);
			(*opengl_vectors->mesh.vertices)[i*2]=V3(fluid.grid.Node(fluid.grid.Node_Coord(i)));
			Vector2 pos2=fluid.grid.Node(fluid.grid.Node_Coord(i))+fluid.u[i]*v_rescale;
			(*opengl_vectors->mesh.vertices)[i*2+1]=V3(pos2);}
		opengl_vectors->Set_Data_Refreshed();
		opengl_vectors->Initialize();

		////initialize polygon
		double width = use_shaders ? 1 : 2;
		opengl_polygon=Add_Interactive_Object<OpenGLPolygon>();
		opengl_polygon->vtx.push_back(Vector3::Zero());
		opengl_polygon->vtx.push_back(Vector3::Unit(0)*width);
		opengl_polygon->vtx.push_back(Vector3::Unit(0)*width+Vector3::Unit(1)*(double)1);
		opengl_polygon->vtx.push_back(Vector3::Unit(1)*(double)1);
		Set_Color(opengl_polygon,OpenGLColor(.0,1.,1.,1.));
		Set_Line_Width(opengl_polygon,4.f);
		opengl_polygon->Set_Data_Refreshed();
		opengl_polygon->Initialize();

		opengl_mesh=Add_Interactive_Object<OpenGLColoredTriangleMesh>();
		
		int cell_num=fluid.grid.cell_counts.prod();
		for(int i=0;i<cell_num;i++){
			Vector2i cell=fluid.grid.Cell_Coord(i);
			Vector2 pos1=fluid.grid.Node(cell);
			Vector2 pos2=pos1+Vector2::Unit(0)*fluid.grid.dx;
			Vector2 pos3=pos1+Vector2::Unit(0)*fluid.grid.dx+Vector2::Unit(1)*fluid.grid.dx;
			Vector2 pos4=pos1+Vector2::Unit(1)*fluid.grid.dx;

			int n=(int)opengl_mesh->mesh.Vertices().size();
			opengl_mesh->mesh.Vertices().push_back(V3(pos1));
			opengl_mesh->mesh.Vertices().push_back(V3(pos2));
			opengl_mesh->mesh.Vertices().push_back(V3(pos3));
			opengl_mesh->colors.push_back(0.);
			opengl_mesh->colors.push_back(0.);
			opengl_mesh->colors.push_back(0.);
			opengl_mesh->mesh.Elements().push_back(Vector3i(n,n+1,n+2));

			n=(int)opengl_mesh->mesh.Vertices().size();
			opengl_mesh->mesh.Vertices().push_back(V3(pos1));
			opengl_mesh->mesh.Vertices().push_back(V3(pos3));
			opengl_mesh->mesh.Vertices().push_back(V3(pos4));
			opengl_mesh->colors.push_back(0.);
			opengl_mesh->colors.push_back(0.);
			opengl_mesh->colors.push_back(0.);
			opengl_mesh->mesh.Elements().push_back(Vector3i(n,n+1,n+2));}
		
		opengl_mesh->Set_Data_Refreshed();
		opengl_mesh->Initialize();

		boundary_mask_fbo = OpenGLFbos::Get_Fbo("fluid_boundary");
		boundary_mask_fbo->Resize(px_width, px_height);

		den_fbos[0] = OpenGLFbos::Get_Fbo("den_fbo_0");
		den_fbos[1] = OpenGLFbos::Get_Fbo("den_fbo_1");

		den_fbos[0]->Resize(smoke_px_width, smoke_px_height);
		den_fbos[1]->Resize(smoke_px_width, smoke_px_height);

		u_fbos[0] = CreatePosFbo("u_fbo_0");
		u_fbos[1] = CreatePosFbo("u_fbo_1");
		div_u_fbo = CreatePosFbo("div_u_fbo");
		p_fbos[0] = CreatePosFbo("p_fbo_0");
		p_fbos[1] = CreatePosFbo("p_fbo_1");
		vor_fbo = CreatePosFbo("vor_fbo");

		clear_shader = OpenGLShaderLibrary::Get_Shader("zero_tex");
		fill_shader = OpenGLShaderLibrary::Get_Shader("one_tex");
		copy_shader = OpenGLShaderLibrary::Get_Shader("copy_tex");

		apply_source_shader = OpenGLShaderLibrary::Get_Shader("gf_apply_source");
		advect_shader = OpenGLShaderLibrary::Get_Shader("gf_advect");
		div_u_shader = OpenGLShaderLibrary::Get_Shader("gf_calc_div_u");
		poisson_iter_shader = OpenGLShaderLibrary::Get_Shader("gf_poisson_iter");
		p_correct_u_shader = OpenGLShaderLibrary::Get_Shader("gf_p_correct_u");
		calc_vor_shader = OpenGLShaderLibrary::Get_Shader("gf_calc_vor");
		vor_conf_shader = OpenGLShaderLibrary::Get_Shader("gf_vor_conf");

		uv_quad = Add_Object<OpenGLUVMesh>();
		uv_quad->Update_Data_To_Render();

		smoke_mesh = Add_Object<OpenGLTexturedTriangleMesh>(false);

		smoke_mesh->mesh.Vertices().emplace_back(0, 0, 0);
		smoke_mesh->tex_coords.emplace_back(0, 0);
		smoke_mesh->mesh.Vertices().emplace_back(1, 0, 0);
		smoke_mesh->tex_coords.emplace_back(1, 0);
		smoke_mesh->mesh.Vertices().emplace_back(1, 1, 0);
		smoke_mesh->tex_coords.emplace_back(1, 1);
		smoke_mesh->mesh.Vertices().emplace_back(0, 1, 0);
		smoke_mesh->tex_coords.emplace_back(0, 1);

		smoke_mesh->mesh.Elements().emplace_back(0, 1, 2);

		smoke_mesh->mesh.Elements().emplace_back(2, 3, 0);

		smoke_mesh->main_tex = den_fbos[0]->tex_index;

		smoke_mesh->Set_Data_Refreshed();
		smoke_mesh->Initialize();

		RenderToFBONoDepth(boundary_mask_fbo, [&]()
			{
				fill_shader->Begin();
				uv_quad->DrawElements();
				fill_shader->End();

				glEnable(GL_SCISSOR_TEST);
				glScissor(1, 1, px_width - 2, px_height - 2);

				clear_shader->Begin();
				uv_quad->DrawElements();
				clear_shader->End();

				glDisable(GL_SCISSOR_TEST);
			});

		if (use_shaders)
		{
			opengl_vectors->visible = false;
			opengl_vectors->Set_Data_Refreshed();
			
			opengl_mesh->visible = false;
			opengl_mesh->Set_Data_Refreshed();

			draw_particles = false;
			draw_velocity = false;
		}
		else
		{
			smoke_mesh->visible = false;
		}
	}

	void ApplySource()
	{
		RenderToFBONoDepthNoBounds(u_fbos[0], [&]()
			{
				apply_source_shader->Begin();
				apply_source_shader->Set_Uniform("src_xyr", glm::vec4(fluid.src_pos[0], fluid.src_pos[1], fluid.src_radius, fluid.src_radius));
				apply_source_shader->Set_Uniform("src_val", glm::vec3(0.25, 0, 0));

				uv_quad->DrawElements();

				apply_source_shader->End();
			});

		RenderToFBONoDepthNoBounds(den_fbos[0], [&]()
			{
				apply_source_shader->Begin();
				apply_source_shader->Set_Uniform("src_xyr", glm::vec4(fluid.src_pos[0], fluid.src_pos[1], fluid.src_radius, fluid.src_radius));
				apply_source_shader->Set_Uniform("src_val", HSVToRGB(fmod(t * 36, 360), 1, 1));

				uv_quad->DrawElements();

				apply_source_shader->End();
			});
	}

	void Advect()
	{
		RenderToFBONoDepth(u_fbos[1], [&]()
			{
				advect_shader->Begin();
				advect_shader->Bind_Texture2D("u", u_fbos[0]->tex_index, 0);
				advect_shader->Set_Uniform("val", 0);
				advect_shader->Set_Uniform("dt", static_cast<GLfloat>(dt));

				uv_quad->DrawElements();

				advect_shader->End();
			});

		RenderToFBONoDepth(den_fbos[1], [&]()
			{
				advect_shader->Begin();
				advect_shader->Bind_Texture2D("u", u_fbos[0]->tex_index, 0);
				advect_shader->Bind_Texture2D("val", den_fbos[0]->tex_index, 1);
				advect_shader->Set_Uniform("dt", static_cast<GLfloat>(dt));

				uv_quad->DrawElements();

				advect_shader->End();
			});

		RenderToFBONoDepth(u_fbos[0], [&]()
			{
				copy_shader->Begin();
				copy_shader->Bind_Texture2D("main_tex", u_fbos[1]->tex_index, 0);

				uv_quad->DrawElements();

				copy_shader->End();
			});

		RenderToFBONoDepth(den_fbos[0], [&]()
			{
				copy_shader->Begin();
				copy_shader->Bind_Texture2D("main_tex", den_fbos[1]->tex_index, 0);

				uv_quad->DrawElements();

				copy_shader->End();
			});
	}

	void ConfineVortices()
	{
		RenderToFBONoDepthNoBounds(vor_fbo, [&]()
			{
				calc_vor_shader->Begin();
				calc_vor_shader->Bind_Texture2D("u", u_fbos[1]->tex_index, 0);
				calc_vor_shader->Set_Uniform("dx", px_dx);

				uv_quad->DrawElements();

				calc_vor_shader->End();
			});

		RenderToFBONoDepthNoBounds(u_fbos[0], [&]()
			{
				vor_conf_shader->Begin();
				vor_conf_shader->Bind_Texture2D("u", u_fbos[1]->tex_index, 0);
				vor_conf_shader->Bind_Texture2D("vor", vor_fbo->tex_index, 1);
				vor_conf_shader->Set_Uniform("dx", px_dx);
				vor_conf_shader->Set_Uniform("dt", static_cast<GLfloat>(dt));
				vor_conf_shader->Set_Uniform("vor_conf_coeff", 2.0f);

				uv_quad->DrawElements();

				vor_conf_shader->End();
			});
	}

	void Project()
	{
		// Ignore bounds for all operations
		// calc div_u from u0
		// 40 iterations of poisson solving
		// Correct velocity

		RenderToFBONoDepthNoBounds(div_u_fbo, [&]()
			{
				div_u_shader->Begin();
				div_u_shader->Bind_Texture2D("u", u_fbos[0]->tex_index, 0);
				div_u_shader->Set_Uniform("dx", px_dx);

				uv_quad->DrawElements();

				div_u_shader->End();
			});

		RenderToFBONoDepth(p_fbos[0], [&]()
			{
				clear_shader->Begin();
				uv_quad->DrawElements();
				clear_shader->End();
			});

		for (int i = 0; i < 40; ++i)
		{
			int si = i % 2;
			int di = (i + 1) % 2;

			RenderToFBONoDepth(p_fbos[di], [&]()
				{
					poisson_iter_shader->Begin();
					poisson_iter_shader->Bind_Texture2D("p", p_fbos[si]->tex_index, 0);
					poisson_iter_shader->Bind_Texture2D("div_u", div_u_fbo->tex_index, 1);
					poisson_iter_shader->Set_Uniform("dx", px_dx);

					uv_quad->DrawElements();

					poisson_iter_shader->End();
				});
		}

		RenderToFBONoDepth(u_fbos[1], [&]()
			{
				p_correct_u_shader->Begin();
				p_correct_u_shader->Bind_Texture2D("u", u_fbos[0]->tex_index, 0);
				p_correct_u_shader->Bind_Texture2D("p", p_fbos[0]->tex_index, 1);
				p_correct_u_shader->Set_Uniform("dx", px_dx);

				uv_quad->DrawElements();

				p_correct_u_shader->End();
			});

		RenderToFBONoDepth(u_fbos[0], [&]()
			{
				copy_shader->Begin();
				copy_shader->Bind_Texture2D("main_tex", u_fbos[1]->tex_index, 0);

				uv_quad->DrawElements();

				copy_shader->End();
			});
	}

	////synchronize simulation data to visualization data
	void Sync_Simulation_And_Visualization_Data()
	{
		////velocity visualization
		if(draw_velocity){
			for(int i=0;i<fluid.node_num;i++){
				Vector2 pos2=fluid.grid.Node(fluid.grid.Node_Coord(i))+fluid.u[i]*v_rescale;
				(*opengl_vectors->mesh.vertices)[i*2+1]=V3(pos2);}
			opengl_vectors->Set_Data_Refreshed();
		}

		////particle visualization
		if(draw_particles){
			fluid.Update_Visualization_Particles(dt);
			for(int i=0;i<fluid.particles.Size();i++){
				bool outside=false;
				double epsilon=fluid.grid.dx*(double).5;
				for(int j=0;j<2;j++){
					if(fluid.particles.X(i)[j]<fluid.grid.domain_min[j]+epsilon||
						fluid.particles.X(i)[j]>fluid.grid.domain_max[j]-epsilon){outside=true;break;}}
				if(outside){
					opengl_circles[i]->visible=false;
					opengl_circles[i]->Set_Data_Refreshed();
					invis_particles.insert(i);
					fluid.particles.I(i)=-1;
					continue;}

				auto opengl_circle=opengl_circles[i];
				opengl_circle->pos=V3(fluid.particles.X(i));
				opengl_circle->pos[2]=(double).05;
				opengl_circle->Set_Data_Refreshed();}			
		}

		////density visualization
		if(draw_density){
			int cell_num=fluid.grid.cell_counts.prod();
			for(int i=0;i<cell_num;i++){
				int idx=i*6;
				Vector2i cell=fluid.grid.Cell_Coord(i);
				Vector2 pos1=fluid.grid.Node(cell);
				double den1=fluid.Interpolate(fluid.smoke_den,pos1);

				Vector2 pos2=pos1+Vector2::Unit(0)*fluid.grid.dx;
				double den2=fluid.Interpolate(fluid.smoke_den,pos2);

				Vector2 pos3=pos1+Vector2::Unit(0)*fluid.grid.dx+Vector2::Unit(1)*fluid.grid.dx;
				double den3=fluid.Interpolate(fluid.smoke_den,pos3);

				Vector2 pos4=pos1+Vector2::Unit(1)*fluid.grid.dx;
				double den4=fluid.Interpolate(fluid.smoke_den,pos4);
			
				opengl_mesh->colors[idx]=den1;
				opengl_mesh->colors[idx+1]=den2;
				opengl_mesh->colors[idx+2]=den3;
			
				opengl_mesh->colors[idx+3]=den1;
				opengl_mesh->colors[idx+4]=den3;
				opengl_mesh->colors[idx+5]=den4;}
			opengl_mesh->Set_Data_Refreshed();		
		}
	}

	////update simulation and visualization for each time step
	virtual void Toggle_Next_Frame()
	{
		t += dt;

		if (use_shaders)
		{
			ApplySource();
			Advect();
			ConfineVortices();
			Project();
		}
		else
		{
			fluid.Advance(dt);
			Sync_Simulation_And_Visualization_Data();
		}

		OpenGLViewer::Toggle_Next_Frame();
	}

	virtual void Run()
	{
		OpenGLViewer::Run();
	}

	////User interaction
	virtual bool Mouse_Drag(int x,int y,int w,int h)
	{
		if(!add_particle)return false;
		Vector3f win_pos=opengl_window->Project(Vector3f::Zero());
		Vector3f pos=opengl_window->Unproject(Vector3f((float)x,(float)y,win_pos[2]));
		Vector2 p_pos;for(int i=0;i<2;i++)p_pos[i]=(double)pos[i];
		fluid.src_pos=p_pos;
		if (!use_shaders) { Add_Source_Particle(p_pos); }
		return true;
	}

	virtual bool Mouse_Click(int left,int right,int mid,int x,int y,int w,int h)
	{
		if(left!=1&&left!=-1){return false;}
		if(left==-1&&add_particle){
			add_particle=false;
			fluid.src_pos=Vector2::Ones()*-1;	////turn off source
			return true;}

		Vector3f win_pos=opengl_window->Project(Vector3f::Zero());
		Vector3f pos=opengl_window->Unproject(Vector3f((float)x,(float)y,win_pos[2]));
		Vector2 p_pos;for(int i=0;i<2;i++)p_pos[i]=(double)pos[i];
		fluid.src_pos=p_pos;
		if (!use_shaders) { Add_Source_Particle(p_pos); }
		add_particle=true;
		return true;
	}

	////Keyboard interaction
	virtual void Initialize_Common_Callback_Keys()
	{
		OpenGLViewer::Initialize_Common_Callback_Keys();
		Bind_Callback_Key('v',&Keyboard_Event_V_Func,"press v");
		Bind_Callback_Key('2',&Keyboard_Event_D_Func,"press 2");
	}

	virtual void Keyboard_Event_V()
	{
		draw_velocity=!draw_velocity;
		if (use_shaders)
		{
			smoke_mesh->main_tex = draw_velocity ? u_fbos[0]->tex_index : den_fbos[0]->tex_index;
			return;
		}
		opengl_vectors->visible = draw_velocity;;
		opengl_vectors->Set_Data_Refreshed();
	}
	Define_Function_Object(GridFluidDriver,Keyboard_Event_V);

	virtual void Keyboard_Event_D()
	{
		draw_density=!draw_density;
		if (use_shaders)
		{
			return;
		}
		opengl_mesh->visible = draw_density;
		opengl_mesh->Set_Data_Refreshed();
	}
	Define_Function_Object(GridFluidDriver,Keyboard_Event_D);

	//////////////////////////////////////////////////////////////////////////
	////User interaction for manipulating sources
	void Add_Source_Particle(Vector2 p_pos)
	{
		double rx=.1*static_cast<float>(rand()%1000)/1000.-.05;
		double ry=.1*static_cast<float>(rand()%1000)/1000.-.05;
		p_pos[0]+=rx;p_pos[1]+=ry;
		if(!invis_particles.empty()){
			int p=(*invis_particles.begin());
			fluid.particles.X(p)=p_pos;
			fluid.particles.I(p)=0;
			opengl_circles[p]->pos=V3(fluid.particles.X(p));
			opengl_circles[p]->Update_Model_Matrix();
			opengl_circles[p]->Set_Data_Refreshed();
			opengl_circles[p]->visible=true;
			invis_particles.erase(p);
		}
		else{
			Add_Particle(p_pos);
			Add_Solid_Circle(fluid.particles.Size()-1);	
		}
	}

	void Add_Particle(Vector2 pos)
	{
		int i=fluid.particles.Add_Element();	////return the last element's index
		fluid.particles.X(i)=pos;
		fluid.particles.C(i)=(double)(rand()%2000-1000)/(double)1000;	////particle vorticity, a random number between [-1,1]
	}

	void Add_Solid_Circle(const int i)
	{
		OpenGLColor c(1.f,0.68f,0.26f,1.f);
		auto opengl_circle=Add_Interactive_Object<OpenGLSolidCircle>();
		opengl_circle->visible=false;
		opengl_circles.push_back(opengl_circle);
		opengl_circle->pos=V3(fluid.particles.X(i));
		opengl_circle->radius=(double).01;
		opengl_circle->color=c;
		opengl_circle->Initialize();
		opengl_circle->Update_Model_Matrix();
		opengl_circle->Set_Data_Refreshed();
		opengl_circle->visible=true;
	}

protected:
	////Helper function to convert a vector to 3d, for c++ template
	Vector3 V3(const Vector2& v2){return Vector3(v2[0],v2[1],.0);}
	Vector3 V3(const Vector3& v3){return v3;}
};
#endif