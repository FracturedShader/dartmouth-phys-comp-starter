//////////////////////////////////////////////////////////////////////////
//// Dartmouth Physical Computing Programming Assignment 3: Grid-based Fluid Simulation (Smoke)
//// Author: TODO: PUT YOUR NAME HERE
////////////////////////////////////////////////////////////////////////// 
#ifndef __GridFluid_h__
#define __GridFluid_h__
#include "Common.h"
#include "Grid.h"
#include "OpenGLBufferObjects.h"
#include "Particles.h"
  
//////////////////////////////////////////////////////////////////////////
////Grid fluid simulator 2D
class GridFluid
{
public:
	//// grid and fields
	Grid<2> grid;											//// Euler grid 
	std::vector<Vector2> u;									//// velocity on grid nodes
	std::vector<double> div_u;								//// velocity divergence on grid nodes (right hand side of the Poisson equation)
	std::vector<double> p;									//// pressure
	std::vector<double> vor;								//// vorticity
	std::vector<double> smoke_den;							//// smoke density

	//// source-related variables
	Vector2 src_pos=Vector2::Ones()*(double).5;				//// source position
	Vector2 src_vel=Vector2::Unit(0)*(double)1.5;			//// source velocity
	double src_radius=(double).025;							//// source radius

	//// grid-related variables
	int node_num=0;											//// total number of grid nodes

	//////////////////////////////////////////////////////////////////////////
	//// Helper functions for grid access
	//// Read these functions carefully before your implementation
	//////////////////////////////////////////////////////////////////////////
	//// return the grid node index given its coordinate
	int Idx(const Vector2i& node_coord) const 
	{return grid.Node_Index(node_coord);}
	
	//// return the grid node coordinate given its index
	Vector2i Coord(const int node_index) const
	{return grid.Node_Coord(node_index);}

	//// return the grid node position given its index
	Vector2 Pos(const int node_index) const
	{return grid.Node(node_index);}

	//// return the grid node position given its coordinate
	Vector2 Pos(const Vector2i& node_coord) const
	{return grid.Node(node_coord);}

	//// cross product in 2D: w is a vector orthogonal to the plane
	Vector2 Cross(const Vector2& v,const double w) const
	{return Vector2(v[1]*w,-v[0]*w);}

	//// check if a grid node is on the grid boundary
	bool Bnd(const Vector2i& node_coord) const
	{
		for(int i=0;i<2;i++){
			if(node_coord[i]==0||node_coord[i]==grid.node_counts[i]-1)
				return true;
		}
		return false;	
	}

	bool Bnd(const int node_index) const
	{return Bnd(Coord(node_index));}

	////clamp pos to ensure it is always inside the grid
	Vector2 Clamp_Pos(const Vector2& pos)
	{
		double epsilon=grid.dx*(double)1e-3;
		Vector2 clamped_pos=pos;
		for(int i=0;i<2;i++){
			if(pos[i]<=grid.domain_min[i])clamped_pos[i]=grid.domain_min[i]+epsilon;
			else if(pos[i]>=grid.domain_max[i])clamped_pos[i]=grid.domain_max[i]-epsilon;}	
		return clamped_pos;
	}

	//// calculate the index, fraction, and interpolated values from the array
	void Calculate_Cell_Index_And_Position(const Vector2& pos,/*result*/Vector2i& cell,/*result*/Vector2& frac)
	{
		Vector2 cell_with_frac=(pos-grid.domain_min)/grid.dx;
		cell=cell_with_frac.template cast<int>();
		frac=cell_with_frac-cell.template cast<double>();		
	}

	//// Interpolation for scalars with a clamped position. This function calls the bi-linear interpolation function you will implement.
	double Interpolate(const std::vector<double>& u,const Vector2& pos)
	{
		Vector2i cell;
		Vector2 frac;
		Vector2 clamped_pos=Clamp_Pos(pos);
		Calculate_Cell_Index_And_Position(clamped_pos,cell,frac);
		return Bilinear_Interpolation(cell,frac,u);
	}

	//// Interpolation for vectors with a clamped position. This function calls the bi-linear interpolation function you will implement.
	Vector2 Interpolate(const std::vector<Vector2>& u,const Vector2& pos)
	{
		Vector2i cell;
		Vector2 frac;
		Vector2 clamped_pos=Clamp_Pos(pos);
		Calculate_Cell_Index_And_Position(clamped_pos,cell,frac);
		return Bilinear_Interpolation(cell,frac,u);
	}

	//////////////////////////////////////////////////////////////////////////
	//// TASK 1: bi-linear interpolation
	//// 2D bi-linear interpolation for scalars
	double Bilinear_Interpolation(const Vector2i& cell,const Vector2& frac,const std::vector<double>& u)
	{
		/*Your implementation ends*/
		const double u00 = u[Idx(cell)];
		const double u10 = u[Idx(cell + Vector2i::UnitX())];
		const double u01 = u[Idx(cell + Vector2i::UnitY())];
		const double u11 = u[Idx(cell + Vector2i::Ones())];

		const double ux0 = u00 * (1 - frac[0]) + u10 * frac[0];
		const double ux1 = u01 * (1 - frac[0]) + u11 * frac[0];

		return ux0 * (1 - frac[1]) + ux1 * frac[1];		//// replace this line with your implementation
		/*Your implementation ends*/
	}

	//// 2D bi-linear interpolation for vectors
	Vector2 Bilinear_Interpolation(const Vector2i& cell,const Vector2& frac,const std::vector<Vector2>& u)
	{
		/*Your implementation starts*/
		const Vector2& u00 = u[Idx(cell)];
		const Vector2& u10 = u[Idx(cell + Vector2i::UnitX())];
		const Vector2& u01 = u[Idx(cell + Vector2i::UnitY())];
		const Vector2& u11 = u[Idx(cell + Vector2i::Ones())];

		const Vector2 ux0 = u00 * (1 - frac[0]) + u10 * frac[0];
		const Vector2 ux1 = u01 * (1 - frac[0]) + u11 * frac[0];

		return ux0 * (1 - frac[1]) + ux1 * frac[1];		//// replace this line with your implementation
		/*Your implementation ends*/
	}

	//// initialize the grid and fields
	virtual void Initialize()
	{
		int n=128;								//// Attention: change this variable to change the grid resolution!
		Vector2i cell_counts=Vector2i(n,n/2);
		double dx=(double)2./(double)n;
		Vector2 domain_min=Vector2::Zero();
		grid.Initialize(cell_counts,dx,domain_min);
		node_num=grid.node_counts.prod();

		u.resize(node_num,Vector2::Unit(0)*(double).01);
		div_u.resize(node_num,(double)0);
		p.resize(node_num,(double)0);
		vor.resize(node_num,(double)0);
		smoke_den.resize(node_num,(double)0);
	}

	//// timestep update
	void Advance(const double dt)
	{
		Source();
		Advection(dt);
		Vorticity_Confinement(dt);
		//Update_Vortex_Particles(dt);				//// Attention: uncomment this line and comment the above line for Part II vortex particle method!
		Projection();
	}

	//////////////////////////////////////////////////////////////////////////
	//// TASK 2: set source velocity and smoke density.
	//// Go over all the grid nodes. For each node, check if it is within the radius of the source position. 
	//// If it is, set the node velocity to be the source velocity and the node smoke density to be the source density.
	//// Hint: You need to use the source-related variables declared at the beginning of the class
	void Source()
	{
		const double src_rad_sq = src_radius * src_radius;

		for(int i=0;i<node_num;i++){
			/*Your implementation starts*/
			if(Bnd(i))continue;		////ignore the nodes on the boundary

			if ((Pos(i) - src_pos).squaredNorm() < src_rad_sq)
			{
				u[i] = src_vel;
				smoke_den[i] = 1;
			}
			/*Your implementation ends*/
		}
	}

	//////////////////////////////////////////////////////////////////////////
	//// TASK 3: Advection: advect BOTH velocity and density on the grid using the semi-Lagrangian method
	//// Hint: You need to use the interpolation function you implemented in Task 1
	virtual void Advection(double dt)
	{
		//// Attention: We made a copy for both u and smoke_den before advection. Why?
		std::vector<Vector2> u_copy=u;
		std::vector<double> den_copy=smoke_den;

		for(int i=0;i<node_num;i++){
			u[i]=Vector2::Zero();
			smoke_den[i]=0.;

			/*Your implementation starts*/
			const Vector2 node_coord = grid.Node(i);
			const Vector2 u_half = Interpolate(u_copy, node_coord - u_copy[i] * (dt / 2));
			const Vector2 source_pos = node_coord - u_half * dt;

			u[i] = Interpolate(u_copy, source_pos);
			smoke_den[i] = Interpolate(den_copy, source_pos);
			/*Your implementation ends*/
		}
	}

	//////////////////////////////////////////////////////////////////////////
	//// TASK 4: project the velocity to a divergence-free field
	virtual void Projection()
	{
		double dx=grid.dx;
		double dx2=grid.dx*grid.dx;

		//// (Sample) Projection step 1: calculate the velocity divergence on each node
		//// Read this sample code to learn how to access data with the node index and coordinate
		double denom = 2 * dx;
		std::fill(div_u.begin(),div_u.end(),(double)0);
		for(int i=0;i<node_num;i++){
			if(Bnd(i))continue;		////ignore the nodes on the boundary
			Vector2i node=Coord(i);
			div_u[i]=(double)0;

			for(int j=0;j<2;j++){
				const Vector2 u_1=u[Idx(node-Vector2i::Unit(j))];
				const Vector2 u_2=u[Idx(node+Vector2i::Unit(j))];
				div_u[i]+=(u_2[j]-u_1[j])/denom;}
		}

		//// TASK 4.1: Projection step 2: solve the Poisson's equation -lap p= div u using the Gauss-Seidel iterations
		denom = 4 / dx2;
		std::fill(p.begin(),p.end(),(double)0);
		for(int iter=0;iter<40;iter++){
			for(int i=0;i<node_num;i++){
				if(Bnd(i))continue;		////ignore the nodes on the boundary
				Vector2i node=Coord(i);

				/*Your implementation starts*/
				double p_sum = 0;
				for(int j=0;j<2;j++){
					const double p_1=p[Idx(node-Vector2i::Unit(j))];
					const double p_2=p[Idx(node+Vector2i::Unit(j))];
					p_sum += p_1 + p_2;}
				//p[i] = (-div_u[i] + p_sum / dx2) / denom;
				p[i] = (-div_u[i] * dx2 + p_sum) / 4;
				/*Your implementation ends*/
			}
		}
		
		//// TASK 4.2: Projection step 3: correct velocity with the pressure gradient
		denom = 2 * dx;
		for(int i=0;i<node_num;i++){
			if(Bnd(i))continue;		////ignore boundary nodes
			Vector2i node=Coord(i);
			Vector2 grad_p=Vector2::Zero();

			/*Your implementation starts*/
			for(int j=0;j<2;j++){
				const double p_1=p[Idx(node-Vector2i::Unit(j))];
				const double p_2=p[Idx(node+Vector2i::Unit(j))];
				grad_p[j]+=(p_2-p_1)/denom;}

			u[i] -= grad_p;
			/*Your implementation ends*/
		}
	}

	//////////////////////////////////////////////////////////////////////////
	//// TASK 5: vorticity confinement
	void Vorticity_Confinement(const double dt)
	{
		double dx=grid.dx;

		////TASK 5.1: update vorticity
		const double denom = 2 * dx;
		std::fill(vor.begin(),vor.end(),(double)0);
		for(int i=0;i<node_num;i++){
			if(Bnd(i))continue;		////ignore boundary nodes
			Vector2i node=Coord(i);
			vor[i]=(double)0;

			/*Your implementation starts*/
			for(int j=0;j<2;j++){
				const Vector2 u_1=u[Idx(node-Vector2i::Unit(j))];
				const Vector2 u_2=u[Idx(node+Vector2i::Unit(j))];
				const int jn = (j + 1) % 2;
				vor[i]+=(u_2[jn]-u_1[jn])/denom;}
			/*Your implementation ends*/
		}

		////TASK 5.2: update N = (grad(|vor|)) / |grad(|vor|)|
		std::vector<Vector2> N(node_num,Vector2::Zero());
		for(int i=0;i<node_num;i++){
			if(Bnd(i))continue;		////ignore boundary nodes
			Vector2i node=Coord(i);
			N[i]=Vector2::Zero();

			/*Your implementation starts*/
			Vector2 grad_vor = Vector2::Zero();

			for(int j=0;j<2;j++){
				const double mv_1=vor[Idx(node-Vector2i::Unit(j))];
				const double mv_2=vor[Idx(node+Vector2i::Unit(j))];
				grad_vor[j]+=(mv_2-mv_1)/denom;}

			N[i] = grad_vor.normalized();
			/*Your implementation ends*/
		}

		//// calculate confinement force and use it to update velocity
		double vor_conf_coef=(double)4;
		for(int i=0;i<node_num;i++){
			if(Bnd(i))continue;		////ignore boundary nodes
			Vector2 f=vor_conf_coef*dx*Cross(N[i],vor[i]);
			u[i]+=f*dt;		////we don't have mass by assuming density=1
		}
	}

	//////////////////////////////////////////////////////////////////////////
	//// vortex particles
	//// This is the Gaussian kernel function you might need to use in calculating your confinement force
	double Kernel(const double length)
	{
		double r=grid.dx*(double)4;
		if(length>r)return (double)0;
		double coef1=(double)1/(double)2*(r*r);
		double coef2=(double)1/(r*r);
		return (double).1*coef2*exp(-length*length*coef1);
	}

	//////////////////////////////////////////////////////////////////////////
	//// TASK 6 (optional): vorticity confinement with vortex particles
	
	void Update_Vortex_Particles(const double dt)
	{
		//// Now we use vortex particles to preserve the vorticity in the domain
		//// particles.C represents the vorticity carried on each particle
		//// particles.I denotes whether the particle is valid for calculating vorticity confinement: skip the particle if(particles.I(i)==-1) 
		for(int i=0;i<node_num;i++){
			if(Bnd(i))continue;		////ignore boundary nodes
			Vector2i node=Coord(i);
			Vector2 node_pos=grid.Node(node);
			vor[i]=(double)0;

			//// Calculate the confinement force for each grid node due to each particle
			//// Hint: recall your implementation in the vorticity confinement function
			/*Your implementation starts*/

			/*Your implementation ends*/
		}		
	}
	//////////////////////////////////////////////////////////////////////////

	////Particles, for visualization only
public:
	Particles<2> particles;

	void Initialize_Visualization_Particles()
	{
		particles.Resize(1);
		particles.X(0)=Vector2::Zero();
	}

	void Update_Visualization_Particles(const double dt)
	{
		for(int i=0;i<particles.Size();i++){
			Vector2 v=Interpolate(u,particles.X(i));
			particles.X(i)+=v*dt;
		}
	}
};

#endif
