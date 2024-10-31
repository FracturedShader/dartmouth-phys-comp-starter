//////////////////////////////////////////////////////////////////////////
//// Dartmouth Physical Computing Programming Assignment 1: Mass Spring
//// Author: TODO: PUT YOUR NAME HERE
////////////////////////////////////////////////////////////////////////// 

#ifndef __SoftBodyMassSpring_h__
#define __SoftBodyMassSpring_h__
#include "Common.h"
#include "Particles.h"

class SoftBodyMassSpring
{
public:
	////Spring parameters
	Particles<3> particles;								//// The particle system. Each particle has the attributes of position, velocity, mass, and force. Read Particles.h in src to know the details.
	std::vector<Vector2i> springs;						//// Each element in springs stores a Vector2i for the indices of the two end points.
	std::vector<double> rest_length;					//// Each element in rest_length specifies the rest length of the spring
	std::vector<double> ks;								//// Each element in ks specifies the spring stiffness of a spring
	std::vector<double> kd;								//// Each element in kd specifies the damping coefficient of a spring

	////Boundary nodes
	std::unordered_map<int,Vector3> boundary_nodes;		//// boundary_notes stores the mapping from node index to its specified velocity. E.g., a fixed node will have a zero velocity.

	////Body force
	Vector3 g=Vector3::Unit(1)*(double)-1.;			//// gravity
	
	enum class TimeIntegration{ExplicitEuler,ImplicitEuler} time_integration=TimeIntegration::ExplicitEuler;	//// set to ExplicitEuler by default; change it to ImplicitEuler when you work on Task 2 (Option 2)

	////Implicit time integration
	SparseMatrixT K;
	VectorX u,b;

	virtual void Initialize()
	{
		////Initialize default spring parameters for standard tests
		double ks_0=(double)1,kd_0=(double)1;
		switch(time_integration){
		case TimeIntegration::ExplicitEuler:{
			ks_0=(double)5e2;
			kd_0=(double)1e1;
		}break;
		case TimeIntegration::ImplicitEuler:{
			ks_0=(double)1e5;
			kd_0=(double)1e1;			
		}break;}

		////Allocate arrays for springs and parameters
		rest_length.resize(springs.size());
		for(int i=0;i<(int)springs.size();i++){const Vector2i& s=springs[i];
			rest_length[i]=(particles.X(s[0])-particles.X(s[1])).norm();}
		ks.resize(springs.size(),ks_0);
		kd.resize(springs.size(),kd_0);

		////Allocate sparse matrix if using implicit time integration 
		////This function needs to be called for only once since the mesh doesn't change during the simulation)
		if(time_integration==TimeIntegration::ImplicitEuler)
			Initialize_Implicit_K_And_b();
	}

	virtual void Advance(const double dt)
	{
		switch(time_integration){
		case TimeIntegration::ExplicitEuler:
			Advance_Explicit_Euler(dt);break;
		case TimeIntegration::ImplicitEuler:
			Advance_Implicit_Euler(dt);break;}
	}
	
	////Set boundary nodes
	void Set_Boundary_Node(const int p,const Vector3 v=Vector3::Zero()){boundary_nodes[p]=v;}
	
	bool Is_Boundary_Node(const int p){return boundary_nodes.find(p)!=boundary_nodes.end();}

	//////////////////////////////////////////////////////////////////////////
	//// P1 TASK: explicit Euler integration and spring force calculation

	//////////////////////////////////////////////////////////////////////////
	//// YOUR IMPLEMENTATION (TASK 1): explicit Euler time integration 
	void Advance_Explicit_Euler(const double dt)
	{
		//// Step 0: Clear the force on each particle (already done for you)
		Clear_Force();

		//// Step 1: add a body force to each particle
		Apply_Body_Force(dt);

		//// Step 2: calculate the spring force for each spring and add it to the two connecting particles (Hint: you may want to implement the function Spring_Force and call it in your for-loop)
		Apply_Spring_Force(dt);

		//// Step 3: enforce the boundary conditions (traversing the particles in the unordered_set, set its velocity to be the value read from the unordered_set, and its force to be zero)
		Enforce_Boundary_Condition();

		//// Step 4: time integration by updating the particle velocities according to their forces and updating particle positions according to the positions
		Time_Integration(dt);
	}

	void Clear_Force()
	{
		for(int i=0;i<particles.Size();i++){particles.F(i)=Vector3::Zero();}
	}

	void Apply_Body_Force(const double dt)
	{
		/* Your implementation start */

		const int n_parts = particles.Size();

		for (int i = 0; i < n_parts; ++i)
		{
			particles.F(i) += g * particles.M(i);
		}

		/* Your implementation end */	
	}

	void Apply_Spring_Force(const double dt)
	{
		/* Your implementation start */

		const int n_springs = static_cast<int>(springs.size());

		for (int i = 0; i < n_springs; ++i)
		{
			const auto& is = springs[i];
			const Vector3 sf = Spring_Force(i);

			particles.F(is[0]) += sf;
			particles.F(is[1]) -= sf;
		}

		/* Your implementation end */	
	}

	void Enforce_Boundary_Condition()
	{
		/* Your implementation start */

		for (const auto& kvp : boundary_nodes)
		{
			particles.F(kvp.first) = kvp.second;
		}

		/* Your implementation end */	
	}

	void Time_Integration(const double dt)
	{
		/* Your implementation start */

		const int n_parts = particles.Size();

		for (int i = 0; i < n_parts; ++i)
		{
			particles.V(i) += particles.F(i) * (dt / particles.M(i));
			particles.X(i) += particles.V(i) * dt;
		}

		/* Your implementation end */		
	}
	
	Vector3 Spring_Force(const int spring_index)
	{
		//// This is an auxiliary function to compute the spring force f=f_s+f_d for the spring with spring_index. 
		//// You may want to call this function in Apply_Spring_Force
		
		/* Your implementation start */

		const auto& is = springs[spring_index];

		const Vector3 dx = particles.X(is[1]) - particles.X(is[0]);
		const double dxn = dx.norm();
		const Vector3 norm = dx / dxn;
		
		const double fs = ks[spring_index] * (dxn - rest_length[spring_index]);

		const Vector3 dv = particles.V(is[1]) - particles.V(is[0]);

		const double fd = kd[spring_index] * dv.dot(norm);

		/* Your implementation end */

		return norm * (fs + fd);
	}

	//////////////////////////////////////////////////////////////////////////
	//// TASK 2 (OPTION 1): creating bending springs for hair strand simulation
	void Initialize_Hair_Strand()
	{
		//// You need to initialize a hair model by setting the springs and particles to simulate human hair.
		//// A key component for a hair simulator is the bending spring (e.g., by connecting particles with a certain index offset).
		//// Think about how to realize these bending and curly effects with the explicit spring model you have implemented in TASK 1.
		//// You may also want to take a look at the function Initialize_Simulation_Data() in MassSpringInteractiveDriver.h for the model initialization.
		
		/* Your implementation start */

		/* Your implementation end */
	}

	//////////////////////////////////////////////////////////////////////////
	//// TASK 2 (OPTION 2): implicit time integration for inextensible cloth simulation
	//// The rest part of this file is all for this task.
	
	////Construct K, step 1: initialize the matrix structure 
	void Initialize_Implicit_K_And_b()
	{
		int n=3*particles.Size();
		K.resize(n,n);u.resize(n);u.fill((double)0);b.resize(n);b.fill((double)0);
		std::vector<TripletT> elements;
		for(int s=0;s<(int)springs.size();s++){int i=springs[s][0];int j=springs[s][1];
			Add_Block_Triplet_Helper(i,i,elements);
			Add_Block_Triplet_Helper(i,j,elements);
			Add_Block_Triplet_Helper(j,i,elements);
			Add_Block_Triplet_Helper(j,j,elements);}
		K.setFromTriplets(elements.begin(),elements.end());
		K.makeCompressed();	
	}
	
	////Construct K, step 2: fill nonzero elements in K
	void Update_Implicit_K_And_b(const double dt)
	{
		////Clear K and b
		K.setZero();
		b.fill((double)0);

		/* Your implementation start */

		/* Your implementation end */
	}

	////Construct K, step 2.1: compute spring force derivative
	void Compute_Ks_Block(const int s,Matrix3& Ks)
	{
		/* Your implementation start */

		/* Your implementation end */
	}

	////Construct K, step 2.2: compute damping force derivative
	void Compute_Kd_Block(const int s,Matrix3& Kd)
	{
		/* Your implementation start */

		/* Your implementation end */
	}

	////Implicit Euler time integration
	void Advance_Implicit_Euler(const double dt)
	{
		//// clear force
		Clear_Force();
		//// add a body force to each particle as in explicit Euler
		Apply_Body_Force(dt);
		//// calculate the spring force as in explicit Euler
		Apply_Spring_Force(dt);
		//// enforce boundary condition
		Enforce_Boundary_Condition();

		Update_Implicit_K_And_b(dt);

		for(int i=0;i<particles.Size();i++){
			for(int j=0;j<3;j++)u[i*3+j]=particles.V(i)[j];}	////set initial guess to be the velocity from the last time step

		SparseSolver::CG(K,u,b);	////solve Ku=b using Conjugate Gradient

		for(int i=0;i<particles.Size();i++){
			Vector3 v;for(int j=0;j<3;j++)v[j]=u[i*3+j];
			particles.V(i)=v;
			particles.X(i)+=particles.V(i)*dt;}
	}

	////Hint: you may want to use these functions when assembling your implicit matrix
	////Add block nonzeros to sparse matrix elements (for initialization)
	void Add_Block_Triplet_Helper(const int i,const int j,std::vector<TripletT>& elements)
	{for(int ii=0;ii<3;ii++)for(int jj=0;jj<3;jj++)elements.push_back(TripletT(i*3+ii,j*3+jj,(double)0));}

	////Add block Ks to K_ij
	void Add_Block_Helper(SparseMatrixT& K,const int i,const int j,const Matrix3& Ks)
	{
		SparseFunc::Add_Block<3,Matrix3>(K,i,i,Ks);
		SparseFunc::Add_Block<3,Matrix3>(K,j,j,Ks);
		if(!Is_Boundary_Node(i)&&!Is_Boundary_Node(j)){
			SparseFunc::Add_Block<3,Matrix3>(K,i,j,-Ks);
			SparseFunc::Add_Block<3,Matrix3>(K,j,i,-Ks);}
	}

	////Set block values on a vector
	void Set_Block(VectorX& b,const int i,const Vector3& bi)
	{for(int ii=0;ii<3;ii++)b[i*3+ii]=bi[ii];}

	////Add block values to a vector
	void Add_Block(VectorX& b,const int i,const Vector3& bi)
	{for(int ii=0;ii<3;ii++)b[i*3+ii]+=bi[ii];}
};

#endif