//////////////////////////////////////////////////////////////////////////
//// Dartmouth Physical Computing Programming Assignment 2: DEM Particle Sand
//// Author: TODO: PUT YOUR NAME HERE
////////////////////////////////////////////////////////////////////////// 

#ifndef __ParticleSand_h__
#define __ParticleSand_h__
#include "Common.h"
#include "Particles.h"
#include "ImplicitGeometry.h"

template<int d> class ParticleSand
{using VectorD=Vector<double,d>;using VectorDi=Vector<int,d>;using MatrixD=Matrix<double,d>;
public:
	Particles<d> particles;
	double ks=(double)2e2;		////stiffness for the collision force
	double kd=(double).5e1;		////damping for the collision force
	VectorD g=VectorD::Unit(1)*(double)-1.;	////gravity

	////a list of implicit geometries describing the environment, by default it has one element, a circle with its normals pointing inward (Bowl)
	std::vector<ImplicitGeometry<d>* > env_objects;	
	std::vector<Vector2i> particle_particle_collision_pairs;
	std::vector<Vector2i> particle_environment_collision_pairs;

	std::vector<VectorD> my_object_vertices={{-1.,5.},{1.,5.}};	////this array stores the positions of the contour of your object for visualization

	virtual void Advance(const double dt)
	{
		////Clear forces on particles
		for(int i=0;i<particles.Size();i++){
			particles.F(i)=VectorD::Zero();}

		////Accumulate body forces
		for(int i=0;i<particles.Size();i++){
			particles.F(i)+=particles.M(i)*g;}

		Particle_Environment_Collision_Detection();
		Particle_Environment_Collision_Response();
		Particle_Particle_Collision_Detection();
		Particle_Particle_Collision_Response();
		Particle_My_Object_Collision_Detection_And_Response();

		for(int i=0;i<particles.Size();i++){
			particles.V(i)+=particles.F(i)*(dt/particles.M(i));
			particles.X(i)+=particles.V(i)*dt;}
	}

	//////////////////////////////////////////////////////////////////////////
	////YOUR IMPLEMENTATION (P1 TASK): detect collision between particles and env_objects (implicit surface) and record the detected collisions in particle_environment_collision_pairs
	////env_objects is a list of implicit geometries, by default there is only one geometry (the bowl) in the list
	////Each element in particle_environment_collision_pairs is a Vector2i, with the first index for the particle and the second index for the env_objects
	virtual void Particle_Environment_Collision_Detection()
	{
		particle_environment_collision_pairs.clear();
		/* Your implementation start */
		const int n_parts = particles.Size();
		const int n_env = static_cast<int>(env_objects.size());

		for (int i = 0; i < n_parts; ++i)
		{
			for (int j = 0; j < n_env; ++j)
			{
				const double phi = env_objects[j]->Phi(particles.X(i));

				if ((phi - particles.R(i)) < 0)
				{
					particle_environment_collision_pairs.emplace_back(i, j);
				}
			}
		}
		/* Your implementation end */
	}
		
	//////////////////////////////////////////////////////////////////////////
	////YOUR IMPLEMENTATION (P1 TASK): compute the penalty-based collision force for the particles that are colliding with the env_objects
	////The collision response force consists of a spring force and a damping force
	virtual void Particle_Environment_Collision_Response()
	{
		for(int pair_idx=0;pair_idx<particle_environment_collision_pairs.size();pair_idx++){
			int i=particle_environment_collision_pairs[pair_idx][0];	////particle index
			int j=particle_environment_collision_pairs[pair_idx][1];	////env_objects index
			VectorD collision_force=VectorD::Zero();

			/* Your implementation start */
			const auto* const env_obj = env_objects[j];
			VectorD xi = particles.X(i);
			VectorD norm = env_obj->Normal(xi);

			collision_force += (ks * (env_obj->Phi(xi) - particles.R(i)) + kd * particles.V(i).dot(norm)) * -norm;

			/* Your implementation end */
			
			particles.F(i)+=collision_force;
		}
	}

	//////////////////////////////////////////////////////////////////////////
	////YOUR IMPLEMENTATION (P1 TASK): find all the pairs of particles that are colliding each other and record the detected pairs in particle_particle_collision_pairs
	////Each element in particle_particle_collision_pairs is a Vector2i specifying the indices of the two colliding particles
	virtual void Particle_Particle_Collision_Detection()
	{
		particle_particle_collision_pairs.clear();
		/* Your implementation start */
		const int n_parts = particles.Size();

		for (int i = 0; i < n_parts; ++i)
		{
			const VectorD ci = particles.X(i);
			const double ri = particles.R(i);

			for (int j = i + 1; j < n_parts; ++j)
			{
				const double dx = (ci - particles.X(j)).norm();
				const double min_d = ri + particles.R(j);

				if (dx < min_d)
				{
					particle_particle_collision_pairs.emplace_back(i, j);
				}
			}
		}
		/* Your implementation end */
	}

	//////////////////////////////////////////////////////////////////////////
	////YOUR IMPLEMENTATION (P1 TASK): compute penalty-based collision forces for pairs of colliding particles in particle_particle_collision_pairs and add the forces to particle.F 
	////The collision response force for each pair consists of a spring force and a damping force
	virtual void Particle_Particle_Collision_Response()
	{
		for(int pair_idx=0;pair_idx<particle_particle_collision_pairs.size();pair_idx++){
			int i=particle_particle_collision_pairs[pair_idx][0];	////the first particle index in the pair
			int j=particle_particle_collision_pairs[pair_idx][1];	////the second particle index in the pair

			/* Your implementation start */
			const VectorD dx = particles.X(j) - particles.X(i);
			const double dxn = dx.norm();
			const double min_x = particles.R(i) + particles.R(j);
			const VectorD norm = dx / dxn;

			const VectorD collision_force = (ks * (dxn - min_x) + kd * (particles.V(j) - particles.V(i)).dot(norm)) * norm;

			particles.F(i) += collision_force;
			particles.F(j) -= collision_force;

			/* Your implementation end */
		}

	}

	//////////////////////////////////////////////////////////////////////////
	////YOUR IMPLEMENTATION (P1 TASK): implement your the collision detection algorithm and calculate the response forces for your own object
	////YOUR IMPLEMENTATION (P1 TASK): visualize your object by filling up the contour vertex array
	////Note: the visualization does not have to be the same as the way you represent the geometry, 
	////E.g., you may use an implicit function to calculate the geometry and the collision force, but still use the vertices to draw it
	virtual void Particle_My_Object_Collision_Detection_And_Response()
	{
		////if you want to visualize your object, fill the array of my_object_shape with its contour vertices, the commented code provides an example
		//my_object_vertices.clear();
		//my_object_vertices.push_back(Vector2(-2.,5.));
		//my_object_vertices.push_back(Vector2(2.,5.));

		for(int i=0;i<particles.Size();i++){
			VectorD collision_force=VectorD::Zero();
			
			/* Your implementation start */
			/* Your implementation end */
			
			particles.F(i)+=collision_force;
		}	
	}
};

#endif
