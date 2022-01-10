//////////////////////////////////////////////////////////////////////////
////Dartmouth Physical Computing Starter Code
////http://www.dartmouth.edu/~boolzhu/cosc89.18.html
//////////////////////////////////////////////////////////////////////////

#ifndef __Common_h__
#define __Common_h__
#include <vector>
#include <list>
#include <queue>
#include <array>
#include <unordered_map>
#include <unordered_set>
#include <memory>
#include <iostream>
#include "Eigen/Dense"
#include "Eigen/Sparse"

//////////////////////////////////////////////////////////////////////////
//// Eigen vector and matrix types

//// To define a vector or matrix type, you may use the following aliased type names:
//// Vector<double,2>, Vector<float,3>, Vector<int,4>, Matrix<float,3>, etc.
//// or Vector2d, Vector3f, Vector4i, Matrix3f, etc.

template<class T,int d> using Vector=Eigen::Matrix<T,d,1>;
template<class T,int d> using Matrix=Eigen::Matrix<T,d,d>;

#define Declare_Eigen_Vector_Types(type,t)		\
using Vector1##t=Eigen::Matrix<type,1,1>;       \
using Vector2##t=Eigen::Vector2##t;             \
using Vector3##t=Eigen::Vector3##t;             \
using Vector4##t=Eigen::Vector4##t;             \
using VectorX##t=Eigen::VectorX##t;

#define Declare_Eigen_Matrix_Types(type,t)		\
using Matrix1##t=Eigen::Matrix<type,1,1>;       \
using Matrix2##t=Eigen::Matrix2##t;             \
using Matrix3##t=Eigen::Matrix3##t;             \
using Matrix4##t=Eigen::Matrix4##t;             \
using MatrixX##t=Eigen::MatrixX##t;  

Declare_Eigen_Vector_Types(int,i)
Declare_Eigen_Vector_Types(float,f)
Declare_Eigen_Vector_Types(double,d)
Declare_Eigen_Matrix_Types(int,i)
Declare_Eigen_Matrix_Types(float,f)
Declare_Eigen_Matrix_Types(double,d)

#define Declare_Eigen_Types(type,t)     \
using Vector1=Eigen::Matrix<double,1,1>;	\
using Vector2=Eigen::Vector2##t;        \
using Vector3=Eigen::Vector3##t;        \
using Vector4=Eigen::Vector4##t;        \
using VectorX=Eigen::VectorX##t;        \
using Matrix2=Eigen::Matrix2##t;        \
using Matrix3=Eigen::Matrix3##t;        \
using Matrix4=Eigen::Matrix4##t;        \
using MatrixX=Eigen::MatrixX##t;        \

#ifdef USE_FLOAT
Declare_Eigen_Types(float,f)
#else
Declare_Eigen_Types(double,d)
#endif

//////////////////////////////////////////////////////////////////////////
////Eigen sparse types

template<class T> using VectorN=Eigen::Matrix<T,-1,1>;
template<class T,int d> using Matrix=Eigen::Matrix<T,d,d>;
template<class T> using SparseMatrix=Eigen::SparseMatrix<T,Eigen::RowMajor,int>;
using SparseMatrixT=SparseMatrix<double>;
template<class T> using InnerIterator=typename SparseMatrix<T>::InnerIterator;
using InnerIteratorT=SparseMatrixT::InnerIterator;
template<class T> using DiagonalMatrix=Eigen::DiagonalMatrix<T,Eigen::Dynamic,Eigen::Dynamic>;
using DiagonalMatrixT=Eigen::DiagonalMatrix<double,Eigen::Dynamic,Eigen::Dynamic>;
template<class T> using Triplet=Eigen::Triplet<T,int>;
using TripletT=Triplet<double>;
template<class T> using IncompleteCholesky=Eigen::IncompleteCholesky<T>;

////Eigen sparse matrix helper functions
namespace SparseFunc{
////block matrix operations
inline double Matrix_Element(const SparseMatrixT& A,const int i,const int j){return A.coeff(i,j);}
inline double Matrix_Element(const MatrixX& A,const int i,const int j){return A(i,j);}

template<int dim,class T_MAT> void Add_Block(SparseMatrixT& K,const int K_i,const int K_j,const T_MAT& K_b,const int Kb_i=0,const int Kb_j=0)
{for(int i=0;i<dim;i++)for(int j=0;j<dim;j++){K.coeffRef(K_i*dim+i,K_j*dim+j)+=Matrix_Element(K_b,Kb_i*dim+i,Kb_j*dim+j);}}

template<int dim,class T_MAT> void Copy_Block(SparseMatrixT& K,const int K_i,const int K_j,const T_MAT& K_b,const int Kb_i=0,const int Kb_j=0)
{for(int i=0;i<dim;i++)for(int j=0;j<dim;j++){K.coeffRef(K_i*dim+i,K_j*dim+j)=Matrix_Element(K_b,Kb_i*dim+i,Kb_j*dim+j);}}

template<int dim,class T_MAT> void Set_Block(SparseMatrixT& K,const int K_i,const int K_j,const double value)
{for(int i=0;i<dim;i++)for(int j=0;j<dim;j++){K.coeffRef(K_i*dim+i,K_j*dim+j)=value;}}
};

//////////////////////////////////////////////////////////////////////////
////Eigen sparse linear solvers

namespace SparseSolver{
    class Params
    {public:
        double tolerance=(double)1e-5;
        int max_iter_num=1000;
    };

	////Eigen solvers
    inline bool CG(const SparseMatrix<double>& A,VectorN<double>& x,const VectorN<double>& b,const Params params=Params())
	{
        Eigen::ConjugateGradient<SparseMatrixT,Eigen::Upper,Eigen::IdentityPreconditioner> cg;
        cg.setMaxIterations(params.max_iter_num);
        cg.setTolerance(params.tolerance);
        cg.compute(A);
        if(cg.info()!=Eigen::Success){std::cerr<<"Error: [Sparse] Eigen CG solver factorization failed."<<std::endl;return false;}
        x = cg.solve(b);
        if(cg.info()!=Eigen::Success){std::cerr<<"Error: [Sparse] Eigen CG solver failed."<<std::endl;return false;}
        std::cout<<"Eigen CG solver converge in "<<cg.iterations()<<" iterations, error: "<<cg.error()<<std::endl;return true;	
	}

	inline bool ICPCG(const SparseMatrix<double>& A,VectorN<double>& x,const VectorN<double>& b,const Params params)
    {
        Eigen::ConjugateGradient<SparseMatrixT,Eigen::Upper,Eigen::IncompleteCholesky<double,Eigen::Upper,Eigen::NaturalOrdering<int> > > cg;
        cg.setMaxIterations(params.max_iter_num);
        cg.setTolerance(params.tolerance);
        cg.compute(A);
        if(cg.info()!=Eigen::Success){std::cerr<<"Error: [Sparse] Eigen ICPCG solver factorization failed."<<std::endl;return false;}
        x = cg.solve(b);
        if(cg.info()!=Eigen::Success){std::cerr<<"Error: [Sparse] Eigen ICPCG solver failed."<<std::endl;return false;}
        std::cout<<"Eigen ICPCG solver converge in "<<cg.iterations()<<" iterations, error: "<<cg.error()<<std::endl;return true;
    }

    inline bool LU(const SparseMatrix<double>& A,VectorN<double>& x,const VectorN<double>& b,const Params params=Params())
	{
		MatrixX Ad(A);x=Ad.fullPivLu().solve(b);return true;
	}
};

//////////////////////////////////////////////////////////////////////////
////Container alias

template<class T> using Array=std::vector<T>;
template<class T> using List=std::list<T>;
template<class T,class CMP=std::less<T> > using Heap=std::priority_queue<T,std::vector<T>,CMP>;
template<class T_KEY,class T> using Hashtable=std::unordered_map<T_KEY,T>;
template<class T_KEY,class T> using HashtableMultiValue=std::unordered_multimap<T_KEY,T>;
template<class T_KEY> using Hashset=std::unordered_set<T_KEY>;

template<class T1,class T2> using Pair=std::pair<T1,T2>;
template<class T> using ArrayPtr=std::shared_ptr<std::vector<T> >;

using size_type=std::vector<int>::size_type;
using uchar=unsigned char;
using ushort=unsigned short;

////std::vector with fixed size
template<class T,int n> using ArrayF=std::array<T,n>;
constexpr int Pow(int x,int p){return p==1?x:x*Pow(x,p-1);}
constexpr int Factorial(int n){return n<=1?1:(n*Factorial(n-1));}
template<class T,int d> using ArrayF2P=ArrayF<T,Pow(2,d) >;
template<class T,int d> using ArrayF3P=ArrayF<T,Pow(3,d) >;

//////////////////////////////////////////////////////////////////////////
////Hash keys
namespace std{
template<> struct hash<Vector2i>
{typedef Vector2i argument_type;typedef std::size_t result_type;
	result_type operator()(argument_type const& arg) const
	{result_type const h1(std::hash<int>()(arg[0]));result_type const h2(std::hash<int>()(arg[1]));return h1^(h2<<1);}
};
template<> struct hash<Vector3i>
{typedef Vector3i argument_type;typedef std::size_t result_type;
	result_type operator()(argument_type const& arg) const
	{result_type const h1(std::hash<int>()(arg[0]));result_type const h2(std::hash<int>()(arg[1]));
	result_type const h3(std::hash<int>()(arg[2]));return h1^(h2<<1)^h3;}
};
template<> struct hash<Vector4i>
{typedef Vector4i argument_type;typedef std::size_t result_type;
	result_type operator()(argument_type const& arg) const
	{result_type const h1(std::hash<int>()(arg[0]));result_type const h2(std::hash<int>()(arg[1]));
	result_type const h3(std::hash<int>()(arg[2]));result_type const h4(std::hash<int>()(arg[3]));return h1^(h2<<1)^h3^(h4<<2);}
};}

//////////////////////////////////////////////////////////////////////////
////Analytical geometry

template<int d> class Box
{using VectorD=Vector<double,d>;using VectorDi=Vector<int,d>;
public:
	VectorD min_corner,max_corner;
	Box(const VectorD& _min=VectorD::Zero(),const VectorD& _max=VectorD::Zero()):min_corner(_min),max_corner(_max){}
};

//////////////////////////////////////////////////////////////////////////
////Dimension conversion

template<class T,int d1,int d2> void Dim_Conversion(const Vector<T,d1>& input,/*result*/Vector<T,d2>& output,const T filled_value=(T)0)
{
	int n=d1<d2?d1:d2;
	for(int i=0;i<n;i++)output[i]=input[i];
	for(int i=n;i<d2;i++)output[i]=filled_value;
}

template<class T,int d1,int d2> void Dim_Conversion_Array(const std::vector<Vector<T,d1> >& input,/*result*/std::vector<Vector<T,d2> >& output,const T filled_value=(T)0)
{
	for(size_type i=0;i<input.size();i++){
		Dim_Conversion<T,d1,d2>(input[i],output[i],filled_value);}
}

#endif
