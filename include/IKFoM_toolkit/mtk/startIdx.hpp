// This is an advanced implementation of the algorithm described in the
// following paper:
//    C. Hertzberg,  R.  Wagner,  U.  Frese,  and  L.  Schroder.  Integratinggeneric   sensor   fusion   algorithms   with   sound   state   representationsthrough  encapsulation  of  manifolds.
//    CoRR,  vol.  abs/1107.1119,  2011.[Online]. Available: http://arxiv.org/abs/1107.1119

/*
 *  Copyright (c) 2019--2023, The University of Hong Kong
 *  All rights reserved.
 *
 *  Modifier: Dongjiao HE <hdj65822@connect.hku.hk>
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Universitaet Bremen nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
 
/*
 *  Copyright (c) 2008--2011, Universitaet Bremen
 *  All rights reserved.
 *
 *  Author: Christoph Hertzberg <chtz@informatik.uni-bremen.de>
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Universitaet Bremen nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
/**
 * @file mtk/startIdx.hpp 
 * @brief Tools to access sub-elements of compound manifolds.
 */
#ifndef GET_START_INDEX_H_
#define GET_START_INDEX_H_

#include <Eigen/Core>

#include "src/SubManifold.hpp"
#include "src/vectview.hpp"

namespace MTK {


/** 
 * \defgroup SubManifolds Accessing Submanifolds
 * For compound manifolds constructed using MTK_BUILD_MANIFOLD, member pointers
 * can be used to get sub-vectors or matrix-blocks of a corresponding big matrix.
 * E.g. for a type @a pose consisting of @a orient and @a trans the member pointers
 * @c &pose::orient and @c &pose::trans give all required information and are still
 * valid if the base type gets extended or the actual types of @a orient and @a trans
 * change (e.g. from 2D to 3D).
 * 
 * @todo Maybe require manifolds to typedef MatrixType and VectorType, etc.
 */
//@{

/**
 * Determine the index of a sub-variable within a compound variable.
 */
template<class Base, class T, int idx, int dim> 
int getStartIdx( MTK::SubManifold<T, idx, dim> Base::*)
{
	return idx;
}

template<class Base, class T, int idx, int dim> 
int getStartIdx_( MTK::SubManifold<T, idx, dim> Base::*)
{
	return dim;
}

/**
 * Determine the degrees of freedom of a sub-variable within a compound variable.
 */
template<class Base, class T, int idx, int dim> 
int getDof( MTK::SubManifold<T, idx, dim> Base::*)
{
	return T::DOF;
}
template<class Base, class T, int idx, int dim> 
int getDim( MTK::SubManifold<T, idx, dim> Base::*)
{
	return T::DIM;
}

/**
 * set the diagonal elements of a covariance matrix corresponding to a sub-variable
 */
template<class Base, class T, int idx, int dim> 
void setDiagonal(Eigen::Matrix<typename Base::scalar, Base::DOF, Base::DOF> &cov, 
		MTK::SubManifold<T, idx, dim> Base::*, const typename Base::scalar &val)
{
	cov.diagonal().template segment<T::DOF>(idx).setConstant(val);
}

template<class Base, class T, int idx, int dim> 
void setDiagonal_(Eigen::Matrix<typename Base::scalar, Base::DIM, Base::DIM> &cov, 
		MTK::SubManifold<T, idx, dim> Base::*, const typename Base::scalar &val)
{
	cov.diagonal().template segment<T::DIM>(dim).setConstant(val);
}

/**
 * Get the subblock of corresponding to two members, i.e.
 * \code
 *  Eigen::Matrix<double, Pose::DOF, Pose::DOF> m;
 *  MTK::subblock(m, &Pose::orient, &Pose::trans) = some_expression;
 *  MTK::subblock(m, &Pose::trans, &Pose::orient) = some_expression.trans();
 * \endcode
 * lets you modify mixed covariance entries in a bigger covariance matrix.
 */
template<class Base, class T1, int idx1, int dim1, class T2, int idx2, int dim2>
typename MTK::internal::CovBlock<Base, T1, T2>::Type
subblock(Eigen::Matrix<typename Base::scalar, Base::DOF, Base::DOF> &cov, 
		MTK::SubManifold<T1, idx1, dim1> Base::*, MTK::SubManifold<T2, idx2, dim2> Base::*)
{
	return cov.template block<T1::DOF, T2::DOF>(idx1, idx2);
}

template<class Base, class T1, int idx1,  int dim1, class T2, int idx2, int dim2>
typename MTK::internal::CovBlock_<Base, T1, T2>::Type
subblock_(Eigen::Matrix<typename Base::scalar, Base::DIM, Base::DIM> &cov, 
		MTK::SubManifold<T1, idx1, dim1> Base::*, MTK::SubManifold<T2, idx2, dim2> Base::*)
{
	return cov.template block<T1::DIM, T2::DIM>(dim1, dim2);
}

template<typename Base1, typename Base2, typename T1, typename T2, int idx1, int idx2, int dim1, int dim2>
typename MTK::internal::CrossCovBlock<Base1, Base2, T1, T2>::Type
subblock(Eigen::Matrix<typename Base1::scalar, Base1::DOF, Base2::DOF> &cov, MTK::SubManifold<T1, idx1, dim1> Base1::*, MTK::SubManifold<T2, idx2, dim2> Base2::*)
{
	return cov.template block<T1::DOF, T2::DOF>(idx1, idx2);
}

template<typename Base1, typename Base2, typename T1, typename T2, int idx1, int idx2, int dim1, int dim2>
typename MTK::internal::CrossCovBlock_<Base1, Base2, T1, T2>::Type
subblock_(Eigen::Matrix<typename Base1::scalar, Base1::DIM, Base2::DIM> &cov, MTK::SubManifold<T1, idx1, dim1> Base1::*, MTK::SubManifold<T2, idx2, dim2> Base2::*)
{
	return cov.template block<T1::DIM, T2::DIM>(dim1, dim2);
}
/**
 * Get the subblock of corresponding to a member, i.e.
 * \code
 *  Eigen::Matrix<double, Pose::DOF, Pose::DOF> m;
 *  MTK::subblock(m, &Pose::orient) = some_expression;
 * \endcode
 * lets you modify covariance entries in a bigger covariance matrix.
 */
template<class Base, class T, int idx, int dim>
typename MTK::internal::CovBlock_<Base, T, T>::Type
subblock_(Eigen::Matrix<typename Base::scalar, Base::DIM, Base::DIM> &cov, 
		MTK::SubManifold<T, idx, dim> Base::*)
{
	return cov.template block<T::DIM, T::DIM>(dim, dim);
}

template<class Base, class T, int idx, int dim>
typename MTK::internal::CovBlock<Base, T, T>::Type
subblock(Eigen::Matrix<typename Base::scalar, Base::DOF, Base::DOF> &cov, 
		MTK::SubManifold<T, idx, dim> Base::*)
{
	return cov.template block<T::DOF, T::DOF>(idx, idx);
}

template<typename Base>
class get_cov { 
public:
    typedef Eigen::Matrix<typename Base::scalar, Base::DOF, Base::DOF> type;
    typedef const Eigen::Matrix<typename Base::scalar, Base::DOF, Base::DOF> const_type;
};

template<typename Base>
class get_cov_ { 
public:
    typedef Eigen::Matrix<typename Base::scalar, Base::DIM, Base::DIM> type;
    typedef const Eigen::Matrix<typename Base::scalar, Base::DIM, Base::DIM> const_type;
};

template<typename Base1, typename Base2>
class get_cross_cov {
public:
    typedef Eigen::Matrix<typename Base1::scalar, Base1::DOF, Base2::DOF> type;
    typedef const type const_type;
};

template<typename Base1, typename Base2>
class get_cross_cov_ {
public:
    typedef Eigen::Matrix<typename Base1::scalar, Base1::DIM, Base2::DIM> type;
    typedef const type const_type;
};


template<class Base, class T, int idx, int dim>
vectview<typename Base::scalar, T::DIM>
subvector_impl_(vectview<typename Base::scalar, Base::DIM> vec, SubManifold<T, idx, dim> Base::*)
{
	return vec.template segment<T::DIM>(dim);
}

template<class Base, class T, int idx, int dim>
vectview<typename Base::scalar, T::DOF>
subvector_impl(vectview<typename Base::scalar, Base::DOF> vec, SubManifold<T, idx, dim> Base::*)
{
	return vec.template segment<T::DOF>(idx);
}

/**
 * Get the subvector corresponding to a sub-manifold from a bigger vector.
 */
 template<class Scalar, int BaseDIM, class Base, class T, int idx, int dim>
vectview<Scalar, T::DIM>
subvector_(vectview<Scalar, BaseDIM> vec, SubManifold<T, idx, dim> Base::* ptr)
{
	return subvector_impl_(vec, ptr);
}

template<class Scalar, int BaseDOF, class Base, class T, int idx, int dim>
vectview<Scalar, T::DOF>
subvector(vectview<Scalar, BaseDOF> vec, SubManifold<T, idx, dim> Base::* ptr)
{
	return subvector_impl(vec, ptr);
}

/**
 * @todo This should be covered already by subvector(vectview<typename Base::scalar,Base::DOF> vec,SubManifold<T,idx> Base::*)
 */
template<class Scalar, int BaseDOF, class Base, class T, int idx, int dim>
vectview<Scalar, T::DOF>
subvector(Eigen::Matrix<Scalar, BaseDOF, 1>& vec, SubManifold<T, idx, dim> Base::* ptr)
{
	return subvector_impl(vectview<Scalar, BaseDOF>(vec), ptr);
}
 
template<class Scalar, int BaseDIM, class Base, class T, int idx, int dim>
vectview<Scalar, T::DIM>
subvector_(Eigen::Matrix<Scalar, BaseDIM, 1>& vec, SubManifold<T, idx, dim> Base::* ptr)
{
	return subvector_impl_(vectview<Scalar, BaseDIM>(vec), ptr);
}

template<class Scalar, int BaseDIM, class Base, class T, int idx, int dim>
vectview<const Scalar, T::DIM>
subvector_(const Eigen::Matrix<Scalar, BaseDIM, 1>& vec, SubManifold<T, idx, dim> Base::* ptr)
{
	return subvector_impl_(vectview<const Scalar, BaseDIM>(vec), ptr);
}

template<class Scalar, int BaseDOF, class Base, class T, int idx, int dim>
vectview<const Scalar, T::DOF>
subvector(const Eigen::Matrix<Scalar, BaseDOF, 1>& vec, SubManifold<T, idx, dim> Base::* ptr)
{
	return subvector_impl(vectview<const Scalar, BaseDOF>(vec), ptr);
}


/**
 * const version of subvector(vectview<typename Base::scalar,Base::DOF> vec,SubManifold<T,idx> Base::*)
 */
template<class Base, class T, int idx, int dim>
vectview<const typename Base::scalar, T::DOF>
subvector_impl(const vectview<const typename Base::scalar, Base::DOF> cvec, SubManifold<T, idx, dim> Base::*)
{
	return cvec.template segment<T::DOF>(idx);
}

template<class Base, class T, int idx, int dim>
vectview<const typename Base::scalar, T::DIM>
subvector_impl_(const vectview<const typename Base::scalar, Base::DIM> cvec, SubManifold<T, idx, dim> Base::*)
{
	return cvec.template segment<T::DIM>(dim);
}

template<class Scalar, int BaseDOF, class Base, class T, int idx, int dim>
vectview<const Scalar, T::DOF>
subvector(const vectview<const Scalar, BaseDOF> cvec, SubManifold<T, idx, dim> Base::* ptr)
{
	return subvector_impl(cvec, ptr);
}


} // namespace MTK

#endif // GET_START_INDEX_H_
