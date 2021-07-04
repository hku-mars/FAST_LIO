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
 * @file mtk/src/mtkmath.hpp
 * @brief several math utility functions.
 */

#ifndef MTKMATH_H_
#define MTKMATH_H_

#include <cmath>

#include <boost/math/tools/precision.hpp>

#include "../types/vect.hpp"

#ifndef M_PI
#define M_PI  3.1415926535897932384626433832795
#endif


namespace MTK {

namespace internal {

template<class Manifold>
struct traits {
	typedef typename Manifold::scalar scalar;
	enum {DOF = Manifold::DOF};
	typedef vect<DOF, scalar> vectorized_type;
	typedef Eigen::Matrix<scalar, DOF, DOF> matrix_type;
};

template<>
struct traits<float> : traits<Scalar<float> > {};
template<>
struct traits<double> : traits<Scalar<double> > {};

}  // namespace internal

/**
 * \defgroup MTKMath Mathematical helper functions
 */
//@{

//! constant @f$ \pi @f$
const double pi = M_PI;

template<class scalar> inline scalar tolerance();

template<> inline float  tolerance<float >() { return 1e-5f; }
template<> inline double tolerance<double>() { return 1e-11; }


/**
 * normalize @a x to @f$[-bound, bound] @f$.
 * 
 * result for @f$ x = bound + 2\cdot n\cdot bound @f$ is arbitrary @f$\pm bound @f$.
 */
template<class scalar>
inline scalar normalize(scalar x, scalar bound){ //not used
	if(std::fabs(x) <= bound) return x;
	int r = (int)(x *(scalar(1.0)/ bound));
	return x - ((r + (r>>31) + 1) & ~1)*bound; 
}

/**
 * Calculate cosine and sinc of sqrt(x2).
 * @param x2 the squared angle must be non-negative
 * @return a pair containing cos and sinc of sqrt(x2)
 */
template<class scalar>
std::pair<scalar, scalar> cos_sinc_sqrt(const scalar &x2){
	using std::sqrt;
	using std::cos;
	using std::sin;
	static scalar const taylor_0_bound = boost::math::tools::epsilon<scalar>();
	static scalar const taylor_2_bound = sqrt(taylor_0_bound);
	static scalar const taylor_n_bound = sqrt(taylor_2_bound);
	
	assert(x2>=0 && "argument must be non-negative");
	
	// FIXME check if bigger bounds are possible
	if(x2>=taylor_n_bound) {
		// slow fall-back solution
		scalar x = sqrt(x2);
		return std::make_pair(cos(x), sin(x)/x); // x is greater than 0.
	}
	
	// FIXME Replace by Horner-Scheme (4 instead of 5 FLOP/term, numerically more stable, theoretically cos and sinc can be calculated in parallel using SSE2 mulpd/addpd)
	// TODO Find optimal coefficients using Remez algorithm
	static scalar const inv[] = {1/3., 1/4., 1/5., 1/6., 1/7., 1/8., 1/9.};
	scalar cosi = 1., sinc=1;
	scalar term = -1/2. * x2;
	for(int i=0; i<3; ++i) {
		cosi += term;
		term *= inv[2*i];
		sinc += term;
		term *= -inv[2*i+1] * x2;
	}
	
	return std::make_pair(cosi, sinc);
	
}

template<typename Base>
Eigen::Matrix<typename Base::scalar, 3, 3> hat(const Base& v) {
    Eigen::Matrix<typename Base::scalar, 3, 3> res;
	res << 0, -v[2], v[1],
		v[2], 0, -v[0],
		-v[1], v[0], 0;
	return res;
}

template<typename Base>
Eigen::Matrix<typename Base::scalar, 3, 3> A_inv_trans(const Base& v){
    Eigen::Matrix<typename Base::scalar, 3, 3> res;
    if(v.norm() > MTK::tolerance<typename Base::scalar>())
    {
        res = Eigen::Matrix<typename Base::scalar, 3, 3>::Identity() + 0.5 * hat<Base>(v) + (1 - v.norm() * std::cos(v.norm() / 2) / 2 / std::sin(v.norm() / 2)) * hat(v) * hat(v) / v.squaredNorm();
    
    }
    else
    {
        res = Eigen::Matrix<typename Base::scalar, 3, 3>::Identity();
    }
    
    return res;
}

template<typename Base>
Eigen::Matrix<typename Base::scalar, 3, 3> A_inv(const Base& v){
    Eigen::Matrix<typename Base::scalar, 3, 3> res;
    if(v.norm() > MTK::tolerance<typename Base::scalar>())
    {
        res = Eigen::Matrix<typename Base::scalar, 3, 3>::Identity() - 0.5 * hat<Base>(v) + (1 - v.norm() * std::cos(v.norm() / 2) / 2 / std::sin(v.norm() / 2)) * hat(v) * hat(v) / v.squaredNorm();
    
    }
    else
    {
        res = Eigen::Matrix<typename Base::scalar, 3, 3>::Identity();
    }
    
    return res;
}

template<typename scalar>
Eigen::Matrix<scalar, 2, 3> S2_w_expw_( Eigen::Matrix<scalar, 2, 1> v, scalar length)
	{
    	Eigen::Matrix<scalar, 2, 3> res;
    	scalar norm = std::sqrt(v[0]*v[0] + v[1]*v[1]);
		if(norm < MTK::tolerance<scalar>()){
	    	res = Eigen::Matrix<scalar, 2, 3>::Zero();
	    	res(0, 1) = 1;
	    	res(1, 2) = 1;
        	res /= length;
		}
		else{
			res << -v[0]*(1/norm-1/std::tan(norm))/std::sin(norm), norm/std::sin(norm), 0,
            	   -v[1]*(1/norm-1/std::tan(norm))/std::sin(norm), 0, norm/std::sin(norm);
        	res /= length;
    	}	
	}

template<typename Base>
Eigen::Matrix<typename Base::scalar, 3, 3> A_matrix(const Base & v){
    Eigen::Matrix<typename Base::scalar, 3, 3> res;
    double squaredNorm = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
	double norm = std::sqrt(squaredNorm);
	if(norm < MTK::tolerance<typename Base::scalar>()){
		res = Eigen::Matrix<typename Base::scalar, 3, 3>::Identity();
	}
	else{
		res = Eigen::Matrix<typename Base::scalar, 3, 3>::Identity() + (1 - std::cos(norm)) / squaredNorm * hat(v) + (1 - std::sin(norm) / norm) / squaredNorm * hat(v) * hat(v);
	}
    return res;
}

template<class scalar, int n>
scalar exp(vectview<scalar, n> result, vectview<const scalar, n> vec, const scalar& scale = 1) {
	scalar norm2 = vec.squaredNorm();
	std::pair<scalar, scalar> cos_sinc = cos_sinc_sqrt(scale*scale * norm2);
	scalar mult = cos_sinc.second * scale; 
	result = mult * vec;
	return cos_sinc.first;
}


/**
 * Inverse function to @c exp.
 * 
 * @param result @c vectview to the result
 * @param w      scalar part of input
 * @param vec    vector part of input
 * @param scale  scale result by this value
 * @param plus_minus_periodicity if true values @f$[w, vec]@f$ and @f$[-w, -vec]@f$ give the same result 
 */
template<class scalar, int n>
void log(vectview<scalar, n> result,
		const scalar &w, const vectview<const scalar, n> vec,
		const scalar &scale, bool plus_minus_periodicity)
{
	// FIXME implement optimized case for vec.squaredNorm() <= tolerance() * (w*w) via Rational Remez approximation ~> only one division
	scalar nv = vec.norm();
	if(nv < tolerance<scalar>()) {
		if(!plus_minus_periodicity && w < 0) {
			// find the maximal entry:
			int i;
			nv = vec.cwiseAbs().maxCoeff(&i);
			result = scale * std::atan2(nv, w) * vect<n, scalar>::Unit(i);
			return;
		}
		nv = tolerance<scalar>();
	}
	scalar s = scale / nv * (plus_minus_periodicity ? std::atan(nv / w) : std::atan2(nv, w) );
	
	result = s * vec;
}


} // namespace MTK


#endif /* MTKMATH_H_ */
