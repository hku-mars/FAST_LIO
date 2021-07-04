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
 * @file mtk/types/vect.hpp
 * @brief Basic vectors interpreted as manifolds.
 * 
 * This file also implements a simple wrapper for matrices, for arbitrary scalars
 * and for positive scalars.
 */
#ifndef VECT_H_
#define VECT_H_

#include <iosfwd>
#include <iostream>
#include <vector>

#include "../src/vectview.hpp"

namespace MTK {

static const Eigen::IOFormat IO_no_spaces(Eigen::StreamPrecision, Eigen::DontAlignCols, ",", ",", "", "", "[", "]"); 


/**
 * A simple vector class.
 * Implementation is basically a wrapper around Eigen::Matrix with manifold 
 * requirements added.
 */
template<int D = 3, class _scalar = double, int _Options=Eigen::AutoAlign>
struct vect : public Eigen::Matrix<_scalar, D, 1, _Options> {
	typedef Eigen::Matrix<_scalar, D, 1, _Options> base;
	enum {DOF = D, DIM = D, TYP = 0};
	typedef _scalar scalar;
	
	//using base::operator=;
	
	/** Standard constructor. Sets all values to zero. */
	vect(const base &src = base::Zero()) : base(src) {}

	/** Constructor copying the value of the expression \a other */
	template<typename OtherDerived>
	EIGEN_STRONG_INLINE vect(const Eigen::DenseBase<OtherDerived>& other) : base(other) {}
	
	/** Construct from memory. */
	vect(const scalar* src, int size = DOF) : base(base::Map(src, size)) { }

	void boxplus(MTK::vectview<const scalar, D> vec, scalar scale=1) {
		*this += scale * vec;
	}
	void boxminus(MTK::vectview<scalar, D> res, const vect<D, scalar>& other) const {
		res = *this - other;
	}

	void oplus(MTK::vectview<const scalar, D> vec, scalar scale=1) {
		*this += scale * vec;
	}

	void S2_hat(Eigen::Matrix<scalar, 3, 3> &res)
	{
		res = Eigen::Matrix<scalar, 3, 3>::Zero();
	}

	void S2_Nx_yy(Eigen::Matrix<scalar, 2, 3> &res)
	{
		std::cerr << "wrong idx for S2" << std::endl;
		std::exit(100);	
    	res = Eigen::Matrix<scalar, 2, 3>::Zero();
	}

	void S2_Mx(Eigen::Matrix<scalar, 3, 2> &res, MTK::vectview<const scalar, 2> delta)
	{
		std::cerr << "wrong idx for S2" << std::endl;
		std::exit(100);	
    	res = Eigen::Matrix<scalar, 3, 2>::Zero();
	}

	friend std::ostream& operator<<(std::ostream &os, const vect<D, scalar, _Options>& v){
		// Eigen sometimes messes with the streams flags, so output manually:
		for(int i=0; i<DOF; ++i)
			os << v(i) << " ";
		return os;
	}
	friend std::istream& operator>>(std::istream &is, vect<D, scalar, _Options>& v){
		char term=0;
		is >> std::ws; // skip whitespace
		switch(is.peek()) {
		case '(': term=')'; is.ignore(1); break;
		case '[': term=']'; is.ignore(1); break;
		case '{': term='}'; is.ignore(1); break;
		default: break;
		}
		if(D==Eigen::Dynamic) { 
			assert(term !=0 && "Dynamic vectors must be embraced");
			std::vector<scalar> temp;
			while(is.good() && is.peek() != term) {
				scalar x;
				is >> x;
				temp.push_back(x);
				if(is.peek()==',') is.ignore(1);
			}
			v = vect::Map(temp.data(), temp.size());
		} else
			for(int i=0; i<v.size(); ++i){
				is >> v[i];
				if(is.peek()==',') { // ignore commas between values
					is.ignore(1);
				}
			}
		if(term!=0) {
			char x;
			is >> x;
			if(x!=term) {
				is.setstate(is.badbit);
//				assert(x==term && "start and end bracket do not match!");
			}
		}
		return is;
	}
	
	template<int dim>
	vectview<scalar, dim> tail(){
		BOOST_STATIC_ASSERT(0< dim && dim <= DOF);
		return base::template tail<dim>();
	}
	template<int dim>
	vectview<const scalar, dim> tail() const{
		BOOST_STATIC_ASSERT(0< dim && dim <= DOF);
		return base::template tail<dim>();
	}
	template<int dim>
	vectview<scalar, dim> head(){
		BOOST_STATIC_ASSERT(0< dim && dim <= DOF);
		return base::template head<dim>();
	}
	template<int dim>
	vectview<const scalar, dim> head() const{
		BOOST_STATIC_ASSERT(0< dim && dim <= DOF);
		return base::template head<dim>();
	}
};


/**
 * A simple matrix class.
 * Implementation is basically a wrapper around Eigen::Matrix with manifold 
 * requirements added, i.e., matrix is viewed as a plain vector for that.
 */
template<int M, int N, class _scalar = double, int _Options = Eigen::Matrix<_scalar, M, N>::Options>
struct matrix : public Eigen::Matrix<_scalar, M, N, _Options> {
	typedef Eigen::Matrix<_scalar, M, N, _Options> base; 
	enum {DOF = M * N, TYP = 4, DIM=0};
	typedef _scalar scalar;
	
	using base::operator=; 
	
	/** Standard constructor. Sets all values to zero. */
	matrix() {
		base::setZero();
	}
	
	/** Constructor copying the value of the expression \a other */
	template<typename OtherDerived>
	EIGEN_STRONG_INLINE matrix(const Eigen::MatrixBase<OtherDerived>& other) : base(other) {}
	
	/** Construct from memory. */
	matrix(const scalar* src) : base(src) { } 
	
	void boxplus(MTK::vectview<const scalar, DOF> vec, scalar scale = 1) {
		*this += scale * base::Map(vec.data());
	}
	void boxminus(MTK::vectview<scalar, DOF> res, const matrix& other) const {
		base::Map(res.data()) = *this - other;
	}

	void S2_hat(Eigen::Matrix<scalar, 3, 3> &res)
	{
		res = Eigen::Matrix<scalar, 3, 3>::Zero();
	}

	void oplus(MTK::vectview<const scalar, DOF> vec, scalar scale = 1) {
		*this += scale * base::Map(vec.data());
	}

	void S2_Nx_yy(Eigen::Matrix<scalar, 2, 3> &res)
	{
		std::cerr << "wrong idx for S2" << std::endl;
		std::exit(100);	
    	res = Eigen::Matrix<scalar, 2, 3>::Zero();
	}

	void S2_Mx(Eigen::Matrix<scalar, 3, 2> &res, MTK::vectview<const scalar, 2> delta)
	{
		std::cerr << "wrong idx for S2" << std::endl;
		std::exit(100);	
    	res = Eigen::Matrix<scalar, 3, 2>::Zero();
	}

	friend std::ostream& operator<<(std::ostream &os, const matrix<M, N, scalar, _Options>& mat){
		for(int i=0; i<DOF; ++i){
			os << mat.data()[i] << " ";
		}
		return os;
	}
	friend std::istream& operator>>(std::istream &is, matrix<M, N, scalar, _Options>& mat){
		for(int i=0; i<DOF; ++i){
			is >> mat.data()[i];
		}
		return is;
	}
};// @todo What if M / N = Eigen::Dynamic?



/**
 * A simple scalar type.
 */
template<class _scalar = double>
struct Scalar {
	enum {DOF = 1, TYP = 5, DIM=0};
	typedef _scalar scalar;
	
	scalar value;
	
	Scalar(const scalar& value = scalar(0)) : value(value) {}
	operator const scalar&() const { return value; }
	operator scalar&() { return value; }
	Scalar& operator=(const scalar& val) { value = val; return *this; }

	void S2_hat(Eigen::Matrix<scalar, 3, 3> &res)
	{
		res = Eigen::Matrix<scalar, 3, 3>::Zero();
	}

	void S2_Nx_yy(Eigen::Matrix<scalar, 2, 3> &res)
	{
		std::cerr << "wrong idx for S2" << std::endl;
		std::exit(100);	
    	res = Eigen::Matrix<scalar, 2, 3>::Zero();
	}

	void S2_Mx(Eigen::Matrix<scalar, 3, 2> &res, MTK::vectview<const scalar, 2> delta)
	{
		std::cerr << "wrong idx for S2" << std::endl;
		std::exit(100);	
    	res = Eigen::Matrix<scalar, 3, 2>::Zero();
	}

	void oplus(MTK::vectview<const scalar, DOF> vec, scalar scale=1) {
		value += scale * vec[0];
	}

	void boxplus(MTK::vectview<const scalar, DOF> vec, scalar scale=1) {
		value += scale * vec[0];
	}
	void boxminus(MTK::vectview<scalar, DOF> res, const Scalar& other) const {
		res[0] = *this - other;
	}
};

/**
 * Positive scalars.
 * Boxplus is implemented using multiplication by @f$x\boxplus\delta = x\cdot\exp(\delta) @f$.
 */
template<class _scalar = double>
struct PositiveScalar {
	enum {DOF = 1, TYP = 6, DIM=0};
	typedef _scalar scalar;
	
	scalar value;
	
	PositiveScalar(const scalar& value = scalar(1)) : value(value) {
		assert(value > scalar(0));
	}
	operator const scalar&() const { return value; }
	PositiveScalar& operator=(const scalar& val) { assert(val>0); value = val; return *this; }

	void boxplus(MTK::vectview<const scalar, DOF> vec, scalar scale = 1) {
		value *= std::exp(scale * vec[0]);
	}
	void boxminus(MTK::vectview<scalar, DOF> res, const PositiveScalar& other) const {
		res[0] = std::log(*this / other);
	}

	void oplus(MTK::vectview<const scalar, DOF> vec, scalar scale = 1) {
		value *= std::exp(scale * vec[0]);
	}

	void S2_hat(Eigen::Matrix<scalar, 3, 3> &res)
	{
		res = Eigen::Matrix<scalar, 3, 3>::Zero();
	}

	void S2_Nx_yy(Eigen::Matrix<scalar, 2, 3> &res)
	{
		std::cerr << "wrong idx for S2" << std::endl;
		std::exit(100);	
    	res = Eigen::Matrix<scalar, 2, 3>::Zero();
	}

	void S2_Mx(Eigen::Matrix<scalar, 3, 2> &res, MTK::vectview<const scalar, 2> delta)
	{
		std::cerr << "wrong idx for S2" << std::endl;
		std::exit(100);	
    	res = Eigen::Matrix<scalar, 3, 2>::Zero();
	}


	friend std::istream& operator>>(std::istream &is, PositiveScalar<scalar>& s){
		is >> s.value;
		assert(s.value > 0);
		return is;
	}
};

template<class _scalar = double>
struct Complex : public std::complex<_scalar>{
	enum {DOF = 2, TYP = 7, DIM=0};
	typedef _scalar scalar;
	
	typedef std::complex<scalar> Base;
	
	Complex(const Base& value) : Base(value) {}
	Complex(const scalar& re = 0.0, const scalar& im = 0.0) : Base(re, im) {}
	Complex(const MTK::vectview<const scalar, 2> &in) : Base(in[0], in[1]) {}
	template<class Derived>
	Complex(const Eigen::DenseBase<Derived> &in) : Base(in[0], in[1]) {}
	
	void boxplus(MTK::vectview<const scalar, DOF> vec, scalar scale = 1) {
		Base::real() += scale * vec[0];
		Base::imag() += scale * vec[1];
	};
	void boxminus(MTK::vectview<scalar, DOF> res, const Complex& other) const {
		Complex diff = *this - other;
		res << diff.real(), diff.imag();
	}

	void S2_hat(Eigen::Matrix<scalar, 3, 3> &res)
	{
		res = Eigen::Matrix<scalar, 3, 3>::Zero();
	}

	void oplus(MTK::vectview<const scalar, DOF> vec, scalar scale = 1) {
		Base::real() += scale * vec[0];
		Base::imag() += scale * vec[1];
	};

	void S2_Nx_yy(Eigen::Matrix<scalar, 2, 3> &res)
	{
		std::cerr << "wrong idx for S2" << std::endl;
		std::exit(100);	
    	res = Eigen::Matrix<scalar, 2, 3>::Zero();
	}

	void S2_Mx(Eigen::Matrix<scalar, 3, 2> &res, MTK::vectview<const scalar, 2> delta)
	{
		std::cerr << "wrong idx for S2" << std::endl;
		std::exit(100);	
    	res = Eigen::Matrix<scalar, 3, 2>::Zero();
	}
	
	scalar squaredNorm() const {
		return std::pow(Base::real(),2) + std::pow(Base::imag(),2);
	}
	
	const scalar& operator()(int i) const {
		assert(0<=i && i<2 && "Index out of range");
		return i==0 ? Base::real() : Base::imag();
	}
	scalar& operator()(int i){
		assert(0<=i && i<2 && "Index out of range");
		return i==0 ? Base::real() : Base::imag();
	}
};


namespace internal {

template<int dim, class Scalar, int Options>
struct UnalignedType<vect<dim, Scalar, Options > >{
	typedef vect<dim, Scalar, Options | Eigen::DontAlign> type;
};

}  // namespace internal


}  // namespace MTK




#endif /*VECT_H_*/
