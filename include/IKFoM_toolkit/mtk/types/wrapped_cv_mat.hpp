/*
 *  Copyright (c) 2010--2011, Universitaet Bremen and DFKI GmbH
 *  All rights reserved.
 *
 *  Author: Rene Wagner <rene.wagner@dfki.de>
 *          Christoph Hertzberg <chtz@informatik.uni-bremen.de>
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
 *   * Neither the name of the Universitaet Bremen nor the DFKI GmbH 
 *     nor the names of its contributors may be used to endorse or 
 *     promote products derived from this software without specific 
 *     prior written permission.
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

#ifndef WRAPPED_CV_MAT_HPP_
#define WRAPPED_CV_MAT_HPP_

#include <Eigen/Core>
#include <opencv/cv.h>

namespace MTK {

template<class f_type>
struct cv_f_type;

template<>
struct cv_f_type<double>
{
	enum {value = CV_64F};
};

template<>
struct cv_f_type<float>
{
	enum {value = CV_32F};
};

/**
 * cv_mat wraps a CvMat around an Eigen Matrix
 */
template<int rows, int cols, class f_type = double>
class cv_mat : public matrix<rows, cols, f_type, cols==1 ? Eigen::ColMajor : Eigen::RowMajor>
{
	typedef matrix<rows, cols, f_type, cols==1 ? Eigen::ColMajor : Eigen::RowMajor> base_type;
	enum {type_ = cv_f_type<f_type>::value};
	CvMat cv_mat_;

public:
	cv_mat()
	{
		cv_mat_ = cvMat(rows, cols, type_, base_type::data());
	}

	cv_mat(const cv_mat& oth) : base_type(oth)
	{
		cv_mat_ = cvMat(rows, cols, type_, base_type::data());
	}

	template<class Derived>
	cv_mat(const Eigen::MatrixBase<Derived> &value) : base_type(value)
	{
		cv_mat_ = cvMat(rows, cols, type_, base_type::data());
	}

	template<class Derived>
	cv_mat& operator=(const Eigen::MatrixBase<Derived> &value)
	{
		base_type::operator=(value);
		return *this;
	}

	cv_mat& operator=(const cv_mat& value)
	{
		base_type::operator=(value);
		return *this;
	}
	
	// FIXME: Maybe overloading operator& is not a good idea ...
	CvMat* operator&()
	{
		return &cv_mat_;
	}
	const CvMat* operator&() const
	{
		return &cv_mat_;
	}
};

} // namespace MTK

#endif /* WRAPPED_CV_MAT_HPP_ */
