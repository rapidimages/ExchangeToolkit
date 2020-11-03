#pragma once

#include <glm/gtc/type_ptr.hpp>
#include <glm/glm.hpp>
#include "ExchangeToolkit.h"

namespace ts3d {
	/*! \brief Alias for a 4x4 matrix type.
	*  \ingroup eigen_bridge
	*/
	using MatrixType = glm::dmat4;

	/*! \brief Alias for a 4d vector type.
	*  \ingroup eigen_bridge
	*/
	using VectorType = glm::dvec4;

	/*! \brief Alias for a 4d position type.
	*  \ingroup eigen_bridge
	*/
	using PositionType = glm::dvec4;

	/*! \brief Use this function to obtain a vector to be used with the matrix.
	*  \ingroup eigen_bridge
	*/
	static inline VectorType getVector( A3DVector3dData const &vec ) {
		return ts3d::VectorType( vec.m_dX, vec.m_dY, vec.m_dZ, 0. );
	}

    /*! \brief Use this function to obtain and Exchange Vector from an Eigen vector/position.
    *  \ingroup eigen_bridge
    */
    static inline A3DVector3dData getExchangeVector( VectorType const &vec ) {
        A3DVector3dData result;
        A3D_INITIALIZE_DATA( A3DVector3dData, result );
        result.m_dX = vec[0];
        result.m_dY = vec[1];
        result.m_dZ = vec[2];
        return result;
    }

	/*! \brief Use this function to obtain a direction.
	*  \ingroup eigen_bridge
	*/
	static inline PositionType getPosition( A3DVector3dData const &vec ) {
		return ts3d::PositionType( vec.m_dX, vec.m_dY, vec.m_dZ, 1. );
	}
}


namespace {
	static inline ts3d::MatrixType getMatrixFromCartesian( A3DMiscCartesianTransformation *xform ) {
		ts3d::A3DMiscCartesianTransformationWrapper d( xform );
		auto const mirror = (d->m_ucBehaviour & kA3DTransformationMirror) ? -1. : 1.;
		auto const s = ts3d::getVector( d->m_sScale );
		auto const o = ts3d::getPosition( d->m_sOrigin );
		auto const x = ts3d::getVector( d->m_sXVector );
		auto const y = ts3d::getVector( d->m_sYVector );
		auto const z = glm::dvec4(glm::cross(glm::dvec3(x), glm::dvec3(y)) * mirror, 0.);

		ts3d::MatrixType result;
		for(auto idx = 0u; idx < 4u; idx++) {
			result[0] = x * s[0];
			result[1] = y * s[1];
			result[2] = z * s[2];
			result[3] = o;
		}

		return result;
	}

	static inline ts3d::MatrixType getMatrixFromGeneralTransformation( A3DMiscGeneralTransformation *xform ) {
		ts3d::A3DMiscGeneralTransformationWrapper d( xform );

		auto const coeff = d->m_adCoeff;
		ts3d::MatrixType result = glm::make_mat4(coeff);
		// for(auto idx = 0u; idx < 16; ++idx) {
		// 	result[idx / 4] = coeff[idx];
		// }
		return result;
	}

}



namespace ts3d {
	/*! \brief This function returns a matrix corresponding to the A3DMiscTranslformation.
	* Both general and cartesian transformations are handled
	* \ingroup eigen_bridge
	*/
	static inline MatrixType getMatrix( A3DMiscTransformation *xform ) {
		if(nullptr == xform) {
			return ts3d::MatrixType(1.0);
		}

		auto t = kA3DTypeUnknown;
		A3DEntityGetType( xform, &t );

		switch(t) {
		case kA3DTypeMiscCartesianTransformation:
			return getMatrixFromCartesian( xform );
			break;
		case kA3DTypeMiscGeneralTransformation:
			break;
			return getMatrixFromGeneralTransformation( xform );
		default:
			throw std::invalid_argument( "Unexpected argument type provided" );
			break;
		}
		return ts3d::MatrixType(1.0);
	}

	/*! \brief Gets the matrix of the leaf entity.
	*  \ingroup eigen_bridge
	*/
	static inline MatrixType getMatrix( ts3d::Instance const &i ) {
		auto const leaf_type = i.leafType();
        if( kA3DTypeAsmProductOccurrence == leaf_type ) {
            return ts3d::getMatrix( getLocation( i.leaf() ) );
        } else if( isRepresentationItem( leaf_type ) ) {
            A3DRiRepresentationItemWrapper d( i.leaf() );
            A3DRiCoordinateSystemWrapper csw( d->m_pCoordinateSystem );
            return  ts3d::getMatrix( csw->m_pTransformation );
        }
		return MatrixType(1.0);
	}

	/*! \brief Gets the net matrix for a given instance.
	*  \ingroup eigen_bridge
	*
	* The matrix of each entry in the instance path is obtained
	* and accumulated to provide a net resultant transform.
	* \return Net transform matrix from the first to the last
	* entity in the instance path.
	*/
	static inline MatrixType getNetMatrix( ts3d::Instance const &i ) {
		if(i.path().size() > 1) {
			return getNetMatrix( i.owner() ) * getMatrix( i );
		}
		return getMatrix( i );
	}
}
