/* Generated from orogen/lib/orogen/templates/typekit/Opaques.cpp */

#include <mars/typekit/OpaqueTypes.hpp>
#include <mars/typekit/Opaques.hpp>

    /** Returns the intermediate value that is contained in \c real_type */
    /** Stores \c intermediate into \c real_type. \c intermediate is owned by \c
     * real_type afterwards. */
    /** Release ownership of \c real_type on the corresponding intermediate
     * pointer.
     */


void orogen_typekits::toIntermediate(::wrappers::Matrix< double, 3, 1 >& intermediate, ::mars::utils::Vector const& real_type)
{
    //re-use Eigen opaque conversion from base/types
    // toIntermediate(intermediate, real_type);
    intermediate.data[0] = real_type.x();
    intermediate.data[1] = real_type.y();
    intermediate.data[2] = real_type.z();
}

void orogen_typekits::fromIntermediate(::mars::utils::Vector& real_type, ::wrappers::Matrix< double, 3, 1 > const& intermediate)
{
    //re-use Eigen opaque conversion from base/types
    // fromIntermediate(real_type, intermediate);
    real_type.x() = intermediate.data[0];
    real_type.y() = intermediate.data[1];
    real_type.z() = intermediate.data[2];
}

