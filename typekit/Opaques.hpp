/* Generated from orogen/lib/orogen/templates/typekit/Opaques.hpp */

#ifndef __OROGEN_GENERATED_mars_USER_MARSHALLING_HH
#define __OROGEN_GENERATED_mars_USER_MARSHALLING_HH

#include <mars/typekit/OpaqueFwd.hpp>

namespace orogen_typekits
{
    
    /** Converts \c real_type into \c intermediate */
    void toIntermediate(::wrappers::Matrix< double, 3, 1 >& intermediate, ::mars::utils::Vector const& real_type);
    /** Converts \c intermediate into \c real_type */
    void fromIntermediate(::mars::utils::Vector& real_type, ::wrappers::Matrix< double, 3, 1 > const& intermediate);
        
    
}

#endif

