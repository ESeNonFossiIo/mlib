#pragma once

/** \addtogroup finance
 *  @{
 */

namespace numerix {
namespace finance {

/// Define the type of option
enum class OptionType {
    EuropeanCall, ///< European call option
    EuropeanPut,  ///< European put option
    None          ///< No option type
};

} // namespace finance

} // namespace numerix

/** @}*/
