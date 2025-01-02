#pragma once

/** \addtogroup finance
 *  @{
 */

namespace mlib
{
  namespace finance
  {

    /// Define the type of option
    enum class OptionType
    {
      EuropeanCall, ///< European call option
      EuropeanPut,  ///< European put option
      None          ///< No option type
    };

  }

}

/** @}*/
