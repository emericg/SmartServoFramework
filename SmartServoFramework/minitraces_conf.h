/*!
 * COPYRIGHT (C) 2017 Emeric Grange - All Rights Reserved
 *
 * This file is part of MiniTraces.
 *
 * MiniTraces is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * MiniTraces is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with MiniTraces.  If not, see <http://www.gnu.org/licenses/>.
 *
 * \file      minitraces_conf.h
 * \author    Emeric Grange <emeric.grange@gmail.com>
 * \date      2017
 * \version   0.51
 */

#ifndef MINITRACES_CONF_H
#define MINITRACES_CONF_H

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus
/* ************************************************************************** */

// Import setting macros from SmartServoFramework
#include "SmartServoFramework_settings.h"

// =============================================================================
// GENERAL SETTINGS
// =============================================================================

#if ENABLE_DEBUG == 1
#define MINITRACES_LEVEL    2   // Enables all traces levels
#else
#define MINITRACES_LEVEL    1   // Only error and warnings traces
#endif

// Enable terminal colored output
#if ENABLE_COLORS == 1
#undef MINITRACES_COLORS
#define MINITRACES_COLORS   1
#else
#undef ENABLE_COLORS
#define MINITRACES_COLORS   0
#endif

// Advanced debugging features
#define MINITRACES_TIMESTAMPS       0   //!< 0: disabled, 1: trace tick (in milliseconds), 2: trace time (hh:mm:ss)
#define MINITRACES_FUNC_INFO        1
#define MINITRACES_FILE_INFO        0
#define MINITRACES_FORCED_SYNC      1
#define MINITRACES_STRICT_PADDING   0   //!< Not implemented yet

// =============================================================================
// PROGRAM IDENTIFIER
// =============================================================================

/*!
 * This string will be used to easily identify from which program a trace comes
 * from if multiple program are outputting traces with MiniTraces at the same time.
 * You can use bracket, spaces, colors...
 * Example: #define PID OUT_BLUE "[MINITRACE]" CLR_RESET " "
 *
 * Leave it blank if you don't need this feature!
 */
#define PID ""

// =============================================================================
// TRACE MODULES
// =============================================================================

/*!
 * \brief This is the list of module you can use when creating a trace with a TRACE_xxx macro.
 * \note The content of this enum must ALWAYS be in sync with the trace_modules_table[] below.
 *
 * When a TRACE_xxx macro is called with a given module, it will try to match it
 * with a TraceModule_t entry inside the trace_modules_table[] to get access to
 * the module parameters.
 */
enum TraceModule_e
{
    MAIN,
    DAPI,
    CAPI,
    DXL,
    HKX,
    TOOLS,
    SERIAL,
    SERVO,
    TABLES,
};

/*!
 * \brief This is the list of trace modules defined in your project.
 * \note The content of this enum must ALWAYS be in sync with the TraceModule_e enum above.
 *
 * - The first field is the "public" module's name, as it will be outputted by MiniTraces.
 * - The second field holds the description of the module.
 * - The last field indicate the level of traces a module can output, using one or a concatenation of TRACE_LEVEL_xxx macros.
 */
static TraceModule_t trace_modules_table[] =
{
    { "MAIN"   , "Main"                             , TRACE_LEVEL_DEBUG },
    { "D-API"  , "Direct API"                       , TRACE_LEVEL_DEBUG },
    { "C-API"  , "Controller API"                   , TRACE_LEVEL_DEBUG },
    { "DXL"    , "Dynamixel protocol"               , TRACE_LEVEL_DEBUG },
    { "HKX"    , "HerkuleX protocol"                , TRACE_LEVEL_DEBUG },
    { "TOOLS"  , "Various tools"                    , TRACE_LEVEL_DEBUG },
    { "SERIAL" , "Serial ports implementations"     , TRACE_LEVEL_DEBUG },
    { "SERVO"  , "Servo devices"                    , TRACE_LEVEL_DEBUG },
    { "TABLES" , "Control tables for servo device"  , TRACE_LEVEL_DEBUG },
};

/* ************************************************************************** */
#ifdef __cplusplus
}
#endif // __cplusplus

#endif // MINITRACES_CONF_H
