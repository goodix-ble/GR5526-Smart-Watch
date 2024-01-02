/**
 ****************************************************************************************
 *
 * @file ble_att.h
 *
 * @brief Attribute Protocol
 *
 ****************************************************************************************
 * @attention
  #####Copyright (c) 2019 GOODIX
  All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
  * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
  * Neither the name of GOODIX nor the names of its contributors may be used
    to endorse or promote products derived from this software without
    specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************************
 */

  /**
 * @addtogroup BLE
 * @{
 */

 /**
* @addtogroup BLE_ATT Attribute Protocol (ATT)
* @{
* @brief Definitions and prototypes for ATT.
*/


#ifndef __BLE_ATT_H__
#define __BLE_ATT_H__

/** @addtogroup BLE_ATT_DEFINES Defines
 * @{ */
#define BLE_ATT_UUID_16(uuid)                  (uuid)       /**< Convert CPU’s integer definition to LSB-first 16-bit UUID. */
#define BLE_ATT_MTU_DEFAULT                    (23)         /**< Default ATT MTU size in bytes. */
#define BLE_ATT_INVALID_HDL                    (0x0000)     /**< Invalid attribute handle. */
#define BLE_ATT_HANDLE_START                   (0x0001)     /**< Attribute handle start. */
#define BLE_ATT_HANDLE_END                     (0xFFFF)     /**< Attribute handle end. */

/** @defgroup BLE_ATT_UUID_LEN Attribute UUID Length(bytes)
 * @{ */
#define BLE_ATT_UUID_16_LEN                     0x0002    /**< UUID length: 2 bytes. */
#define BLE_ATT_UUID_32_LEN                     0x0004    /**< UUID length: 4 bytes. */
#define BLE_ATT_UUID_128_LEN                    0x0010    /**< UUID length: 16 bytes. */
/** @} */

/** @defgroup BLE_ATT_CHAR_PROPERTIES Characteristic Properties
 * @{ */
#define BLE_ATT_CHAR_PROP_BCAST                 0x01      /**< Characteristic Property: Broadcast. */
#define BLE_ATT_CHAR_PROP_RD                    0x02      /**< Characteristic Property: Read. */
#define BLE_ATT_CHAR_PROP_WR_NO_RESP            0x04      /**< Characteristic Property: Write Without Response. */
#define BLE_ATT_CHAR_PROP_WR                    0x08      /**< Characteristic Property: Write. */
#define BLE_ATT_CHAR_PROP_NTF                   0x10      /**< Characteristic Property: Notify. */
#define BLE_ATT_CHAR_PROP_IND                   0x20      /**< Characteristic Property: Indicate. */
#define BLE_ATT_CHAR_PROP_AUTH                  0x40      /**< Characteristic Property: Authenticated Signed Writes. */
#define BLE_ATT_CHAR_PROP_EXT_PROP              0x80      /**< Characteristic Property: Extended Properties. */
/** @} */

/** @defgroup BLE_ATT_CHAR_EXTENDED_PROPERTIES Characteristic Extended Properties
 * @{ */
#define BLE_ATT_EXT_RELIABLE_WRITE              0x0001    /**< Characteristic Extended Property: Reliable Write. */
#define BLE_ATT_EXT_WRITABLE_AUX                0x0002    /**< Characteristic Extended Property: Writable Auxiliaries. */
#define BLE_ATT_EXT_RFU                         0xFFFC    /**< Characteristic Extended Property:  Reserved for Future Use. */
/** @} */

/**@brief Characteristic Base UUID. */
#define BLE_ATT_BT_UUID_128             {0xFB, 0x34, 0x9B, 0x5F, 0x80, 0x00, 0x00, 0x80, \
                                         0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
/** @brief  Change a 16-bit UUID array to a 128-bit one (append 0).
  * @param  uuid: 16-bit UUID
  * @retval None
  */
#define BLE_ATT_16_TO_128_ARRAY(uuid)   {(uuid) & 0xFF, ((uuid) >> 8) & 0xFF, 0x00, 0x00, 0x00, \
                                         0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 }
/** @brief  Change a 16-bit UUID array to a 16-bit one (append 0).
  * @param  uuid: 16-bit UUID
  * @retval None
  */
#define BLE_ATT_16_TO_16_ARRAY(uuid)    {(uuid) & 0xFF, ((uuid) >> 8) & 0xFF}
/** @} */


/** @addtogroup BLE_ATT_ENUMERATIONS Enumerations
 * @{ */

/**@brief Attribute Specification Definitions: Common 16-bit (Universal Unique Identifier). */
typedef enum
{
    BLE_ATT_INVALID_UUID                                         = BLE_ATT_UUID_16(0x0000), /**< Invalid UUID. */
    /*----------------- SERVICES ---------------------*/
    BLE_ATT_SVC_GENERIC_ACCESS                                   = BLE_ATT_UUID_16(0x1800), /**< Generic Access Profile. */
    BLE_ATT_SVC_GENERIC_ATTRIBUTE                                = BLE_ATT_UUID_16(0x1801), /**< Attribute Profile. */
    BLE_ATT_SVC_IMMEDIATE_ALERT                                  = BLE_ATT_UUID_16(0x1802), /**< Immediate Alert Service. */
    BLE_ATT_SVC_LINK_LOSS                                        = BLE_ATT_UUID_16(0x1803), /**< Link Loss Service. */
    BLE_ATT_SVC_TX_POWER                                         = BLE_ATT_UUID_16(0x1804), /**< TX Power Service. */
    BLE_ATT_SVC_CURRENT_TIME                                     = BLE_ATT_UUID_16(0x1805), /**< Current Time Service. */
    BLE_ATT_SVC_REF_TIME_UPDATE                                  = BLE_ATT_UUID_16(0x1806), /**< Reference Time Update Service. */
    BLE_ATT_SVC_NEXT_DST_CHANGE                                  = BLE_ATT_UUID_16(0x1807), /**< Next DST Change Service. */
    BLE_ATT_SVC_GLUCOSE                                          = BLE_ATT_UUID_16(0x1808), /**< Glucose Service. */
    BLE_ATT_SVC_HEALTH_THERMOM                                   = BLE_ATT_UUID_16(0x1809), /**< Health Thermometer Service. */
    BLE_ATT_SVC_DEVICE_INFO                                      = BLE_ATT_UUID_16(0x180A), /**< Device Information Service. */
    BLE_ATT_SVC_HEART_RATE                                       = BLE_ATT_UUID_16(0x180D), /**< Heart Rate Service. */
    BLE_ATT_SVC_PHONE_ALERT_STATUS                               = BLE_ATT_UUID_16(0x180E), /**< Phone Alert Status Service. */
    BLE_ATT_SVC_BATTERY_SERVICE                                  = BLE_ATT_UUID_16(0x180F), /**< Battery Service. */
    BLE_ATT_SVC_BLOOD_PRESSURE                                   = BLE_ATT_UUID_16(0x1810), /**< Blood Pressure Service. */
    BLE_ATT_SVC_ALERT_NTF                                        = BLE_ATT_UUID_16(0x1811), /**< Alert Notification Service. */
    BLE_ATT_SVC_HID                                              = BLE_ATT_UUID_16(0x1812), /**< HID Service. */
    BLE_ATT_SVC_SCAN_PARAMETERS                                  = BLE_ATT_UUID_16(0x1813), /**< Scan Parameters Service. */
    BLE_ATT_SVC_RUNNING_SPEED_CADENCE                            = BLE_ATT_UUID_16(0x1814), /**< Running Speed and Cadence Service. */
    BLE_ATT_SVC_CYCLING_SPEED_CADENCE                            = BLE_ATT_UUID_16(0x1816), /**< Cycling Speed and Cadence Service. */
    BLE_ATT_SVC_CYCLING_POWER                                    = BLE_ATT_UUID_16(0x1818), /**< Cycling Power Service. */
    BLE_ATT_SVC_LOCATION_AND_NAVIGATION                          = BLE_ATT_UUID_16(0x1819), /**< Location and Navigation Service. */
    BLE_ATT_SVC_ENVIRONMENTAL_SENSING                            = BLE_ATT_UUID_16(0x181A), /**< Environmental Sensing Service. */
    BLE_ATT_SVC_BODY_COMPOSITION                                 = BLE_ATT_UUID_16(0x181B), /**< Body Composition Service. */
    BLE_ATT_SVC_USER_DATA                                        = BLE_ATT_UUID_16(0x181C), /**< User Data Service. */
    BLE_ATT_SVC_WEIGHT_SCALE                                     = BLE_ATT_UUID_16(0x181D), /**< Weight Scale Service. */
    BLE_ATT_SVC_BOND_MANAGEMENT                                  = BLE_ATT_UUID_16(0x181E), /**< Bond Management Service. */
    BLE_ATT_SVC_CONTINUOUS_GLUCOSE_MONITORING                    = BLE_ATT_UUID_16(0x181F), /**< Continuous Glucose Monitoring Service. */
    BLE_ATT_SVC_IP_SUPPORT                                       = BLE_ATT_UUID_16(0x1820), /**< Internet Protocol Support Service. */
    BLE_ATT_SVC_INDOOR_POSITIONING                               = BLE_ATT_UUID_16(0x1821), /**< Indoor Positioning Service. */
    BLE_ATT_SVC_PULSE_OXIMETER                                   = BLE_ATT_UUID_16(0x1822), /**< Pulse Oximeter Service. */
    BLE_ATT_SVC_HTTP_PROXY                                       = BLE_ATT_UUID_16(0x1823), /**< HTTP Proxy Service. */
    BLE_ATT_SVC_TRANSPORT_DISCOVERY                              = BLE_ATT_UUID_16(0x1824), /**< Transport Discovery Service. */
    BLE_ATT_SVC_OBJECT_TRANSFER                                  = BLE_ATT_UUID_16(0x1825), /**< Object Transfer Service. */
    BLE_ATT_SVC_CONSTANT_TONE_EXTENSION                          = BLE_ATT_UUID_16(0x1826), /**< Constant Tone Extention Service. */

    /*------------------- UNITS ---------------------*/
    BLE_ATT_UNIT_UNITLESS                                        = BLE_ATT_UUID_16(0x2700), /**< No defined unit. */
    BLE_ATT_UNIT_METRE                                           = BLE_ATT_UUID_16(0x2701), /**< Length unit: meter. */
    BLE_ATT_UNIT_KG                                              = BLE_ATT_UUID_16(0x2702), /**< Mass unit: kilogram. */
    BLE_ATT_UNIT_SECOND                                          = BLE_ATT_UUID_16(0x2703), /**< Time unit: second. */
    BLE_ATT_UNIT_AMPERE                                          = BLE_ATT_UUID_16(0x2704), /**< Electric current unit: ampere. */
    BLE_ATT_UNIT_KELVIN                                          = BLE_ATT_UUID_16(0x2705), /**< Thermodynamic Temperature unit: kelvin. */
    BLE_ATT_UNIT_MOLE                                            = BLE_ATT_UUID_16(0x2706), /**< Amount of substance unit: mole. */
    BLE_ATT_UNIT_CANDELA                                         = BLE_ATT_UUID_16(0x2707), /**< Luminous intensity unit: candela. */
    BLE_ATT_UNIT_SQ_METRE                                        = BLE_ATT_UUID_16(0x2710), /**< Area unit: square meter. */
    BLE_ATT_UNIT_CUBIC_METRE                                     = BLE_ATT_UUID_16(0x2710), /**< Column unit: cubic meter. */
    BLE_ATT_UNIT_METRE_PER_SECOND                                = BLE_ATT_UUID_16(0x2711), /**< Velocity unit: meter per second. */
    BLE_ATT_UNIT_METRES_PER_SEC_SQ                               = BLE_ATT_UUID_16(0x2712), /**< Acceleration unit: meter per second squared. */
    BLE_ATT_UNIT_RECIPROCAL_METRE                                = BLE_ATT_UUID_16(0x2713), /**< Wavenumber unit: reciprocal meter. */
    BLE_ATT_UNIT_DENS_KG_PER_CUBIC_METRE                         = BLE_ATT_UUID_16(0x2714), /**< Density unit: kilogram per cubic meter. */
    BLE_ATT_UNIT_KG_PER_SQ_METRE                                 = BLE_ATT_UUID_16(0x2715), /**< Surface density unit: kilogram per square meter. */
    BLE_ATT_UNIT_CUBIC_METRE_PER_KG                              = BLE_ATT_UUID_16(0x2716), /**< Specific volume unit: cubic meter per kilogram. */
    BLE_ATT_UNIT_AMPERE_PER_SQ_METRE                             = BLE_ATT_UUID_16(0x2717), /**< Current density unit: ampere per square meter. */
    BLE_ATT_UNIT_AMPERE_PER_METRE                                = BLE_ATT_UUID_16(0x2718), /**< Magnetic field strength unit: ampere per meter. */
    BLE_ATT_UNIT_MOLE_PER_CUBIC_METRE                            = BLE_ATT_UUID_16(0x2719), /**< Amount concentration unit: mole per cubic meter. */
    BLE_ATT_UNIT_MASS_KG_PER_CUBIC_METRE                         = BLE_ATT_UUID_16(0x271A), /**< Mass Concentration unit: kilogram per cubic meter. */
    BLE_ATT_UNIT_CANDELA_PER_SQ_METRE                            = BLE_ATT_UUID_16(0x271B), /**< Luminance unit: candela per square meter. */
    BLE_ATT_UNIT_REFRACTIVE_INDEX                                = BLE_ATT_UUID_16(0x271C), /**< Refractive index unit. */
    BLE_ATT_UNIT_RELATIVE_PERMEABILITY                           = BLE_ATT_UUID_16(0x271D), /**< Relative permeability unit. */
    BLE_ATT_UNIT_RADIAN                                          = BLE_ATT_UUID_16(0x2720), /**< Plane angle unit: radian. */
    BLE_ATT_UNIT_STERADIAN                                       = BLE_ATT_UUID_16(0x2721), /**< Solid angle unit: steradian. */
    BLE_ATT_UNIT_HERTZ                                           = BLE_ATT_UUID_16(0x2722), /**< Frequency unit: hertz. */
    BLE_ATT_UNIT_NEWTON                                          = BLE_ATT_UUID_16(0x2723), /**< Force unit: newton. */
    BLE_ATT_UNIT_PASCAL                                          = BLE_ATT_UUID_16(0x2724), /**< Pressure unit: pascal. */
    BLE_ATT_UNIT_JOULE                                           = BLE_ATT_UUID_16(0x2725), /**< Energy unit: joule. */
    BLE_ATT_UNIT_WATT                                            = BLE_ATT_UUID_16(0x2726), /**< Power unit: watt. */
    BLE_ATT_UNIT_COULOMB                                         = BLE_ATT_UUID_16(0x2727), /**< Electric Charge unit: coulomb. */
    BLE_ATT_UNIT_VOLT                                            = BLE_ATT_UUID_16(0x2728), /**< Electric potential difference unit: Volt. */
    BLE_ATT_UNIT_FARAD                                           = BLE_ATT_UUID_16(0x2729), /**< Capacitance unit: Farad. */
    BLE_ATT_UNIT_OHM                                             = BLE_ATT_UUID_16(0x272A), /**< Electric resistance unit: ohm. */
    BLE_ATT_UNIT_SIEMENS                                         = BLE_ATT_UUID_16(0x272B), /**< Electric conductance unit: siemens. */
    BLE_ATT_UNIT_WEBER                                           = BLE_ATT_UUID_16(0x272C), /**< Magnetic flux unit: weber. */
    BLE_ATT_UNIT_TESLA                                           = BLE_ATT_UUID_16(0x272D), /**< Magnetic flux density unit: Tesla. */
    BLE_ATT_UNIT_HENRY                                           = BLE_ATT_UUID_16(0x272E), /**< Inductance unit: henry. */
    BLE_ATT_UNIT_CELSIUS                                         = BLE_ATT_UUID_16(0x272F), /**< Temperature unit: degree Celsius. */
    BLE_ATT_UNIT_LUMEN                                           = BLE_ATT_UUID_16(0x2730), /**< Luminous flux unit: lumen. */
    BLE_ATT_UNIT_LUX                                             = BLE_ATT_UUID_16(0x2731), /**< Illuminance unit: lux. */
    BLE_ATT_UNIT_BECQUEREL                                       = BLE_ATT_UUID_16(0x2732), /**< Activity referred to a radionuclide unit: becquerel. */
    BLE_ATT_UNIT_GRAY                                            = BLE_ATT_UUID_16(0x2733), /**< Absorbed dose unit: gray. */
    BLE_ATT_UNIT_SIEVERT                                         = BLE_ATT_UUID_16(0x2734), /**< Dose equivalent unit: sievert. */
    BLE_ATT_UNIT_KATAL                                           = BLE_ATT_UUID_16(0x2735), /**< Catalytic activity unit: katal. */
    BLE_ATT_UNIT_PASCAL_SECOND                                   = BLE_ATT_UUID_16(0x2740), /**< Synamic viscosity unit: pascal second. */
    BLE_ATT_UNIT_NEWTON_METRE                                    = BLE_ATT_UUID_16(0x2741), /**< Moment of force unit: newton meter. */
    BLE_ATT_UNIT_NEWTON_PER_METRE                                = BLE_ATT_UUID_16(0x2742), /**< Surface tension unit: newton per meter. */
    BLE_ATT_UNIT_RADIAN_PER_SECOND                               = BLE_ATT_UUID_16(0x2743), /**< Angular velocity unit: radian per second. */
    BLE_ATT_UNIT_RADIAN_PER_SECOND_SQ                            = BLE_ATT_UUID_16(0x2744), /**< Angular acceleration unit: radian per second squared. */
    BLE_ATT_UNIT_WATT_PER_SQ_METRE                               = BLE_ATT_UUID_16(0x2745), /**< Heat flux density unit: watt per square meter. */
    BLE_ATT_UNIT_JOULE_PER_KELVIN                                = BLE_ATT_UUID_16(0x2746), /**< Heat capacity unit: joule per Kelvin. */
    BLE_ATT_UNIT_JOULE_PER_KG_KELVIN                             = BLE_ATT_UUID_16(0x2747), /**< Specific heat capacity unit: joule per kilogram kelvin. */
    BLE_ATT_UNIT_JOULE_PER_KG                                    = BLE_ATT_UUID_16(0x2748), /**< Specific Energy unit: joule per kilogram. */
    BLE_ATT_UNIT_WATT_PER_METRE_KELVIN                           = BLE_ATT_UUID_16(0x2749), /**< Thermal conductivity unit: watt per meter Kelvin. */
    BLE_ATT_UNIT_JOULE_PER_CUBIC_METRE                           = BLE_ATT_UUID_16(0x274A), /**< Energy Density unit: joule per cubic meter. */
    BLE_ATT_UNIT_VOLT_PER_METRE                                  = BLE_ATT_UUID_16(0x274B), /**< Electric field strength unit: volt per meter. */
    BLE_ATT_UNIT_COULOMB_PER_CUBIC_METRE                         = BLE_ATT_UUID_16(0x274C), /**< Electric charge density unit: coulomb per cubic meter. */
    BLE_ATT_UNIT_SURF_COULOMB_PER_SQ_METRE                       = BLE_ATT_UUID_16(0x274D), /**< Surface charge density unit: coulomb per square meter. */
    BLE_ATT_UNIT_FLUX_COULOMB_PER_SQ_METRE                       = BLE_ATT_UUID_16(0x274E), /**< Electric flux density unit: coulomb per square meter. */
    BLE_ATT_UNIT_FARAD_PER_METRE                                 = BLE_ATT_UUID_16(0x274F), /**< Permittivity unit: farad per meter. */
    BLE_ATT_UNIT_HENRY_PER_METRE                                 = BLE_ATT_UUID_16(0x2750), /**< Permeability unit: henry per meter. */
    BLE_ATT_UNIT_JOULE_PER_MOLE                                  = BLE_ATT_UUID_16(0x2751), /**< Molar energy unit: joule per mole. */
    BLE_ATT_UNIT_JOULE_PER_MOLE_KELVIN                           = BLE_ATT_UUID_16(0x2752), /**< Molar entropy unit: joule per mole kelvin. */
    BLE_ATT_UNIT_COULOMB_PER_KG                                  = BLE_ATT_UUID_16(0x2753), /**< Exposure unit: coulomb per kilogram. */
    BLE_ATT_UNIT_GRAY_PER_SECOND                                 = BLE_ATT_UUID_16(0x2754), /**< Absorbed dose rate unit: gray per second. */
    BLE_ATT_UNIT_WATT_PER_STERADIAN                              = BLE_ATT_UUID_16(0x2755), /**< Radiant intensity unit: watt per steradian. */
    BLE_ATT_UNIT_WATT_PER_SQ_METRE_STERADIAN                     = BLE_ATT_UUID_16(0x2756), /**< Radiance unit: watt per square meter steradian. */
    BLE_ATT_UNIT_KATAL_PER_CUBIC_METRE                           = BLE_ATT_UUID_16(0x2757), /**< Catalytic activity concentration unit: katal per cubic meter. */
    BLE_ATT_UNIT_MINUTE                                          = BLE_ATT_UUID_16(0x2760), /**< Time unit: minute. */
    BLE_ATT_UNIT_HOUR                                            = BLE_ATT_UUID_16(0x2761), /**< Time unit: hour. */
    BLE_ATT_UNIT_DAY                                             = BLE_ATT_UUID_16(0x2762), /**< Time unit: day. */
    BLE_ATT_UNIT_ANGLE_DEGREE                                    = BLE_ATT_UUID_16(0x2763), /**< Plane angle unit: degree. */
    BLE_ATT_UNIT_ANGLE_MINUTE                                    = BLE_ATT_UUID_16(0x2764), /**< Plane angle unit: minute. */
    BLE_ATT_UNIT_ANGLE_SECOND                                    = BLE_ATT_UUID_16(0x2765), /**< Plane angle unit: second. */
    BLE_ATT_UNIT_HECTARE                                         = BLE_ATT_UUID_16(0x2766), /**< Area unit: hectare. */
    BLE_ATT_UNIT_LITRE                                           = BLE_ATT_UUID_16(0x2767), /**< Volume unit: litre. */
    BLE_ATT_UNIT_TONNE                                           = BLE_ATT_UUID_16(0x2768), /**< Mass unit: tonne. */
    BLE_ATT_UNIT_BAR                                             = BLE_ATT_UUID_16(0x2780), /**< Pressure unit: bar. */
    BLE_ATT_UNIT_MM_MERCURY                                      = BLE_ATT_UUID_16(0x2781), /**< Pressure unit: millimetre of mercury. */
    BLE_ATT_UNIT_ANGSTROM                                        = BLE_ATT_UUID_16(0x2782), /**< Length unit: angstrom. */
    BLE_ATT_UNIT_NAUTICAL_MILE                                   = BLE_ATT_UUID_16(0x2783), /**< Length unit: nautical mile. */
    BLE_ATT_UNIT_BARN                                            = BLE_ATT_UUID_16(0x2784), /**< Area unit: barn. */
    BLE_ATT_UNIT_KNOT                                            = BLE_ATT_UUID_16(0x2785), /**< Velocity unit: knot. */
    BLE_ATT_UNIT_NEPER                                           = BLE_ATT_UUID_16(0x2786), /**< Logarithmic radio quantity unit: neper. */
    BLE_ATT_UNIT_BEL                                             = BLE_ATT_UUID_16(0x2787), /**< Logarithmic radio quantity unit: bel. */
    BLE_ATT_UNIT_YARD                                            = BLE_ATT_UUID_16(0x27A0), /**< Length unit: yard. */
    BLE_ATT_UNIT_PARSEC                                          = BLE_ATT_UUID_16(0x27A1), /**< Length unit: parsec. */
    BLE_ATT_UNIT_INCH                                            = BLE_ATT_UUID_16(0x27A2), /**< Length unit: inch. */
    BLE_ATT_UNIT_FOOT                                            = BLE_ATT_UUID_16(0x27A3), /**< Length unit: foot. */
    BLE_ATT_UNIT_MILE                                            = BLE_ATT_UUID_16(0x27A4), /**< Length unit: mile. */
    BLE_ATT_UNIT_POUND_FORCE_PER_SQ_INCH                         = BLE_ATT_UUID_16(0x27A5), /**< Pressure unit: pound-force per square inch. */
    BLE_ATT_UNIT_KM_PER_HOUR                                     = BLE_ATT_UUID_16(0x27A6), /**< Velocity unit: kilometre per hour. */
    BLE_ATT_UNIT_MILE_PER_HOUR                                   = BLE_ATT_UUID_16(0x27A7), /**< Velocity unit: mile per hour. */
    BLE_ATT_UNIT_REVOLUTION_PER_MINUTE                           = BLE_ATT_UUID_16(0x27A8), /**< Angular velocity unit: revolution per minute. */
    BLE_ATT_UNIT_GRAM_CALORIE                                    = BLE_ATT_UUID_16(0x27A9), /**< Energy unit: gram calorie. */
    BLE_ATT_UNIT_KG_CALORIE                                      = BLE_ATT_UUID_16(0x27AA), /**< Energy unit: kilogram calorie. */
    BLE_ATT_UNIT_KILOWATT_HOUR                                   = BLE_ATT_UUID_16(0x27AB), /**< Energy unit: kilowatt hour. */
    BLE_ATT_UNIT_FAHRENHEIT                                      = BLE_ATT_UUID_16(0x27AC), /**< Thermodynamic temperature unit: degree Fahrenheit. */
    BLE_ATT_UNIT_PERCENTAGE                                      = BLE_ATT_UUID_16(0x27AD), /**< Unit: Percentage. */
    BLE_ATT_UNIT_PER_MILLE                                       = BLE_ATT_UUID_16(0x27AE), /**< Unit: per mille. */
    BLE_ATT_UNIT_BEATS_PER_MINUTE                                = BLE_ATT_UUID_16(0x27AF), /**< Period unit: beats per minute. */
    BLE_ATT_UNIT_AMPERE_HOURS                                    = BLE_ATT_UUID_16(0x27B0), /**< Electric charge unit: ampere hours. */
    BLE_ATT_UNIT_MILLIGRAM_PER_DECILITRE                         = BLE_ATT_UUID_16(0x27B1), /**< Mass density unit: milligram per decilitre. */
    BLE_ATT_UNIT_MILLIMOLE_PER_LITRE                             = BLE_ATT_UUID_16(0x27B2), /**< Mass density unit: millimole per litre. */
    BLE_ATT_UNIT_YEAR                                            = BLE_ATT_UUID_16(0x27B3), /**< Time unit: year. */
    BLE_ATT_UNIT_MONTH                                           = BLE_ATT_UUID_16(0x27B4), /**< Time unit: month. */

    /*---------------- DECLARATIONS -----------------*/
    BLE_ATT_DECL_PRIMARY_SERVICE                                 = BLE_ATT_UUID_16(0x2800), /**< Primary service Declaration. */
    BLE_ATT_DECL_SECONDARY_SERVICE                               = BLE_ATT_UUID_16(0x2801), /**< Secondary service Declaration. */
    BLE_ATT_DECL_INCLUDE                                         = BLE_ATT_UUID_16(0x2802), /**< Include Declaration. */
    BLE_ATT_DECL_CHARACTERISTIC                                  = BLE_ATT_UUID_16(0x2803), /**< Characteristic Declaration. */

    /*----------------- DESCRIPTORS -----------------*/                        
    BLE_ATT_DESC_CHAR_EXT_PROPERTIES                             = BLE_ATT_UUID_16(0x2900), /**< Characteristic extended properties. */
    BLE_ATT_DESC_CHAR_USER_DESCRIPTION                           = BLE_ATT_UUID_16(0x2901), /**< Characteristic user description. */
    BLE_ATT_DESC_CLIENT_CHAR_CFG                                 = BLE_ATT_UUID_16(0x2902), /**< Client characteristic configuration. */
    BLE_ATT_DESC_SERVER_CHAR_CFG                                 = BLE_ATT_UUID_16(0x2903), /**< Server characteristic configuration. */
    BLE_ATT_DESC_CHAR_PRES_FORMAT                                = BLE_ATT_UUID_16(0x2904), /**< Characteristic Presentation Format. */
    BLE_ATT_DESC_CHAR_AGGREGATE_FORMAT                           = BLE_ATT_UUID_16(0x2905), /**< Characteristic Aggregate Format. */
    BLE_ATT_DESC_VALID_RANGE                                     = BLE_ATT_UUID_16(0x2906), /**< Valid Range. */
    BLE_ATT_DESC_EXT_REPORT_REF                                  = BLE_ATT_UUID_16(0x2907), /**< External Report Reference. */
    BLE_ATT_DESC_REPORT_REF                                      = BLE_ATT_UUID_16(0x2908), /**< Report Reference. */
    BLE_ATT_DESC_ES_CONFIGURATION                                = BLE_ATT_UUID_16(0x290B), /**< Environmental Sensing Configuration. */
    BLE_ATT_DESC_ES_MEASUREMENT                                  = BLE_ATT_UUID_16(0x290C), /**< Environmental Sensing Measurement. */
    BLE_ATT_DESC_ES_TRIGGER_SETTING                              = BLE_ATT_UUID_16(0x290D), /**< Environmental Sensing Trigger Setting. */

    /*--------------- CHARACTERISTICS ---------------*/
    BLE_ATT_CHAR_DEVICE_NAME                                     = BLE_ATT_UUID_16(0x2A00), /**< Device name. */
    BLE_ATT_CHAR_APPEARANCE                                      = BLE_ATT_UUID_16(0x2A01), /**< Appearance. */
    BLE_ATT_CHAR_PRIVACY_FLAG                                    = BLE_ATT_UUID_16(0x2A02), /**< Privacy flag. */
    BLE_ATT_CHAR_RECONNECTION_ADDR                               = BLE_ATT_UUID_16(0x2A03), /**< Reconnection address. */
    BLE_ATT_CHAR_PERIPH_PREF_CON_PARAM                           = BLE_ATT_UUID_16(0x2A04), /**< Peripheral preferred connection parameters. */
    BLE_ATT_CHAR_SERVICE_CHANGED                                 = BLE_ATT_UUID_16(0x2A05), /**< Service handles changed. */
    BLE_ATT_CHAR_ALERT_LEVEL                                     = BLE_ATT_UUID_16(0x2A06), /**< Alert Level characteristic. */
    BLE_ATT_CHAR_TX_POWER_LEVEL                                  = BLE_ATT_UUID_16(0x2A07), /**< Tx Power Level. */
    BLE_ATT_CHAR_DATE_TIME                                       = BLE_ATT_UUID_16(0x2A08), /**< Date Time. */
    BLE_ATT_CHAR_DAY_WEEK                                        = BLE_ATT_UUID_16(0x2A09), /**< Day of Week. */
    BLE_ATT_CHAR_DAY_DATE_TIME                                   = BLE_ATT_UUID_16(0x2A0A), /**< Day Date Time. */
    BLE_ATT_CHAR_EXACT_TIME_256                                  = BLE_ATT_UUID_16(0x2A0C), /**< Exact time 256. */
    BLE_ATT_CHAR_DST_OFFSET                                      = BLE_ATT_UUID_16(0x2A0D), /**< DST Offset. */
    BLE_ATT_CHAR_TIME_ZONE                                       = BLE_ATT_UUID_16(0x2A0E), /**< Time zone. */
    BLE_ATT_CHAR_LOCAL_TIME_INFO                                 = BLE_ATT_UUID_16(0x2A0F), /**< Local time Information. */
    BLE_ATT_CHAR_TIME_WITH_DST                                   = BLE_ATT_UUID_16(0x2A11), /**< Time with DST. */
    BLE_ATT_CHAR_TIME_ACCURACY                                   = BLE_ATT_UUID_16(0x2A12), /**< Time Accuracy. */
    BLE_ATT_CHAR_TIME_SOURCE                                     = BLE_ATT_UUID_16(0x2A13), /**< Time Source. */
    BLE_ATT_CHAR_REFERENCE_TIME_INFO                             = BLE_ATT_UUID_16(0x2A14), /**< Reference Time Information. */
    BLE_ATT_CHAR_TIME_UPDATE_CNTL_POINT                          = BLE_ATT_UUID_16(0x2A16), /**< Time Update Control Point. */
    BLE_ATT_CHAR_TIME_UPDATE_STATE                               = BLE_ATT_UUID_16(0x2A17), /**< Time Update State. */
    BLE_ATT_CHAR_GLUCOSE_MEAS                                    = BLE_ATT_UUID_16(0x2A18), /**< Glucose Measurement. */
    BLE_ATT_CHAR_BATTERY_LEVEL                                   = BLE_ATT_UUID_16(0x2A19), /**< Battery Level. */
    BLE_ATT_CHAR_TEMPERATURE_MEAS                                = BLE_ATT_UUID_16(0x2A1C), /**< Temperature Measurement. */
    BLE_ATT_CHAR_TEMPERATURE_TYPE                                = BLE_ATT_UUID_16(0x2A1D), /**< Temperature Type. */
    BLE_ATT_CHAR_INTERMED_TEMPERATURE                            = BLE_ATT_UUID_16(0x2A1E), /**< Intermediate Temperature. */
    BLE_ATT_CHAR_MEAS_INTERVAL                                   = BLE_ATT_UUID_16(0x2A21), /**< Measurement Interval. */
    BLE_ATT_CHAR_BOOT_KB_IN_REPORT                               = BLE_ATT_UUID_16(0x2A22), /**< Boot Keyboard Input Report. */
    BLE_ATT_CHAR_SYS_ID                                          = BLE_ATT_UUID_16(0x2A23), /**< System ID. */
    BLE_ATT_CHAR_MODEL_NB                                        = BLE_ATT_UUID_16(0x2A24), /**< Model Number String. */
    BLE_ATT_CHAR_SERIAL_NB                                       = BLE_ATT_UUID_16(0x2A25), /**< Serial Number String. */
    BLE_ATT_CHAR_FW_REV                                          = BLE_ATT_UUID_16(0x2A26), /**< Firmware Revision String. */
    BLE_ATT_CHAR_HW_REV                                          = BLE_ATT_UUID_16(0x2A27), /**< Hardware revision String. */
    BLE_ATT_CHAR_SW_REV                                          = BLE_ATT_UUID_16(0x2A28), /**< Software Revision String. */
    BLE_ATT_CHAR_MANUF_NAME                                      = BLE_ATT_UUID_16(0x2A29), /**< Manufacturer Name String. */
    BLE_ATT_CHAR_IEEE_CERTIF                                     = BLE_ATT_UUID_16(0x2A2A), /**< IEEE Regulatory Certification Data List. */
    BLE_ATT_CHAR_CT_TIME                                         = BLE_ATT_UUID_16(0x2A2B), /**< CT Time. */
    BLE_ATT_CHAR_MAGN_DECLINE                                    = BLE_ATT_UUID_16(0x2A2C), /**< Magnetic Declination. */
    BLE_ATT_CHAR_SCAN_REFRESH                                    = BLE_ATT_UUID_16(0x2A31), /**< Scan Refresh. */
    BLE_ATT_CHAR_BOOT_KB_OUT_REPORT                              = BLE_ATT_UUID_16(0x2A32), /**< Boot Keyboard Output Report. */
    BLE_ATT_CHAR_BOOT_MOUSE_IN_REPORT                            = BLE_ATT_UUID_16(0x2A33), /**< Boot Mouse Input Report. */
    BLE_ATT_CHAR_GLUCOSE_MEAS_CTX                                = BLE_ATT_UUID_16(0x2A34), /**< Glucose Measurement Context. */
    BLE_ATT_CHAR_BLOOD_PRESSURE_MEAS                             = BLE_ATT_UUID_16(0x2A35), /**< Blood Pressure Measurement. */
    BLE_ATT_CHAR_INTERMEDIATE_CUFF_PRESSURE                      = BLE_ATT_UUID_16(0x2A36), /**< Intermediate Cuff Pressure. */
    BLE_ATT_CHAR_HEART_RATE_MEAS                                 = BLE_ATT_UUID_16(0x2A37), /**< Heart Rate Measurement. */
    BLE_ATT_CHAR_BODY_SENSOR_LOCATION                            = BLE_ATT_UUID_16(0x2A38), /**< Body Sensor Location. */
    BLE_ATT_CHAR_HEART_RATE_CNTL_POINT                           = BLE_ATT_UUID_16(0x2A39), /**< Heart Rate Control Point. */
    BLE_ATT_CHAR_ALERT_STATUS                                    = BLE_ATT_UUID_16(0x2A3F), /**< Alert Status. */
    BLE_ATT_CHAR_RINGER_CNTL_POINT                               = BLE_ATT_UUID_16(0x2A40), /**< Ringer Control Point. */
    BLE_ATT_CHAR_RINGER_SETTING                                  = BLE_ATT_UUID_16(0x2A41), /**< Ringer Setting. */
    BLE_ATT_CHAR_ALERT_CAT_ID_BIT_MASK                           = BLE_ATT_UUID_16(0x2A42), /**< Alert Category ID Bit Mask. */
    BLE_ATT_CHAR_ALERT_CAT_ID                                    = BLE_ATT_UUID_16(0x2A43), /**< Alert Category ID. */
    BLE_ATT_CHAR_ALERT_NTF_CTNL_PT                               = BLE_ATT_UUID_16(0x2A44), /**< Alert Notification Control Point. */
    BLE_ATT_CHAR_UNREAD_ALERT_STATUS                             = BLE_ATT_UUID_16(0x2A45), /**< Unread Alert Status. */
    BLE_ATT_CHAR_NEW_ALERT                                       = BLE_ATT_UUID_16(0x2A46), /**< New Alert. */
    BLE_ATT_CHAR_SUP_NEW_ALERT_CAT                               = BLE_ATT_UUID_16(0x2A47), /**< Supported New Alert Category. */
    BLE_ATT_CHAR_SUP_UNREAD_ALERT_CAT                            = BLE_ATT_UUID_16(0x2A48), /**< Supported Unread Alert Category. */
    BLE_ATT_CHAR_BLOOD_PRESSURE_FEATURE                          = BLE_ATT_UUID_16(0x2A49), /**< Blood Pressure Feature. */
    BLE_ATT_CHAR_HID_INFO                                        = BLE_ATT_UUID_16(0x2A4A), /**< HID Information. */
    BLE_ATT_CHAR_REPORT_MAP                                      = BLE_ATT_UUID_16(0x2A4B), /**< Report Map. */
    BLE_ATT_CHAR_HID_CTNL_PT                                     = BLE_ATT_UUID_16(0x2A4C), /**< HID Control Point. */
    BLE_ATT_CHAR_REPORT                                          = BLE_ATT_UUID_16(0x2A4D), /**< Report. */
    BLE_ATT_CHAR_PROTOCOL_MODE                                   = BLE_ATT_UUID_16(0x2A4E), /**< Protocol Mode. */
    BLE_ATT_CHAR_SCAN_INTV_WD                                    = BLE_ATT_UUID_16(0x2A4F), /**< Scan Interval Window. */
    BLE_ATT_CHAR_PNP_ID                                          = BLE_ATT_UUID_16(0x2A50), /**< PnP ID. */
    BLE_ATT_CHAR_GLUCOSE_FEATURE                                 = BLE_ATT_UUID_16(0x2A51), /**< Glucose Feature. */
    BLE_ATT_CHAR_REC_ACCESS_CTRL_PT                              = BLE_ATT_UUID_16(0x2A52), /**< Record access control point. */
    BLE_ATT_CHAR_RSC_MEAS                                        = BLE_ATT_UUID_16(0x2A53), /**< RSC Measurement. */
    BLE_ATT_CHAR_RSC_FEAT                                        = BLE_ATT_UUID_16(0x2A54), /**< RSC Feature. */
    BLE_ATT_CHAR_SC_CNTL_PT                                      = BLE_ATT_UUID_16(0x2A55), /**< SC Control Point. */
    BLE_ATT_CHAR_CSC_MEAS                                        = BLE_ATT_UUID_16(0x2A5B), /**< CSC Measurement. */
    BLE_ATT_CHAR_CSC_FEAT                                        = BLE_ATT_UUID_16(0x2A5C), /**< CSC Feature. */
    BLE_ATT_CHAR_SENSOR_LOC                                      = BLE_ATT_UUID_16(0x2A5D), /**< Sensor Location. */
    BLE_ATT_CHAR_PLX_SPOT_CHECK_MEASUREMENT_LOC                  = BLE_ATT_UUID_16(0x2A5E), /**< PLX Spot-Check Measurement. */
    BLE_ATT_CHAR_PLX_CONTINUOUS_MEASUREMENT_LOC                  = BLE_ATT_UUID_16(0x2A5F), /**< PLX Continuous Measurement. */
    BLE_ATT_CHAR_PLX_FEATURES_LOC                                = BLE_ATT_UUID_16(0x2A60), /**< PLX Features. */
    BLE_ATT_CHAR_CP_MEAS                                         = BLE_ATT_UUID_16(0x2A63), /**< CP Measurement. */
    BLE_ATT_CHAR_CP_VECTOR                                       = BLE_ATT_UUID_16(0x2A64), /**< CP Vector. */
    BLE_ATT_CHAR_CP_FEAT                                         = BLE_ATT_UUID_16(0x2A65), /**< CP Feature. */
    BLE_ATT_CHAR_CP_CNTL_PT                                      = BLE_ATT_UUID_16(0x2A66), /**< CP Control Point. */
    BLE_ATT_CHAR_LOC_SPEED                                       = BLE_ATT_UUID_16(0x2A67), /**< Location and Speed. */
    BLE_ATT_CHAR_NAVIGATION                                      = BLE_ATT_UUID_16(0x2A68), /**< Navigation. */
    BLE_ATT_CHAR_POS_QUALITY                                     = BLE_ATT_UUID_16(0x2A69), /**< Position Quality. */
    BLE_ATT_CHAR_LN_FEAT                                         = BLE_ATT_UUID_16(0x2A6A), /**< LN Feature. */
    BLE_ATT_CHAR_LN_CNTL_PT                                      = BLE_ATT_UUID_16(0x2A6B), /**< LN Control Point. */
    BLE_ATT_CHAR_ELEVATION                                       = BLE_ATT_UUID_16(0x2A6C), /**< Elevation. */
    BLE_ATT_CHAR_PRESSURE                                        = BLE_ATT_UUID_16(0x2A6D), /**< Pressure. */
    BLE_ATT_CHAR_TEMPERATURE                                     = BLE_ATT_UUID_16(0x2A6E), /**< Temperature. */
    BLE_ATT_CHAR_HUMIDITY                                        = BLE_ATT_UUID_16(0x2A6F), /**< Humidity. */
    BLE_ATT_CHAR_TRUE_WIND_SPEED                                 = BLE_ATT_UUID_16(0x2A70), /**< True Wind Speed. */
    BLE_ATT_CHAR_TRUE_WIND_DIR                                   = BLE_ATT_UUID_16(0x2A71), /**< True Wind Direction. */
    BLE_ATT_CHAR_APRNT_WIND_SPEED                                = BLE_ATT_UUID_16(0x2A72), /**< Apparent Wind Speed. */
    BLE_ATT_CHAR_APRNT_WIND_DIRECTION                            = BLE_ATT_UUID_16(0x2A73), /**< Apparent Wind Direction. */
    BLE_ATT_CHAR_GUST_FACTOR                                     = BLE_ATT_UUID_16(0x2A74), /**< Gust Factor. */
    BLE_ATT_CHAR_POLLEN_CONC                                     = BLE_ATT_UUID_16(0x2A75), /**< Pollen Concentration. */
    BLE_ATT_CHAR_UV_INDEX                                        = BLE_ATT_UUID_16(0x2A76), /**< UV Index. */
    BLE_ATT_CHAR_IRRADIANCE                                      = BLE_ATT_UUID_16(0x2A77), /**< Irradiance. */
    BLE_ATT_CHAR_RAINFALL                                        = BLE_ATT_UUID_16(0x2A78), /**< Rainfall. */
    BLE_ATT_CHAR_WIND_CHILL                                      = BLE_ATT_UUID_16(0x2A79), /**< Wind Chill. */
    BLE_ATT_CHAR_HEAT_INDEX                                      = BLE_ATT_UUID_16(0x2A7A), /**< Heat Index. */
    BLE_ATT_CHAR_DEW_POINT                                       = BLE_ATT_UUID_16(0x2A7B), /**< Dew Point. */
    BLE_ATT_CHAR_DESCRIPTOR_VALUE_CHANGED                        = BLE_ATT_UUID_16(0x2A7D), /**< Descriptor Value Changed. */
    BLE_ATT_CHAR_AEROBIC_HEART_RATE_LOWER_LIMIT                  = BLE_ATT_UUID_16(0x2A7E), /**< Aerobic Heart Rate Lower Limit. */
    BLE_ATT_CHAR_AEROBIC_THRESHOLD                               = BLE_ATT_UUID_16(0x2A7F), /**< Aerobic Threshold. */
    BLE_ATT_CHAR_AGE                                             = BLE_ATT_UUID_16(0x2A80), /**< Age. */
    BLE_ATT_CHAR_ANAEROBIC_HEART_RATE_LOWER_LIMIT                = BLE_ATT_UUID_16(0x2A81), /**< Anaerobic Heart Rate Lower Limit. */
    BLE_ATT_CHAR_ANAEROBIC_HEART_RATE_UPPER_LIMIT                = BLE_ATT_UUID_16(0x2A82), /**< Anaerobic Heart Rate Upper Limit. */
    BLE_ATT_CHAR_ANAEROBIC_THRESHHOLD                            = BLE_ATT_UUID_16(0x2A83), /**< Anaerobic Threshhold. */
    BLE_ATT_CHAR_AEROBIC_HEART_RATE_UPPER_LIMIT                  = BLE_ATT_UUID_16(0x2A84), /**< Aerobic Heart Rate Upper Limit. */
    BLE_ATT_CHAR_DATE_OF_BIRTH                                   = BLE_ATT_UUID_16(0x2A85), /**< Date of Birth. */
    BLE_ATT_CHAR_DATE_OF_THRESHOLD_ASSESSMENT                    = BLE_ATT_UUID_16(0x2A86), /**< Date of Threshold Assessment. */
    BLE_ATT_CHAR_EMAIL_ADDRESS                                   = BLE_ATT_UUID_16(0x2A87), /**< Email Address. */
    BLE_ATT_CHAR_FAT_BURN_HEART_RATE_LOWER_LIMIT                 = BLE_ATT_UUID_16(0x2A88), /**< Fat Burn Heart Rate Lower Limit. */
    BLE_ATT_CHAR_FAT_BURN_HEART_RATE_UPPER_LIMIT                 = BLE_ATT_UUID_16(0x2A89), /**< Fat Burn Heart Rate Upper Limit. */
    BLE_ATT_CHAR_FIRST_NAME                                      = BLE_ATT_UUID_16(0x2A8A), /**< First Name. */
    BLE_ATT_CHAR_FIVE_ZONE_HEART_RATE_LIMITS                     = BLE_ATT_UUID_16(0x2A8B), /**< Five Zone Heart Rate Limits. */
    BLE_ATT_CHAR_GENDER                                          = BLE_ATT_UUID_16(0x2A8C), /**< Gender. */
    BLE_ATT_CHAR_MAX_HEART_RATE                                  = BLE_ATT_UUID_16(0x2A8D), /**< Max Heart Rate. */
    BLE_ATT_CHAR_HEIGHT                                          = BLE_ATT_UUID_16(0x2A8E), /**< Height. */
    BLE_ATT_CHAR_HIP_CIRCUMFERENCE                               = BLE_ATT_UUID_16(0x2A8F), /**< Hip Circumference. */
    BLE_ATT_CHAR_LAST_NAME                                       = BLE_ATT_UUID_16(0x2A90), /**< Last Name. */
    BLE_ATT_CHAR_MAXIMUM_RECOMMENDED_HEART_RATE                  = BLE_ATT_UUID_16(0x2A91), /**< Maximum Recommended Heart Rate. */
    BLE_ATT_CHAR_RESTING_HEART_RATE                              = BLE_ATT_UUID_16(0x2A92), /**< Resting Heart Rate. */
    BLE_ATT_CHAR_SPORT_TYPE_FOR_AEROBIC_AND_ANAEROBIC_THRESHOLDS = BLE_ATT_UUID_16(0x2A93), /**< Sport Type For Aerobic And Anaerobic Thresholds. */
    BLE_ATT_CHAR_THREE_ZONE_HEART_RATE_LIMITS                    = BLE_ATT_UUID_16(0x2A94), /**< Three Zone Heart Rate Limits. */
    BLE_ATT_CHAR_TWO_ZONE_HEART_RATE_LIMIT                       = BLE_ATT_UUID_16(0x2A95), /**< Two Zone Heart Rate Limits. */
    BLE_ATT_CHAR_VO2_MAX                                         = BLE_ATT_UUID_16(0x2A96), /**< Vo2 Max. */
    BLE_ATT_CHAR_WAIST_CIRCUMFERENCE                             = BLE_ATT_UUID_16(0x2A97), /**< Waist Circumference. */
    BLE_ATT_CHAR_WEIGHT                                          = BLE_ATT_UUID_16(0x2A98), /**< Weight. */
    BLE_ATT_CHAR_DATABASE_CHANGE_INCREMENT                       = BLE_ATT_UUID_16(0x2A99), /**< Database Change Increment. */
    BLE_ATT_CHAR_USER_INDEX                                      = BLE_ATT_UUID_16(0x2A9A), /**< User Index. */
    BLE_ATT_CHAR_BODY_COMPOSITION_FEATURE                        = BLE_ATT_UUID_16(0x2A9B), /**< Body Composition Feature. */
    BLE_ATT_CHAR_BODY_COMPOSITION_MEASUREMENT                    = BLE_ATT_UUID_16(0x2A9C), /**< Body Composition Measurement. */
    BLE_ATT_CHAR_WEIGHT_MEASUREMENT                              = BLE_ATT_UUID_16(0x2A9D), /**< Weight Measurement. */
    BLE_ATT_CHAR_WEIGHT_SCALE_FEATURE                            = BLE_ATT_UUID_16(0x2A9E), /**< Weight Scale Feature. */
    BLE_ATT_CHAR_USER_CONTROL_POINT                              = BLE_ATT_UUID_16(0x2A9F), /**< User Control Point. */
    BLE_ATT_CHAR_MAGN_FLUX_2D                                    = BLE_ATT_UUID_16(0x2AA0), /**< Flux Density - 2D. */
    BLE_ATT_CHAR_MAGN_FLUX_3D                                    = BLE_ATT_UUID_16(0x2AA1), /**< Magnetic Flux Density - 3D. */
    BLE_ATT_CHAR_LANGUAGE                                        = BLE_ATT_UUID_16(0x2AA2), /**< Language string. */
    BLE_ATT_CHAR_BAR_PRES_TREND                                  = BLE_ATT_UUID_16(0x2AA3), /**< Barometric Pressure Trend. */
    BLE_ATT_CHAR_CTL_ADDR_RESOL_SUPP                             = BLE_ATT_UUID_16(0x2AA6), /**< Central Address Resolution Support. */
    BLE_ATT_CHAR_OTS_FEATURES                                    = BLE_ATT_UUID_16(0x2ABD), /**< OTS Service Feature. */
    BLE_ATT_CHAR_OTS_OBJECT_NAME                                 = BLE_ATT_UUID_16(0x2ABE), /**< Object Name. */
    BLE_ATT_CHAR_OTS_OBJECT_TYPE                                 = BLE_ATT_UUID_16(0x2ABF), /**< Object Type. */
    BLE_ATT_CHAR_OTS_OBJECT_SIZE                                 = BLE_ATT_UUID_16(0x2AC0), /**< Object Size. */
    BLE_ATT_CHAR_OTS_OBJECT_FIRST_CREATED                        = BLE_ATT_UUID_16(0x2AC1), /**< Object First Created. */
    BLE_ATT_CHAR_OTS_OBJECT_LAST_MODIFIED                        = BLE_ATT_UUID_16(0x2AC2), /**< Object Last Modified. */
    BLE_ATT_CHAR_OTS_OBJECT_ID                                   = BLE_ATT_UUID_16(0x2AC3), /**< Object ID. */
    BLE_ATT_CHAR_OTS_OBJECT_PROPERTIES                           = BLE_ATT_UUID_16(0x2AC4), /**< Object Properties. */
    BLE_ATT_CHAR_OTS_OACP                                        = BLE_ATT_UUID_16(0x2AC5), /**< Object Action Control Point. */
    BLE_ATT_CHAR_OTS_OLCP                                        = BLE_ATT_UUID_16(0x2AC6), /**< Object List Control Point. */
    BLE_ATT_CHAR_OTS_LF                                          = BLE_ATT_UUID_16(0x2AC7), /**< Object List Filter. */
    BLE_ATT_CHAR_OTS_OBJECT_CHANGED                              = BLE_ATT_UUID_16(0x2AC8), /**< Object Changed.  */
    BLE_ATT_CHAR_RSLV_PRIV_ADDR_ONLY                             = BLE_ATT_UUID_16(0x2AC9), /**< Resolvable Private Address only. */
    BLE_ATT_CHAR_CTE_ENABLE                                      = BLE_ATT_UUID_16(0x2ACA), /**< Constant Tone Extension Enable. */
    BLE_ATT_CHAR_CTE_ADV_MIN_LEN                                 = BLE_ATT_UUID_16(0x2ACB), /**< Advertising Constant Tone Extension Minimum Length. */
    BLE_ATT_CHAR_CTE_ADV_MIN_TRANS_CNT                           = BLE_ATT_UUID_16(0x2ACC), /**< Advertising Constant Tone Extension Minimum Transmit Count. */
    BLE_ATT_CHAR_CTE_ADV_TRANS_DUR                               = BLE_ATT_UUID_16(0x2ACD), /**< Advertising Constant Tone Extension Transmit Duration. */
    BLE_ATT_CHAR_CTE_ADV_INTERVAL                                = BLE_ATT_UUID_16(0x2ACE), /**< Advertising Constant Tone Extension Interval. */
    BLE_ATT_CHAR_CTE_ADV_PHY                                     = BLE_ATT_UUID_16(0x2ACF), /**< Advertising Constatn Tone Extension PHY. */

    BLE_ATT_CHAR_CLI_SUP_FEAT                                    = BLE_ATT_UUID_16(0x2B29), /**< Client Supported Features. */
    BLE_ATT_CHAR_DB_HASH                                         = BLE_ATT_UUID_16(0x2B2A), /**< Database Hash. */
    BLE_ATT_CHAR_REGISTERED_USER                                 = BLE_ATT_UUID_16(0X2B37), /**< Registered User.  */
    BLE_ATT_CHAR_SRV_SUP_FEAT                                    = BLE_ATT_UUID_16(0x2B3A), /**< Server Supported Features. */
}att_uuid_t;

/**
 * @brief Format for Characteristic Presentation.
 */
typedef enum
{
    BLE_ATT_FORMAT_BOOL = 0x01,     /**< Unsigned 1-bit: true or false. */
    BLE_ATT_FORMAT_2BIT,            /**< Unsigned 2-bit integer. */
    BLE_ATT_FORMAT_NIBBLE,          /**< Unsigned 4-bit integer. */
    BLE_ATT_FORMAT_UINT8,           /**< Unsigned 8-bit integer. */
    BLE_ATT_FORMAT_UINT12,          /**< Unsigned 12-bit integer. */
    BLE_ATT_FORMAT_UINT16,          /**< Unsigned 16-bit integer. */
    BLE_ATT_FORMAT_UINT24,          /**< Unsigned 24-bit integer. */
    BLE_ATT_FORMAT_UINT32,          /**< Unsigned 32-bit integer. */
    BLE_ATT_FORMAT_UINT48,          /**< Unsigned 48-bit integer. */
    BLE_ATT_FORMAT_UINT64,          /**< Unsigned 64-bit integer. */
    BLE_ATT_FORMAT_UINT128,         /**< Unsigned 128-bit integer. */
    BLE_ATT_FORMAT_SINT8,           /**< Signed 8-bit integer. */
    BLE_ATT_FORMAT_SINT12,          /**< Signed 12-bit integer. */
    BLE_ATT_FORMAT_SINT16,          /**< Signed 16-bit integer. */
    BLE_ATT_FORMAT_SINT24,          /**< Signed 24-bit integer. */
    BLE_ATT_FORMAT_SINT32,          /**< Signed 32-bit integer. */
    BLE_ATT_FORMAT_SINT48,          /**< Signed 48-bit integer. */
    BLE_ATT_FORMAT_SINT64,          /**< Signed 64-bit integer. */
    BLE_ATT_FORMAT_SINT128,         /**< Signed 128-bit integer. */
    BLE_ATT_FORMAT_FLOAT32,         /**< IEEE-754 32-bit floating point. */
    BLE_ATT_FORMAT_FLOAT64,         /**< IEEE-754 64-bit floating point. */
    BLE_ATT_FORMAT_SFLOAT,          /**< IEEE-11073 16-bit SFLOAT. */
    BLE_ATT_FORMAT_FLOAT,           /**< IEEE-11073 32-bit FLOAT. */
    BLE_ATT_FORMAT_DUINT16,         /**< IEEE-20601 format. */
    BLE_ATT_FORMAT_UTF8S,           /**< UTF-8 string. */
    BLE_ATT_FORMAT_UTF16S,          /**< UTF-16 string. */
    BLE_ATT_FORMAT_STRUCT,          /**< Opaque structure. */
    BLE_ATT_FORMAT_LAST             /**< Last format. */
}att_format_t;
/** @} */

#endif
/** @} */
/** @} */

