/* ----------------------------------------------------------------------   
* Copyright (C) 2010 ARM Limited. All rights reserved.   
*   
* $Date:        15. July 2011  
* $Revision: 	V1.0.10  
*   
* Project: 	    CMSIS DSP Library   
* Title:	    arm_cfft_radix4_init_q31.c   
*   
* Description:	Radix-4 Decimation in Frequency Q31 FFT & IFFT initialization function   
*   
* Target Processor: Cortex-M4/Cortex-M3/Cortex-M0
*  
* Version 1.0.10 2011/7/15 
*    Big Endian support added and Merged M0 and M3/M4 Source code.  
*   
* Version 1.0.3 2010/11/29  
*    Re-organized the CMSIS folders and updated documentation.   
*    
* Version 1.0.2 2010/11/11   
*    Documentation updated.    
*   
* Version 1.0.1 2010/10/05    
*    Production release and review comments incorporated.   
*   
* Version 1.0.0 2010/09/20    
*    Production release and review comments incorporated.   
*   
* Version 0.0.5  2010/04/26    
* 	 incorporated review comments and updated with latest CMSIS layer   
*   
* Version 0.0.3  2010/03/10    
*    Initial version   
* -------------------------------------------------------------------- */

#include "arm_math.h"
#include "arm_common_tables.h"

/**   
 * @ingroup groupTransforms   
 */

/**   
 * @addtogroup CFFT_CIFFT   
 * @{   
 */

/*   
* @brief  Twiddle factors Table   
*/

/**   
* \par  
* Example code for Q31 Twiddle factors Generation::   
* \par   
* <pre>for(i = 0; i< N; i++)   
* {   
*    twiddleCoefQ31[2*i]= cos(i * 2*PI/(float)N);   
*    twiddleCoefQ31[2*i+1]= sin(i * 2*PI/(float)N);   
* } </pre>   
* \par   
* where N = 1024	and PI = 3.14159265358979   
* \par   
* Cos and Sin values are interleaved fashion   
* \par   
* Convert Floating point to Q31(Fixed point 1.31):   
*	round(twiddleCoefQ31(i) * pow(2, 31))   
*   
*/

static const q31_t twiddleCoefQ31[2048] = {
  0x7fffffff, 0x0, 0x7fff6216, 0xc90f88, 0x7ffd885a, 0x1921d20, 0x7ffa72d1,
  0x25b26d7,
  0x7ff62182, 0x3242abf, 0x7ff09478, 0x3ed26e6, 0x7fe9cbc0, 0x4b6195d,
  0x7fe1c76b, 0x57f0035,
  0x7fd8878e, 0x647d97c, 0x7fce0c3e, 0x710a345, 0x7fc25596, 0x7d95b9e,
  0x7fb563b3, 0x8a2009a,
  0x7fa736b4, 0x96a9049, 0x7f97cebd, 0xa3308bd, 0x7f872bf3, 0xafb6805,
  0x7f754e80, 0xbc3ac35,
  0x7f62368f, 0xc8bd35e, 0x7f4de451, 0xd53db92, 0x7f3857f6, 0xe1bc2e4,
  0x7f2191b4, 0xee38766,
  0x7f0991c4, 0xfab272b, 0x7ef05860, 0x1072a048, 0x7ed5e5c6, 0x1139f0cf,
  0x7eba3a39, 0x120116d5,
  0x7e9d55fc, 0x12c8106f, 0x7e7f3957, 0x138edbb1, 0x7e5fe493, 0x145576b1,
  0x7e3f57ff, 0x151bdf86,
  0x7e1d93ea, 0x15e21445, 0x7dfa98a8, 0x16a81305, 0x7dd6668f, 0x176dd9de,
  0x7db0fdf8, 0x183366e9,
  0x7d8a5f40, 0x18f8b83c, 0x7d628ac6, 0x19bdcbf3, 0x7d3980ec, 0x1a82a026,
  0x7d0f4218, 0x1b4732ef,
  0x7ce3ceb2, 0x1c0b826a, 0x7cb72724, 0x1ccf8cb3, 0x7c894bde, 0x1d934fe5,
  0x7c5a3d50, 0x1e56ca1e,
  0x7c29fbee, 0x1f19f97b, 0x7bf88830, 0x1fdcdc1b, 0x7bc5e290, 0x209f701c,
  0x7b920b89, 0x2161b3a0,
  0x7b5d039e, 0x2223a4c5, 0x7b26cb4f, 0x22e541af, 0x7aef6323, 0x23a6887f,
  0x7ab6cba4, 0x24677758,
  0x7a7d055b, 0x25280c5e, 0x7a4210d8, 0x25e845b6, 0x7a05eead, 0x26a82186,
  0x79c89f6e, 0x27679df4,
  0x798a23b1, 0x2826b928, 0x794a7c12, 0x28e5714b, 0x7909a92d, 0x29a3c485,
  0x78c7aba2, 0x2a61b101,
  0x78848414, 0x2b1f34eb, 0x78403329, 0x2bdc4e6f, 0x77fab989, 0x2c98fbba,
  0x77b417df, 0x2d553afc,
  0x776c4edb, 0x2e110a62, 0x77235f2d, 0x2ecc681e, 0x76d94989, 0x2f875262,
  0x768e0ea6, 0x3041c761,
  0x7641af3d, 0x30fbc54d, 0x75f42c0b, 0x31b54a5e, 0x75a585cf, 0x326e54c7,
  0x7555bd4c, 0x3326e2c3,
  0x7504d345, 0x33def287, 0x74b2c884, 0x34968250, 0x745f9dd1, 0x354d9057,
  0x740b53fb, 0x36041ad9,
  0x73b5ebd1, 0x36ba2014, 0x735f6626, 0x376f9e46, 0x7307c3d0, 0x382493b0,
  0x72af05a7, 0x38d8fe93,
  0x72552c85, 0x398cdd32, 0x71fa3949, 0x3a402dd2, 0x719e2cd2, 0x3af2eeb7,
  0x71410805, 0x3ba51e29,
  0x70e2cbc6, 0x3c56ba70, 0x708378ff, 0x3d07c1d6, 0x7023109a, 0x3db832a6,
  0x6fc19385, 0x3e680b2c,
  0x6f5f02b2, 0x3f1749b8, 0x6efb5f12, 0x3fc5ec98, 0x6e96a99d, 0x4073f21d,
  0x6e30e34a, 0x4121589b,
  0x6dca0d14, 0x41ce1e65, 0x6d6227fa, 0x427a41d0, 0x6cf934fc, 0x4325c135,
  0x6c8f351c, 0x43d09aed,
  0x6c242960, 0x447acd50, 0x6bb812d1, 0x452456bd, 0x6b4af279, 0x45cd358f,
  0x6adcc964, 0x46756828,
  0x6a6d98a4, 0x471cece7, 0x69fd614a, 0x47c3c22f, 0x698c246c, 0x4869e665,
  0x6919e320, 0x490f57ee,
  0x68a69e81, 0x49b41533, 0x683257ab, 0x4a581c9e, 0x67bd0fbd, 0x4afb6c98,
  0x6746c7d8, 0x4b9e0390,
  0x66cf8120, 0x4c3fdff4, 0x66573cbb, 0x4ce10034, 0x65ddfbd3, 0x4d8162c4,
  0x6563bf92, 0x4e210617,
  0x64e88926, 0x4ebfe8a5, 0x646c59bf, 0x4f5e08e3, 0x63ef3290, 0x4ffb654d,
  0x637114cc, 0x5097fc5e,
  0x62f201ac, 0x5133cc94, 0x6271fa69, 0x51ced46e, 0x61f1003f, 0x5269126e,
  0x616f146c, 0x53028518,
  0x60ec3830, 0x539b2af0, 0x60686ccf, 0x5433027d, 0x5fe3b38d, 0x54ca0a4b,
  0x5f5e0db3, 0x556040e2,
  0x5ed77c8a, 0x55f5a4d2, 0x5e50015d, 0x568a34a9, 0x5dc79d7c, 0x571deefa,
  0x5d3e5237, 0x57b0d256,
  0x5cb420e0, 0x5842dd54, 0x5c290acc, 0x58d40e8c, 0x5b9d1154, 0x59646498,
  0x5b1035cf, 0x59f3de12,
  0x5a82799a, 0x5a82799a, 0x59f3de12, 0x5b1035cf, 0x59646498, 0x5b9d1154,
  0x58d40e8c, 0x5c290acc,
  0x5842dd54, 0x5cb420e0, 0x57b0d256, 0x5d3e5237, 0x571deefa, 0x5dc79d7c,
  0x568a34a9, 0x5e50015d,
  0x55f5a4d2, 0x5ed77c8a, 0x556040e2, 0x5f5e0db3, 0x54ca0a4b, 0x5fe3b38d,
  0x5433027d, 0x60686ccf,
  0x539b2af0, 0x60ec3830, 0x53028518, 0x616f146c, 0x5269126e, 0x61f1003f,
  0x51ced46e, 0x6271fa69,
  0x5133cc94, 0x62f201ac, 0x5097fc5e, 0x637114cc, 0x4ffb654d, 0x63ef3290,
  0x4f5e08e3, 0x646c59bf,
  0x4ebfe8a5, 0x64e88926, 0x4e210617, 0x6563bf92, 0x4d8162c4, 0x65ddfbd3,
  0x4ce10034, 0x66573cbb,
  0x4c3fdff4, 0x66cf8120, 0x4b9e0390, 0x6746c7d8, 0x4afb6c98, 0x67bd0fbd,
  0x4a581c9e, 0x683257ab,
  0x49b41533, 0x68a69e81, 0x490f57ee, 0x6919e320, 0x4869e665, 0x698c246c,
  0x47c3c22f, 0x69fd614a,
  0x471cece7, 0x6a6d98a4, 0x46756828, 0x6adcc964, 0x45cd358f, 0x6b4af279,
  0x452456bd, 0x6bb812d1,
  0x447acd50, 0x6c242960, 0x43d09aed, 0x6c8f351c, 0x4325c135, 0x6cf934fc,
  0x427a41d0, 0x6d6227fa,
  0x41ce1e65, 0x6dca0d14, 0x4121589b, 0x6e30e34a, 0x4073f21d, 0x6e96a99d,
  0x3fc5ec98, 0x6efb5f12,
  0x3f1749b8, 0x6f5f02b2, 0x3e680b2c, 0x6fc19385, 0x3db832a6, 0x7023109a,
  0x3d07c1d6, 0x708378ff,
  0x3c56ba70, 0x70e2cbc6, 0x3ba51e29, 0x71410805, 0x3af2eeb7, 0x719e2cd2,
  0x3a402dd2, 0x71fa3949,
  0x398cdd32, 0x72552c85, 0x38d8fe93, 0x72af05a7, 0x382493b0, 0x7307c3d0,
  0x376f9e46, 0x735f6626,
  0x36ba2014, 0x73b5ebd1, 0x36041ad9, 0x740b53fb, 0x354d9057, 0x745f9dd1,
  0x34968250, 0x74b2c884,
  0x33def287, 0x7504d345, 0x3326e2c3, 0x7555bd4c, 0x326e54c7, 0x75a585cf,
  0x31b54a5e, 0x75f42c0b,
  0x30fbc54d, 0x7641af3d, 0x3041c761, 0x768e0ea6, 0x2f875262, 0x76d94989,
  0x2ecc681e, 0x77235f2d,
  0x2e110a62, 0x776c4edb, 0x2d553afc, 0x77b417df, 0x2c98fbba, 0x77fab989,
  0x2bdc4e6f, 0x78403329,
  0x2b1f34eb, 0x78848414, 0x2a61b101, 0x78c7aba2, 0x29a3c485, 0x7909a92d,
  0x28e5714b, 0x794a7c12,
  0x2826b928, 0x798a23b1, 0x27679df4, 0x79c89f6e, 0x26a82186, 0x7a05eead,
  0x25e845b6, 0x7a4210d8,
  0x25280c5e, 0x7a7d055b, 0x24677758, 0x7ab6cba4, 0x23a6887f, 0x7aef6323,
  0x22e541af, 0x7b26cb4f,
  0x2223a4c5, 0x7b5d039e, 0x2161b3a0, 0x7b920b89, 0x209f701c, 0x7bc5e290,
  0x1fdcdc1b, 0x7bf88830,
  0x1f19f97b, 0x7c29fbee, 0x1e56ca1e, 0x7c5a3d50, 0x1d934fe5, 0x7c894bde,
  0x1ccf8cb3, 0x7cb72724,
  0x1c0b826a, 0x7ce3ceb2, 0x1b4732ef, 0x7d0f4218, 0x1a82a026, 0x7d3980ec,
  0x19bdcbf3, 0x7d628ac6,
  0x18f8b83c, 0x7d8a5f40, 0x183366e9, 0x7db0fdf8, 0x176dd9de, 0x7dd6668f,
  0x16a81305, 0x7dfa98a8,
  0x15e21445, 0x7e1d93ea, 0x151bdf86, 0x7e3f57ff, 0x145576b1, 0x7e5fe493,
  0x138edbb1, 0x7e7f3957,
  0x12c8106f, 0x7e9d55fc, 0x120116d5, 0x7eba3a39, 0x1139f0cf, 0x7ed5e5c6,
  0x1072a048, 0x7ef05860,
  0xfab272b, 0x7f0991c4, 0xee38766, 0x7f2191b4, 0xe1bc2e4, 0x7f3857f6,
  0xd53db92, 0x7f4de451,
  0xc8bd35e, 0x7f62368f, 0xbc3ac35, 0x7f754e80, 0xafb6805, 0x7f872bf3,
  0xa3308bd, 0x7f97cebd,
  0x96a9049, 0x7fa736b4, 0x8a2009a, 0x7fb563b3, 0x7d95b9e, 0x7fc25596,
  0x710a345, 0x7fce0c3e,
  0x647d97c, 0x7fd8878e, 0x57f0035, 0x7fe1c76b, 0x4b6195d, 0x7fe9cbc0,
  0x3ed26e6, 0x7ff09478,
  0x3242abf, 0x7ff62182, 0x25b26d7, 0x7ffa72d1, 0x1921d20, 0x7ffd885a,
  0xc90f88, 0x7fff6216,
  0x0, 0x7fffffff, 0xff36f078, 0x7fff6216, 0xfe6de2e0, 0x7ffd885a, 0xfda4d929,
  0x7ffa72d1,
  0xfcdbd541, 0x7ff62182, 0xfc12d91a, 0x7ff09478, 0xfb49e6a3, 0x7fe9cbc0,
  0xfa80ffcb, 0x7fe1c76b,
  0xf9b82684, 0x7fd8878e, 0xf8ef5cbb, 0x7fce0c3e, 0xf826a462, 0x7fc25596,
  0xf75dff66, 0x7fb563b3,
  0xf6956fb7, 0x7fa736b4, 0xf5ccf743, 0x7f97cebd, 0xf50497fb, 0x7f872bf3,
  0xf43c53cb, 0x7f754e80,
  0xf3742ca2, 0x7f62368f, 0xf2ac246e, 0x7f4de451, 0xf1e43d1c, 0x7f3857f6,
  0xf11c789a, 0x7f2191b4,
  0xf054d8d5, 0x7f0991c4, 0xef8d5fb8, 0x7ef05860, 0xeec60f31, 0x7ed5e5c6,
  0xedfee92b, 0x7eba3a39,
  0xed37ef91, 0x7e9d55fc, 0xec71244f, 0x7e7f3957, 0xebaa894f, 0x7e5fe493,
  0xeae4207a, 0x7e3f57ff,
  0xea1debbb, 0x7e1d93ea, 0xe957ecfb, 0x7dfa98a8, 0xe8922622, 0x7dd6668f,
  0xe7cc9917, 0x7db0fdf8,
  0xe70747c4, 0x7d8a5f40, 0xe642340d, 0x7d628ac6, 0xe57d5fda, 0x7d3980ec,
  0xe4b8cd11, 0x7d0f4218,
  0xe3f47d96, 0x7ce3ceb2, 0xe330734d, 0x7cb72724, 0xe26cb01b, 0x7c894bde,
  0xe1a935e2, 0x7c5a3d50,
  0xe0e60685, 0x7c29fbee, 0xe02323e5, 0x7bf88830, 0xdf608fe4, 0x7bc5e290,
  0xde9e4c60, 0x7b920b89,
  0xdddc5b3b, 0x7b5d039e, 0xdd1abe51, 0x7b26cb4f, 0xdc597781, 0x7aef6323,
  0xdb9888a8, 0x7ab6cba4,
  0xdad7f3a2, 0x7a7d055b, 0xda17ba4a, 0x7a4210d8, 0xd957de7a, 0x7a05eead,
  0xd898620c, 0x79c89f6e,
  0xd7d946d8, 0x798a23b1, 0xd71a8eb5, 0x794a7c12, 0xd65c3b7b, 0x7909a92d,
  0xd59e4eff, 0x78c7aba2,
  0xd4e0cb15, 0x78848414, 0xd423b191, 0x78403329, 0xd3670446, 0x77fab989,
  0xd2aac504, 0x77b417df,
  0xd1eef59e, 0x776c4edb, 0xd13397e2, 0x77235f2d, 0xd078ad9e, 0x76d94989,
  0xcfbe389f, 0x768e0ea6,
  0xcf043ab3, 0x7641af3d, 0xce4ab5a2, 0x75f42c0b, 0xcd91ab39, 0x75a585cf,
  0xccd91d3d, 0x7555bd4c,
  0xcc210d79, 0x7504d345, 0xcb697db0, 0x74b2c884, 0xcab26fa9, 0x745f9dd1,
  0xc9fbe527, 0x740b53fb,
  0xc945dfec, 0x73b5ebd1, 0xc89061ba, 0x735f6626, 0xc7db6c50, 0x7307c3d0,
  0xc727016d, 0x72af05a7,
  0xc67322ce, 0x72552c85, 0xc5bfd22e, 0x71fa3949, 0xc50d1149, 0x719e2cd2,
  0xc45ae1d7, 0x71410805,
  0xc3a94590, 0x70e2cbc6, 0xc2f83e2a, 0x708378ff, 0xc247cd5a, 0x7023109a,
  0xc197f4d4, 0x6fc19385,
  0xc0e8b648, 0x6f5f02b2, 0xc03a1368, 0x6efb5f12, 0xbf8c0de3, 0x6e96a99d,
  0xbedea765, 0x6e30e34a,
  0xbe31e19b, 0x6dca0d14, 0xbd85be30, 0x6d6227fa, 0xbcda3ecb, 0x6cf934fc,
  0xbc2f6513, 0x6c8f351c,
  0xbb8532b0, 0x6c242960, 0xbadba943, 0x6bb812d1, 0xba32ca71, 0x6b4af279,
  0xb98a97d8, 0x6adcc964,
  0xb8e31319, 0x6a6d98a4, 0xb83c3dd1, 0x69fd614a, 0xb796199b, 0x698c246c,
  0xb6f0a812, 0x6919e320,
  0xb64beacd, 0x68a69e81, 0xb5a7e362, 0x683257ab, 0xb5049368, 0x67bd0fbd,
  0xb461fc70, 0x6746c7d8,
  0xb3c0200c, 0x66cf8120, 0xb31effcc, 0x66573cbb, 0xb27e9d3c, 0x65ddfbd3,
  0xb1def9e9, 0x6563bf92,
  0xb140175b, 0x64e88926, 0xb0a1f71d, 0x646c59bf, 0xb0049ab3, 0x63ef3290,
  0xaf6803a2, 0x637114cc,
  0xaecc336c, 0x62f201ac, 0xae312b92, 0x6271fa69, 0xad96ed92, 0x61f1003f,
  0xacfd7ae8, 0x616f146c,
  0xac64d510, 0x60ec3830, 0xabccfd83, 0x60686ccf, 0xab35f5b5, 0x5fe3b38d,
  0xaa9fbf1e, 0x5f5e0db3,
  0xaa0a5b2e, 0x5ed77c8a, 0xa975cb57, 0x5e50015d, 0xa8e21106, 0x5dc79d7c,
  0xa84f2daa, 0x5d3e5237,
  0xa7bd22ac, 0x5cb420e0, 0xa72bf174, 0x5c290acc, 0xa69b9b68, 0x5b9d1154,
  0xa60c21ee, 0x5b1035cf,
  0xa57d8666, 0x5a82799a, 0xa4efca31, 0x59f3de12, 0xa462eeac, 0x59646498,
  0xa3d6f534, 0x58d40e8c,
  0xa34bdf20, 0x5842dd54, 0xa2c1adc9, 0x57b0d256, 0xa2386284, 0x571deefa,
  0xa1affea3, 0x568a34a9,
  0xa1288376, 0x55f5a4d2, 0xa0a1f24d, 0x556040e2, 0xa01c4c73, 0x54ca0a4b,
  0x9f979331, 0x5433027d,
  0x9f13c7d0, 0x539b2af0, 0x9e90eb94, 0x53028518, 0x9e0effc1, 0x5269126e,
  0x9d8e0597, 0x51ced46e,
  0x9d0dfe54, 0x5133cc94, 0x9c8eeb34, 0x5097fc5e, 0x9c10cd70, 0x4ffb654d,
  0x9b93a641, 0x4f5e08e3,
  0x9b1776da, 0x4ebfe8a5, 0x9a9c406e, 0x4e210617, 0x9a22042d, 0x4d8162c4,
  0x99a8c345, 0x4ce10034,
  0x99307ee0, 0x4c3fdff4, 0x98b93828, 0x4b9e0390, 0x9842f043, 0x4afb6c98,
  0x97cda855, 0x4a581c9e,
  0x9759617f, 0x49b41533, 0x96e61ce0, 0x490f57ee, 0x9673db94, 0x4869e665,
  0x96029eb6, 0x47c3c22f,
  0x9592675c, 0x471cece7, 0x9523369c, 0x46756828, 0x94b50d87, 0x45cd358f,
  0x9447ed2f, 0x452456bd,
  0x93dbd6a0, 0x447acd50, 0x9370cae4, 0x43d09aed, 0x9306cb04, 0x4325c135,
  0x929dd806, 0x427a41d0,
  0x9235f2ec, 0x41ce1e65, 0x91cf1cb6, 0x4121589b, 0x91695663, 0x4073f21d,
  0x9104a0ee, 0x3fc5ec98,
  0x90a0fd4e, 0x3f1749b8, 0x903e6c7b, 0x3e680b2c, 0x8fdcef66, 0x3db832a6,
  0x8f7c8701, 0x3d07c1d6,
  0x8f1d343a, 0x3c56ba70, 0x8ebef7fb, 0x3ba51e29, 0x8e61d32e, 0x3af2eeb7,
  0x8e05c6b7, 0x3a402dd2,
  0x8daad37b, 0x398cdd32, 0x8d50fa59, 0x38d8fe93, 0x8cf83c30, 0x382493b0,
  0x8ca099da, 0x376f9e46,
  0x8c4a142f, 0x36ba2014, 0x8bf4ac05, 0x36041ad9, 0x8ba0622f, 0x354d9057,
  0x8b4d377c, 0x34968250,
  0x8afb2cbb, 0x33def287, 0x8aaa42b4, 0x3326e2c3, 0x8a5a7a31, 0x326e54c7,
  0x8a0bd3f5, 0x31b54a5e,
  0x89be50c3, 0x30fbc54d, 0x8971f15a, 0x3041c761, 0x8926b677, 0x2f875262,
  0x88dca0d3, 0x2ecc681e,
  0x8893b125, 0x2e110a62, 0x884be821, 0x2d553afc, 0x88054677, 0x2c98fbba,
  0x87bfccd7, 0x2bdc4e6f,
  0x877b7bec, 0x2b1f34eb, 0x8738545e, 0x2a61b101, 0x86f656d3, 0x29a3c485,
  0x86b583ee, 0x28e5714b,
  0x8675dc4f, 0x2826b928, 0x86376092, 0x27679df4, 0x85fa1153, 0x26a82186,
  0x85bdef28, 0x25e845b6,
  0x8582faa5, 0x25280c5e, 0x8549345c, 0x24677758, 0x85109cdd, 0x23a6887f,
  0x84d934b1, 0x22e541af,
  0x84a2fc62, 0x2223a4c5, 0x846df477, 0x2161b3a0, 0x843a1d70, 0x209f701c,
  0x840777d0, 0x1fdcdc1b,
  0x83d60412, 0x1f19f97b, 0x83a5c2b0, 0x1e56ca1e, 0x8376b422, 0x1d934fe5,
  0x8348d8dc, 0x1ccf8cb3,
  0x831c314e, 0x1c0b826a, 0x82f0bde8, 0x1b4732ef, 0x82c67f14, 0x1a82a026,
  0x829d753a, 0x19bdcbf3,
  0x8275a0c0, 0x18f8b83c, 0x824f0208, 0x183366e9, 0x82299971, 0x176dd9de,
  0x82056758, 0x16a81305,
  0x81e26c16, 0x15e21445, 0x81c0a801, 0x151bdf86, 0x81a01b6d, 0x145576b1,
  0x8180c6a9, 0x138edbb1,
  0x8162aa04, 0x12c8106f, 0x8145c5c7, 0x120116d5, 0x812a1a3a, 0x1139f0cf,
  0x810fa7a0, 0x1072a048,
  0x80f66e3c, 0xfab272b, 0x80de6e4c, 0xee38766, 0x80c7a80a, 0xe1bc2e4,
  0x80b21baf, 0xd53db92,
  0x809dc971, 0xc8bd35e, 0x808ab180, 0xbc3ac35, 0x8078d40d, 0xafb6805,
  0x80683143, 0xa3308bd,
  0x8058c94c, 0x96a9049, 0x804a9c4d, 0x8a2009a, 0x803daa6a, 0x7d95b9e,
  0x8031f3c2, 0x710a345,
  0x80277872, 0x647d97c, 0x801e3895, 0x57f0035, 0x80163440, 0x4b6195d,
  0x800f6b88, 0x3ed26e6,
  0x8009de7e, 0x3242abf, 0x80058d2f, 0x25b26d7, 0x800277a6, 0x1921d20,
  0x80009dea, 0xc90f88,
  0x80000000, 0x0, 0x80009dea, 0xff36f078, 0x800277a6, 0xfe6de2e0, 0x80058d2f,
  0xfda4d929,
  0x8009de7e, 0xfcdbd541, 0x800f6b88, 0xfc12d91a, 0x80163440, 0xfb49e6a3,
  0x801e3895, 0xfa80ffcb,
  0x80277872, 0xf9b82684, 0x8031f3c2, 0xf8ef5cbb, 0x803daa6a, 0xf826a462,
  0x804a9c4d, 0xf75dff66,
  0x8058c94c, 0xf6956fb7, 0x80683143, 0xf5ccf743, 0x8078d40d, 0xf50497fb,
  0x808ab180, 0xf43c53cb,
  0x809dc971, 0xf3742ca2, 0x80b21baf, 0xf2ac246e, 0x80c7a80a, 0xf1e43d1c,
  0x80de6e4c, 0xf11c789a,
  0x80f66e3c, 0xf054d8d5, 0x810fa7a0, 0xef8d5fb8, 0x812a1a3a, 0xeec60f31,
  0x8145c5c7, 0xedfee92b,
  0x8162aa04, 0xed37ef91, 0x8180c6a9, 0xec71244f, 0x81a01b6d, 0xebaa894f,
  0x81c0a801, 0xeae4207a,
  0x81e26c16, 0xea1debbb, 0x82056758, 0xe957ecfb, 0x82299971, 0xe8922622,
  0x824f0208, 0xe7cc9917,
  0x8275a0c0, 0xe70747c4, 0x829d753a, 0xe642340d, 0x82c67f14, 0xe57d5fda,
  0x82f0bde8, 0xe4b8cd11,
  0x831c314e, 0xe3f47d96, 0x8348d8dc, 0xe330734d, 0x8376b422, 0xe26cb01b,
  0x83a5c2b0, 0xe1a935e2,
  0x83d60412, 0xe0e60685, 0x840777d0, 0xe02323e5, 0x843a1d70, 0xdf608fe4,
  0x846df477, 0xde9e4c60,
  0x84a2fc62, 0xdddc5b3b, 0x84d934b1, 0xdd1abe51, 0x85109cdd, 0xdc597781,
  0x8549345c, 0xdb9888a8,
  0x8582faa5, 0xdad7f3a2, 0x85bdef28, 0xda17ba4a, 0x85fa1153, 0xd957de7a,
  0x86376092, 0xd898620c,
  0x8675dc4f, 0xd7d946d8, 0x86b583ee, 0xd71a8eb5, 0x86f656d3, 0xd65c3b7b,
  0x8738545e, 0xd59e4eff,
  0x877b7bec, 0xd4e0cb15, 0x87bfccd7, 0xd423b191, 0x88054677, 0xd3670446,
  0x884be821, 0xd2aac504,
  0x8893b125, 0xd1eef59e, 0x88dca0d3, 0xd13397e2, 0x8926b677, 0xd078ad9e,
  0x8971f15a, 0xcfbe389f,
  0x89be50c3, 0xcf043ab3, 0x8a0bd3f5, 0xce4ab5a2, 0x8a5a7a31, 0xcd91ab39,
  0x8aaa42b4, 0xccd91d3d,
  0x8afb2cbb, 0xcc210d79, 0x8b4d377c, 0xcb697db0, 0x8ba0622f, 0xcab26fa9,
  0x8bf4ac05, 0xc9fbe527,
  0x8c4a142f, 0xc945dfec, 0x8ca099da, 0xc89061ba, 0x8cf83c30, 0xc7db6c50,
  0x8d50fa59, 0xc727016d,
  0x8daad37b, 0xc67322ce, 0x8e05c6b7, 0xc5bfd22e, 0x8e61d32e, 0xc50d1149,
  0x8ebef7fb, 0xc45ae1d7,
  0x8f1d343a, 0xc3a94590, 0x8f7c8701, 0xc2f83e2a, 0x8fdcef66, 0xc247cd5a,
  0x903e6c7b, 0xc197f4d4,
  0x90a0fd4e, 0xc0e8b648, 0x9104a0ee, 0xc03a1368, 0x91695663, 0xbf8c0de3,
  0x91cf1cb6, 0xbedea765,
  0x9235f2ec, 0xbe31e19b, 0x929dd806, 0xbd85be30, 0x9306cb04, 0xbcda3ecb,
  0x9370cae4, 0xbc2f6513,
  0x93dbd6a0, 0xbb8532b0, 0x9447ed2f, 0xbadba943, 0x94b50d87, 0xba32ca71,
  0x9523369c, 0xb98a97d8,
  0x9592675c, 0xb8e31319, 0x96029eb6, 0xb83c3dd1, 0x9673db94, 0xb796199b,
  0x96e61ce0, 0xb6f0a812,
  0x9759617f, 0xb64beacd, 0x97cda855, 0xb5a7e362, 0x9842f043, 0xb5049368,
  0x98b93828, 0xb461fc70,
  0x99307ee0, 0xb3c0200c, 0x99a8c345, 0xb31effcc, 0x9a22042d, 0xb27e9d3c,
  0x9a9c406e, 0xb1def9e9,
  0x9b1776da, 0xb140175b, 0x9b93a641, 0xb0a1f71d, 0x9c10cd70, 0xb0049ab3,
  0x9c8eeb34, 0xaf6803a2,
  0x9d0dfe54, 0xaecc336c, 0x9d8e0597, 0xae312b92, 0x9e0effc1, 0xad96ed92,
  0x9e90eb94, 0xacfd7ae8,
  0x9f13c7d0, 0xac64d510, 0x9f979331, 0xabccfd83, 0xa01c4c73, 0xab35f5b5,
  0xa0a1f24d, 0xaa9fbf1e,
  0xa1288376, 0xaa0a5b2e, 0xa1affea3, 0xa975cb57, 0xa2386284, 0xa8e21106,
  0xa2c1adc9, 0xa84f2daa,
  0xa34bdf20, 0xa7bd22ac, 0xa3d6f534, 0xa72bf174, 0xa462eeac, 0xa69b9b68,
  0xa4efca31, 0xa60c21ee,
  0xa57d8666, 0xa57d8666, 0xa60c21ee, 0xa4efca31, 0xa69b9b68, 0xa462eeac,
  0xa72bf174, 0xa3d6f534,
  0xa7bd22ac, 0xa34bdf20, 0xa84f2daa, 0xa2c1adc9, 0xa8e21106, 0xa2386284,
  0xa975cb57, 0xa1affea3,
  0xaa0a5b2e, 0xa1288376, 0xaa9fbf1e, 0xa0a1f24d, 0xab35f5b5, 0xa01c4c73,
  0xabccfd83, 0x9f979331,
  0xac64d510, 0x9f13c7d0, 0xacfd7ae8, 0x9e90eb94, 0xad96ed92, 0x9e0effc1,
  0xae312b92, 0x9d8e0597,
  0xaecc336c, 0x9d0dfe54, 0xaf6803a2, 0x9c8eeb34, 0xb0049ab3, 0x9c10cd70,
  0xb0a1f71d, 0x9b93a641,
  0xb140175b, 0x9b1776da, 0xb1def9e9, 0x9a9c406e, 0xb27e9d3c, 0x9a22042d,
  0xb31effcc, 0x99a8c345,
  0xb3c0200c, 0x99307ee0, 0xb461fc70, 0x98b93828, 0xb5049368, 0x9842f043,
  0xb5a7e362, 0x97cda855,
  0xb64beacd, 0x9759617f, 0xb6f0a812, 0x96e61ce0, 0xb796199b, 0x9673db94,
  0xb83c3dd1, 0x96029eb6,
  0xb8e31319, 0x9592675c, 0xb98a97d8, 0x9523369c, 0xba32ca71, 0x94b50d87,
  0xbadba943, 0x9447ed2f,
  0xbb8532b0, 0x93dbd6a0, 0xbc2f6513, 0x9370cae4, 0xbcda3ecb, 0x9306cb04,
  0xbd85be30, 0x929dd806,
  0xbe31e19b, 0x9235f2ec, 0xbedea765, 0x91cf1cb6, 0xbf8c0de3, 0x91695663,
  0xc03a1368, 0x9104a0ee,
  0xc0e8b648, 0x90a0fd4e, 0xc197f4d4, 0x903e6c7b, 0xc247cd5a, 0x8fdcef66,
  0xc2f83e2a, 0x8f7c8701,
  0xc3a94590, 0x8f1d343a, 0xc45ae1d7, 0x8ebef7fb, 0xc50d1149, 0x8e61d32e,
  0xc5bfd22e, 0x8e05c6b7,
  0xc67322ce, 0x8daad37b, 0xc727016d, 0x8d50fa59, 0xc7db6c50, 0x8cf83c30,
  0xc89061ba, 0x8ca099da,
  0xc945dfec, 0x8c4a142f, 0xc9fbe527, 0x8bf4ac05, 0xcab26fa9, 0x8ba0622f,
  0xcb697db0, 0x8b4d377c,
  0xcc210d79, 0x8afb2cbb, 0xccd91d3d, 0x8aaa42b4, 0xcd91ab39, 0x8a5a7a31,
  0xce4ab5a2, 0x8a0bd3f5,
  0xcf043ab3, 0x89be50c3, 0xcfbe389f, 0x8971f15a, 0xd078ad9e, 0x8926b677,
  0xd13397e2, 0x88dca0d3,
  0xd1eef59e, 0x8893b125, 0xd2aac504, 0x884be821, 0xd3670446, 0x88054677,
  0xd423b191, 0x87bfccd7,
  0xd4e0cb15, 0x877b7bec, 0xd59e4eff, 0x8738545e, 0xd65c3b7b, 0x86f656d3,
  0xd71a8eb5, 0x86b583ee,
  0xd7d946d8, 0x8675dc4f, 0xd898620c, 0x86376092, 0xd957de7a, 0x85fa1153,
  0xda17ba4a, 0x85bdef28,
  0xdad7f3a2, 0x8582faa5, 0xdb9888a8, 0x8549345c, 0xdc597781, 0x85109cdd,
  0xdd1abe51, 0x84d934b1,
  0xdddc5b3b, 0x84a2fc62, 0xde9e4c60, 0x846df477, 0xdf608fe4, 0x843a1d70,
  0xe02323e5, 0x840777d0,
  0xe0e60685, 0x83d60412, 0xe1a935e2, 0x83a5c2b0, 0xe26cb01b, 0x8376b422,
  0xe330734d, 0x8348d8dc,
  0xe3f47d96, 0x831c314e, 0xe4b8cd11, 0x82f0bde8, 0xe57d5fda, 0x82c67f14,
  0xe642340d, 0x829d753a,
  0xe70747c4, 0x8275a0c0, 0xe7cc9917, 0x824f0208, 0xe8922622, 0x82299971,
  0xe957ecfb, 0x82056758,
  0xea1debbb, 0x81e26c16, 0xeae4207a, 0x81c0a801, 0xebaa894f, 0x81a01b6d,
  0xec71244f, 0x8180c6a9,
  0xed37ef91, 0x8162aa04, 0xedfee92b, 0x8145c5c7, 0xeec60f31, 0x812a1a3a,
  0xef8d5fb8, 0x810fa7a0,
  0xf054d8d5, 0x80f66e3c, 0xf11c789a, 0x80de6e4c, 0xf1e43d1c, 0x80c7a80a,
  0xf2ac246e, 0x80b21baf,
  0xf3742ca2, 0x809dc971, 0xf43c53cb, 0x808ab180, 0xf50497fb, 0x8078d40d,
  0xf5ccf743, 0x80683143,
  0xf6956fb7, 0x8058c94c, 0xf75dff66, 0x804a9c4d, 0xf826a462, 0x803daa6a,
  0xf8ef5cbb, 0x8031f3c2,
  0xf9b82684, 0x80277872, 0xfa80ffcb, 0x801e3895, 0xfb49e6a3, 0x80163440,
  0xfc12d91a, 0x800f6b88,
  0xfcdbd541, 0x8009de7e, 0xfda4d929, 0x80058d2f, 0xfe6de2e0, 0x800277a6,
  0xff36f078, 0x80009dea,
  0x0, 0x80000000, 0xc90f88, 0x80009dea, 0x1921d20, 0x800277a6, 0x25b26d7,
  0x80058d2f,
  0x3242abf, 0x8009de7e, 0x3ed26e6, 0x800f6b88, 0x4b6195d, 0x80163440,
  0x57f0035, 0x801e3895,
  0x647d97c, 0x80277872, 0x710a345, 0x8031f3c2, 0x7d95b9e, 0x803daa6a,
  0x8a2009a, 0x804a9c4d,
  0x96a9049, 0x8058c94c, 0xa3308bd, 0x80683143, 0xafb6805, 0x8078d40d,
  0xbc3ac35, 0x808ab180,
  0xc8bd35e, 0x809dc971, 0xd53db92, 0x80b21baf, 0xe1bc2e4, 0x80c7a80a,
  0xee38766, 0x80de6e4c,
  0xfab272b, 0x80f66e3c, 0x1072a048, 0x810fa7a0, 0x1139f0cf, 0x812a1a3a,
  0x120116d5, 0x8145c5c7,
  0x12c8106f, 0x8162aa04, 0x138edbb1, 0x8180c6a9, 0x145576b1, 0x81a01b6d,
  0x151bdf86, 0x81c0a801,
  0x15e21445, 0x81e26c16, 0x16a81305, 0x82056758, 0x176dd9de, 0x82299971,
  0x183366e9, 0x824f0208,
  0x18f8b83c, 0x8275a0c0, 0x19bdcbf3, 0x829d753a, 0x1a82a026, 0x82c67f14,
  0x1b4732ef, 0x82f0bde8,
  0x1c0b826a, 0x831c314e, 0x1ccf8cb3, 0x8348d8dc, 0x1d934fe5, 0x8376b422,
  0x1e56ca1e, 0x83a5c2b0,
  0x1f19f97b, 0x83d60412, 0x1fdcdc1b, 0x840777d0, 0x209f701c, 0x843a1d70,
  0x2161b3a0, 0x846df477,
  0x2223a4c5, 0x84a2fc62, 0x22e541af, 0x84d934b1, 0x23a6887f, 0x85109cdd,
  0x24677758, 0x8549345c,
  0x25280c5e, 0x8582faa5, 0x25e845b6, 0x85bdef28, 0x26a82186, 0x85fa1153,
  0x27679df4, 0x86376092,
  0x2826b928, 0x8675dc4f, 0x28e5714b, 0x86b583ee, 0x29a3c485, 0x86f656d3,
  0x2a61b101, 0x8738545e,
  0x2b1f34eb, 0x877b7bec, 0x2bdc4e6f, 0x87bfccd7, 0x2c98fbba, 0x88054677,
  0x2d553afc, 0x884be821,
  0x2e110a62, 0x8893b125, 0x2ecc681e, 0x88dca0d3, 0x2f875262, 0x8926b677,
  0x3041c761, 0x8971f15a,
  0x30fbc54d, 0x89be50c3, 0x31b54a5e, 0x8a0bd3f5, 0x326e54c7, 0x8a5a7a31,
  0x3326e2c3, 0x8aaa42b4,
  0x33def287, 0x8afb2cbb, 0x34968250, 0x8b4d377c, 0x354d9057, 0x8ba0622f,
  0x36041ad9, 0x8bf4ac05,
  0x36ba2014, 0x8c4a142f, 0x376f9e46, 0x8ca099da, 0x382493b0, 0x8cf83c30,
  0x38d8fe93, 0x8d50fa59,
  0x398cdd32, 0x8daad37b, 0x3a402dd2, 0x8e05c6b7, 0x3af2eeb7, 0x8e61d32e,
  0x3ba51e29, 0x8ebef7fb,
  0x3c56ba70, 0x8f1d343a, 0x3d07c1d6, 0x8f7c8701, 0x3db832a6, 0x8fdcef66,
  0x3e680b2c, 0x903e6c7b,
  0x3f1749b8, 0x90a0fd4e, 0x3fc5ec98, 0x9104a0ee, 0x4073f21d, 0x91695663,
  0x4121589b, 0x91cf1cb6,
  0x41ce1e65, 0x9235f2ec, 0x427a41d0, 0x929dd806, 0x4325c135, 0x9306cb04,
  0x43d09aed, 0x9370cae4,
  0x447acd50, 0x93dbd6a0, 0x452456bd, 0x9447ed2f, 0x45cd358f, 0x94b50d87,
  0x46756828, 0x9523369c,
  0x471cece7, 0x9592675c, 0x47c3c22f, 0x96029eb6, 0x4869e665, 0x9673db94,
  0x490f57ee, 0x96e61ce0,
  0x49b41533, 0x9759617f, 0x4a581c9e, 0x97cda855, 0x4afb6c98, 0x9842f043,
  0x4b9e0390, 0x98b93828,
  0x4c3fdff4, 0x99307ee0, 0x4ce10034, 0x99a8c345, 0x4d8162c4, 0x9a22042d,
  0x4e210617, 0x9a9c406e,
  0x4ebfe8a5, 0x9b1776da, 0x4f5e08e3, 0x9b93a641, 0x4ffb654d, 0x9c10cd70,
  0x5097fc5e, 0x9c8eeb34,
  0x5133cc94, 0x9d0dfe54, 0x51ced46e, 0x9d8e0597, 0x5269126e, 0x9e0effc1,
  0x53028518, 0x9e90eb94,
  0x539b2af0, 0x9f13c7d0, 0x5433027d, 0x9f979331, 0x54ca0a4b, 0xa01c4c73,
  0x556040e2, 0xa0a1f24d,
  0x55f5a4d2, 0xa1288376, 0x568a34a9, 0xa1affea3, 0x571deefa, 0xa2386284,
  0x57b0d256, 0xa2c1adc9,
  0x5842dd54, 0xa34bdf20, 0x58d40e8c, 0xa3d6f534, 0x59646498, 0xa462eeac,
  0x59f3de12, 0xa4efca31,
  0x5a82799a, 0xa57d8666, 0x5b1035cf, 0xa60c21ee, 0x5b9d1154, 0xa69b9b68,
  0x5c290acc, 0xa72bf174,
  0x5cb420e0, 0xa7bd22ac, 0x5d3e5237, 0xa84f2daa, 0x5dc79d7c, 0xa8e21106,
  0x5e50015d, 0xa975cb57,
  0x5ed77c8a, 0xaa0a5b2e, 0x5f5e0db3, 0xaa9fbf1e, 0x5fe3b38d, 0xab35f5b5,
  0x60686ccf, 0xabccfd83,
  0x60ec3830, 0xac64d510, 0x616f146c, 0xacfd7ae8, 0x61f1003f, 0xad96ed92,
  0x6271fa69, 0xae312b92,
  0x62f201ac, 0xaecc336c, 0x637114cc, 0xaf6803a2, 0x63ef3290, 0xb0049ab3,
  0x646c59bf, 0xb0a1f71d,
  0x64e88926, 0xb140175b, 0x6563bf92, 0xb1def9e9, 0x65ddfbd3, 0xb27e9d3c,
  0x66573cbb, 0xb31effcc,
  0x66cf8120, 0xb3c0200c, 0x6746c7d8, 0xb461fc70, 0x67bd0fbd, 0xb5049368,
  0x683257ab, 0xb5a7e362,
  0x68a69e81, 0xb64beacd, 0x6919e320, 0xb6f0a812, 0x698c246c, 0xb796199b,
  0x69fd614a, 0xb83c3dd1,
  0x6a6d98a4, 0xb8e31319, 0x6adcc964, 0xb98a97d8, 0x6b4af279, 0xba32ca71,
  0x6bb812d1, 0xbadba943,
  0x6c242960, 0xbb8532b0, 0x6c8f351c, 0xbc2f6513, 0x6cf934fc, 0xbcda3ecb,
  0x6d6227fa, 0xbd85be30,
  0x6dca0d14, 0xbe31e19b, 0x6e30e34a, 0xbedea765, 0x6e96a99d, 0xbf8c0de3,
  0x6efb5f12, 0xc03a1368,
  0x6f5f02b2, 0xc0e8b648, 0x6fc19385, 0xc197f4d4, 0x7023109a, 0xc247cd5a,
  0x708378ff, 0xc2f83e2a,
  0x70e2cbc6, 0xc3a94590, 0x71410805, 0xc45ae1d7, 0x719e2cd2, 0xc50d1149,
  0x71fa3949, 0xc5bfd22e,
  0x72552c85, 0xc67322ce, 0x72af05a7, 0xc727016d, 0x7307c3d0, 0xc7db6c50,
  0x735f6626, 0xc89061ba,
  0x73b5ebd1, 0xc945dfec, 0x740b53fb, 0xc9fbe527, 0x745f9dd1, 0xcab26fa9,
  0x74b2c884, 0xcb697db0,
  0x7504d345, 0xcc210d79, 0x7555bd4c, 0xccd91d3d, 0x75a585cf, 0xcd91ab39,
  0x75f42c0b, 0xce4ab5a2,
  0x7641af3d, 0xcf043ab3, 0x768e0ea6, 0xcfbe389f, 0x76d94989, 0xd078ad9e,
  0x77235f2d, 0xd13397e2,
  0x776c4edb, 0xd1eef59e, 0x77b417df, 0xd2aac504, 0x77fab989, 0xd3670446,
  0x78403329, 0xd423b191,
  0x78848414, 0xd4e0cb15, 0x78c7aba2, 0xd59e4eff, 0x7909a92d, 0xd65c3b7b,
  0x794a7c12, 0xd71a8eb5,
  0x798a23b1, 0xd7d946d8, 0x79c89f6e, 0xd898620c, 0x7a05eead, 0xd957de7a,
  0x7a4210d8, 0xda17ba4a,
  0x7a7d055b, 0xdad7f3a2, 0x7ab6cba4, 0xdb9888a8, 0x7aef6323, 0xdc597781,
  0x7b26cb4f, 0xdd1abe51,
  0x7b5d039e, 0xdddc5b3b, 0x7b920b89, 0xde9e4c60, 0x7bc5e290, 0xdf608fe4,
  0x7bf88830, 0xe02323e5,
  0x7c29fbee, 0xe0e60685, 0x7c5a3d50, 0xe1a935e2, 0x7c894bde, 0xe26cb01b,
  0x7cb72724, 0xe330734d,
  0x7ce3ceb2, 0xe3f47d96, 0x7d0f4218, 0xe4b8cd11, 0x7d3980ec, 0xe57d5fda,
  0x7d628ac6, 0xe642340d,
  0x7d8a5f40, 0xe70747c4, 0x7db0fdf8, 0xe7cc9917, 0x7dd6668f, 0xe8922622,
  0x7dfa98a8, 0xe957ecfb,
  0x7e1d93ea, 0xea1debbb, 0x7e3f57ff, 0xeae4207a, 0x7e5fe493, 0xebaa894f,
  0x7e7f3957, 0xec71244f,
  0x7e9d55fc, 0xed37ef91, 0x7eba3a39, 0xedfee92b, 0x7ed5e5c6, 0xeec60f31,
  0x7ef05860, 0xef8d5fb8,
  0x7f0991c4, 0xf054d8d5, 0x7f2191b4, 0xf11c789a, 0x7f3857f6, 0xf1e43d1c,
  0x7f4de451, 0xf2ac246e,
  0x7f62368f, 0xf3742ca2, 0x7f754e80, 0xf43c53cb, 0x7f872bf3, 0xf50497fb,
  0x7f97cebd, 0xf5ccf743,
  0x7fa736b4, 0xf6956fb7, 0x7fb563b3, 0xf75dff66, 0x7fc25596, 0xf826a462,
  0x7fce0c3e, 0xf8ef5cbb,
  0x7fd8878e, 0xf9b82684, 0x7fe1c76b, 0xfa80ffcb, 0x7fe9cbc0, 0xfb49e6a3,
  0x7ff09478, 0xfc12d91a,
  0x7ff62182, 0xfcdbd541, 0x7ffa72d1, 0xfda4d929, 0x7ffd885a, 0xfe6de2e0,
  0x7fff6216, 0xff36f078
};

/**   
*   
* @brief  Initialization function for the Q31 CFFT/CIFFT.  
* @param[in,out] *S             points to an instance of the Q31 CFFT/CIFFT structure.  
* @param[in]     fftLen         length of the FFT.  
* @param[in]     ifftFlag       flag that selects forward (ifftFlag=0) or inverse (ifftFlag=1) transform.  
* @param[in]     bitReverseFlag flag that enables (bitReverseFlag=1) or disables (bitReverseFlag=0) bit reversal of output.  
* @return        The function returns ARM_MATH_SUCCESS if initialization is successful or ARM_MATH_ARGUMENT_ERROR if <code>fftLen</code> is not a supported value.  
*   
* \par Description:  
* \par   
* The parameter <code>ifftFlag</code> controls whether a forward or inverse transform is computed.   
* Set(=1) ifftFlag for calculation of CIFFT otherwise  CFFT is calculated  
* \par   
* The parameter <code>bitReverseFlag</code> controls whether output is in normal order or bit reversed order.   
* Set(=1) bitReverseFlag for output to be in normal order otherwise output is in bit reversed order.   
* \par   
* The parameter <code>fftLen</code>	Specifies length of CFFT/CIFFT process. Supported FFT Lengths are 16, 64, 256, 1024.   
* \par   
* This Function also initializes Twiddle factor table pointer and Bit reversal table pointer.   
*/

arm_status arm_cfft_radix4_init_q31(
  arm_cfft_radix4_instance_q31 * S,
  uint16_t fftLen,
  uint8_t ifftFlag,
  uint8_t bitReverseFlag)
{
  /*  Initialise the default arm status */
  arm_status status = ARM_MATH_SUCCESS;
  /*  Initialise the FFT length */
  S->fftLen = fftLen;
  /*  Initialise the Twiddle coefficient pointer */
  S->pTwiddle = (q31_t *) twiddleCoefQ31;
  /*  Initialise the Flag for selection of CFFT or CIFFT */
  S->ifftFlag = ifftFlag;
  /*  Initialise the Flag for calculation Bit reversal or not */
  S->bitReverseFlag = bitReverseFlag;

  /*  Initializations of Instance structure depending on the FFT length */
  switch (S->fftLen)
  {
    /*  Initializations of structure parameters for 1024 point FFT */
  case 1024u:
    /*  Initialise the twiddle coef modifier value */
    S->twidCoefModifier = 1u;
    /*  Initialise the bit reversal table modifier */
    S->bitRevFactor = 1u;
    /*  Initialise the bit reversal table pointer */
    S->pBitRevTable = armBitRevTable;
    break;

  case 256u:
    /*  Initializations of structure parameters for 256 point FFT */
    S->twidCoefModifier = 4u;
    S->bitRevFactor = 4u;
    S->pBitRevTable = (uint16_t *) & armBitRevTable[3];
    break;

  case 64u:
    /*  Initializations of structure parameters for 64 point FFT */
    S->twidCoefModifier = 16u;
    S->bitRevFactor = 16u;
    S->pBitRevTable = &armBitRevTable[15];
    break;

  case 16u:
    /*  Initializations of structure parameters for 16 point FFT */
    S->twidCoefModifier = 64u;
    S->bitRevFactor = 64u;
    S->pBitRevTable = &armBitRevTable[63];
    break;

  default:
    /*  Reporting argument error if fftSize is not valid value */
    status = ARM_MATH_ARGUMENT_ERROR;
    break;
  }

  return (status);
}

/**   
 * @} end of CFFT_CIFFT group   
 */
