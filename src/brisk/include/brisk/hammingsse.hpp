/*
    BRISK - Binary Robust Invariant Scalable Keypoints
    Reference implementation of
    [1] Stefan Leutenegger,Margarita Chli and Roland Siegwart, BRISK:
    	Binary Robust Invariant Scalable Keypoints, in Proceedings of
    	the IEEE International Conference on Computer Vision (ICCV2011).

    Copyright (C) 2011  The Autonomous Systems Lab, ETH Zurich,
    Stefan Leutenegger and Margarita Chli.

    This file is part of BRISK.

    BRISK is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    BRISK is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with BRISK.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef HAMMINGSSE_HPP_
#define HAMMINGSSE_HPP_

#include <emmintrin.h>
#include <tmmintrin.h>

namespace cv{

#ifdef __GNUC__
static const char __attribute__((aligned(16))) MASK_4bit[16] = {0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf};
static const uint8_t __attribute__((aligned(16))) POPCOUNT_4bit[16] = { 0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4};
static const __m128i shiftval = _mm_set_epi32 (0,0,0,4);
#endif
#ifdef _MSC_VER
__declspec(align(16)) static const char MASK_4bit[16] = {0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf};
__declspec(align(16)) static const uint8_t POPCOUNT_4bit[16] = { 0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4};
static const __m128i shiftval = _mm_set_epi32 (0,0,0,4);
#endif

__inline__ // - SSSE3 - better alorithm, minimized psadbw usage - adapted from http://wm.ite.pl/articles/sse-popcount.html
uint32_t HammingSse::ssse3_popcntofXORed(const __m128i* signature1, const __m128i* signature2, const int numberOf128BitWords) {

	uint32_t result = 0;

	register __m128i xmm0;
	register __m128i xmm1;
	register __m128i xmm2;
	register __m128i xmm3;
	register __m128i xmm4;
	register __m128i xmm5;
	register __m128i xmm6;
	register __m128i xmm7;

	//__asm__ volatile ("movdqa (%0), %%xmm7" : : "a" (POPCOUNT_4bit) : "xmm7");
	xmm7 = _mm_load_si128 ((__m128i *)POPCOUNT_4bit);
	//__asm__ volatile ("movdqa (%0), %%xmm6" : : "a" (MASK_4bit) : "xmm6");
	xmm6 = _mm_load_si128 ((__m128i *)MASK_4bit);
	//__asm__ volatile ("pxor    %%xmm5, %%xmm5" : : : "xmm5"); // xmm5 -- global accumulator
	xmm5 = _mm_setzero_si128();

	const size_t end=(size_t)(signature1+numberOf128BitWords);

	//__asm__ volatile ("movdqa %xmm5, %xmm4"); // xmm4 -- local accumulator
	xmm4 = xmm5;//_mm_load_si128(&xmm5);

	//for (n=0; n < numberOf128BitWords; n++) {
	do{
		//__asm__ volatile ("movdqa (%0), %%xmm0" : : "a" (signature1++) : "xmm0"); //slynen load data for XOR
		//		__asm__ volatile(
		//				"movdqa	  (%0), %%xmm0	\n"
		//"pxor      (%0), %%xmm0   \n" //slynen do XOR
		xmm0 = _mm_xor_si128 ( (__m128i)*signature1++, (__m128i)*signature2++); //slynen load data for XOR and do XOR
		//				"movdqu    %%xmm0, %%xmm1	\n"
		xmm1 = xmm0;//_mm_loadu_si128(&xmm0);
		//				"psrlw         $4, %%xmm1	\n"
		xmm1 = _mm_srl_epi16 (xmm1, shiftval);
		//				"pand      %%xmm6, %%xmm0	\n"	// xmm0 := lower nibbles
		xmm0 = _mm_and_si128 (xmm0, xmm6);
		//				"pand      %%xmm6, %%xmm1	\n"	// xmm1 := higher nibbles
		xmm1 = _mm_and_si128 (xmm1, xmm6);
		//				"movdqu    %%xmm7, %%xmm2	\n"
		xmm2 = xmm7;//_mm_loadu_si128(&xmm7);
		//				"movdqu    %%xmm7, %%xmm3	\n"	// get popcount
		xmm3 = xmm7;//_mm_loadu_si128(&xmm7);
		//				"pshufb    %%xmm0, %%xmm2	\n"	// for all nibbles
		xmm2 = _mm_shuffle_epi8(xmm2, xmm0);
		//				"pshufb    %%xmm1, %%xmm3	\n"	// using PSHUFB
		xmm3 = _mm_shuffle_epi8(xmm3, xmm1);
		//				"paddb     %%xmm2, %%xmm4	\n"	// update local
		xmm4 = _mm_add_epi8(xmm4, xmm2);
		//				"paddb     %%xmm3, %%xmm4	\n"	// accumulator
		xmm4 = _mm_add_epi8(xmm4, xmm3);
		//				:
		//				: "a" (buffer++)
		//				: "xmm0","xmm1","xmm2","xmm3","xmm4"
		//		);
	}while((size_t)signature1<end);
	// update global accumulator (two 32-bits counters)
	//	__asm__ volatile (
	//			/*"pxor	%xmm0, %xmm0		\n"*/
	//			"psadbw	%%xmm5, %%xmm4		\n"
	xmm4 = _mm_sad_epu8(xmm4, xmm5);
	//			"paddd	%%xmm4, %%xmm5		\n"
	xmm5 = _mm_add_epi32(xmm5, xmm4);
	//			:
	//			:
	//			: "xmm4","xmm5"
	//	);
	// finally add together 32-bits counters stored in global accumulator
//	__asm__ volatile (
//			"movhlps   %%xmm5, %%xmm0	\n"
	xmm0 = _mm_cvtps_epi32(_mm_movehl_ps(_mm_cvtepi32_ps(xmm0), _mm_cvtepi32_ps(xmm5))); //TODO fix with appropriate intrinsic
//			"paddd     %%xmm5, %%xmm0	\n"
	xmm0 = _mm_add_epi32(xmm0, xmm5);
//			"movd      %%xmm0, %%eax	\n"
	result = _mm_cvtsi128_si32 (xmm0);
//			: "=a" (result) : : "xmm5","xmm0"
//	);
	return result;
}

}
#endif /* HAMMINGSSE_HPP_ */
