#pragma once

// Functions that computes distances between blocks

// See legal notice in Copying.txt for more information

// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA, or visit
// http://www.gnu.org/copyleft/gpl.html .

#include <cstdint>


typedef unsigned int (*SADFunction)(const uint8_t *pSrc, intptr_t nSrcPitch,
                                    const uint8_t *pRef, intptr_t nRefPitch) noexcept;


SADFunction selectSADFunction(unsigned width, unsigned height, unsigned bits);

SADFunction selectSATDFunction(unsigned width, unsigned height, unsigned bits);


#if defined(MVTOOLS_X86)
SADFunction selectSADFunctionAVX2(unsigned width, unsigned height, unsigned bits);
#endif
