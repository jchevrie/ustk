/****************************************************************************
 *
 * This file is part of the ustk software.
 * Copyright (C) 2016 - 2017 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ustk with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * This software was developed at:
 * Inria Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 *
 * If you have questions regarding the use of this file, please contact
 * Inria at ustk@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Authors:
 * Marc Pouliquen
 *
 *****************************************************************************/

/**
 * @file usRFToPostScan3DConverter.h
 * @brief 3D converter from RF to post-scan.
 */

#ifndef __usRFToPostScan3DConverter_h_
#define __usRFToPostScan3DConverter_h_

#include <visp3/ustk_core/usConfig.h>

#if defined(USTK_HAVE_FFTW)

// visp/ustk includes
#include <visp3/ustk_core/usRFToPreScan3DConverter.h>
#include <visp3/ustk_core/usPreScanToPostScan3DConverter.h>

/**
 * @class usRFToPostScan3DConverter
 * @brief 3D conversion from RF signal to post-scan image
 * @ingroup module_ustk_core
 *
 * This class allows to convert 3D RF ultrasound images to post-scan.
 *
 */
class VISP_EXPORT usRFToPostScan3DConverter
{
 public:

  usRFToPostScan3DConverter(int decimationFactor=10);

  ~usRFToPostScan3DConverter();

  void convert(const usImageRF3D<short int> &rfImage, usImagePostScan3D<unsigned char> &postScanImage);

private:
  usRFToPreScan3DConverter m_RFConverter;
  usPreScanToPostScan3DConverter m_scanConverter;

  bool m_scanConverterIsInit;
  bool m_inputImageChanged;

  usImagePreScan3D<unsigned char> m_intermediateImage;
};

#endif // USTK_HAVE_FFTW
#endif // __usRFToPostScan3DConverter_h_
