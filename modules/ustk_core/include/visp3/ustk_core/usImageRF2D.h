/****************************************************************************
 *
 * This file is part of the UsTk software.
 * Copyright (C) 2014 by Inria. All rights reserved.
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License ("GPL") as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 * See the file COPYING at the root directory of this source
 * distribution for additional information about the GNU GPL.
 * 
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact the
 * authors at Alexandre.Krupa@inria.fr
 * 
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 *
 * Authors:
 * Marc Pouliquen
 *
 *****************************************************************************/

/**
 * @file usImageRF2D.h
 * @brief 2D RF ultrasound image.
 */

#ifndef US_IMAGE_RF_2D_H
#define US_IMAGE_RF_2D_H

#include <cstring>

#include <visp3/core/vpImage.h>

#include <visp3/ustk_core/usImageSettings.h>

/**
 * @class usImageRF2D
 * @brief 2D Rf ultrasound image.
 *
 * This class represents a 2D ultrasound RF frame.
 */
template<class T>
class usImageRF2D : public vpImage<T>, public usImageSettings {
 public:
  
  usImageRF2D();

  usImageRF2D(unsigned int AN, unsigned int LN);

  usImageRF2D(unsigned int AN, unsigned int LN, double probeRadius, double scanLinePitch, bool isConvex);

  usImageRF2D(const usImageRF2D &other);

  ~usImageRF2D();

  unsigned int getAN() const;

  unsigned int  getLN() const;

  double getAxialResolution() const;

  void setAxialResolution(double axialResolution);

private:
  double m_axialResolution;
};


/**
* unsigned char
* Constructor.
*/
template<class T>
usImageRF2D<T>::usImageRF2D() : vpImage<T>(), usImageSettings()
{

}

/**
* Initializing constructor for image dimentions.
* @param AN number of A-samples in a line.
* @param LN number of lines.
*/
template<class T>
usImageRF2D<T>::usImageRF2D(unsigned int AN, unsigned int LN)
  : vpImage<T>(AN, LN), usImageSettings()
{

}

/**
* Initializing constructor.
* @param AN number of A-samples in a line.
* @param LN number of lines.
* @param probeRadius radius of the ultrasound probe used to acquire the RF image.
* @param scanLinePitch Angle(rad) / Distance(m) between 2 lines of the ultrasound probe used to acquire the RF image. Angle if isConvex is true, distance if it's false.
* @param isConvex Boolean to specify if the probe used was convex(true) or linear(false).
*/
template<class T>
usImageRF2D<T>::usImageRF2D(unsigned int AN, unsigned int LN, double probeRadius, double scanLinePitch, bool isConvex)
  : vpImage<T>(AN, LN), usImageSettings(probeRadius, scanLinePitch, isConvex)
{

}

/**
* Copy constructor.
* @param other usImageRF2D to copy
*/
template<class T>
usImageRF2D<T>::usImageRF2D(const usImageRF2D& other)
  : vpImage<T>(other), usImageSettings(other)
{

}

/**
* Destructor.
*/
template<class T>
usImageRF2D<T>::~usImageRF2D()
{

}

/**
* Get the number of A-samples in a line.
* @return AN number of A-samples in a line.
*/
template<class T>
unsigned int usImageRF2D<T>::getAN() const { return vpImage<T>::getHeight(); }

/**
* Get the number of lines.
* @return LN number of lines.
*/
template<class T>
unsigned int usImageRF2D<T>::getLN() const { return vpImage<T>::getWidth(); }

/**
* Getter for the axial resolution
* @return The axial resolution : distance(in meters) between 2 successive pixels acquired along a scanLine.
*/
template<class T>
double usImageRF2D<T>::getAxialResolution() const { return m_axialResolution; }

/**
* Setter for the axial resolution
* @param axialResolution The axial resolution : distance(in meters) between 2 successive pixels acquired along a scanLine.
*/
template<class T>
void usImageRF2D<T>::setAxialResolution(double axialResolution)
{
  m_axialResolution = axialResolution;
}
#endif // US_IMAGE_RF_2D_H
