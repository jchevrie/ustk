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
* Pierre Chatelain
* Marc Pouliquen
*
*****************************************************************************/

/**
* @file usImage3D.h
* @brief 3D image handling.
*
* This class is used to represent 3D data with physical information such as element spacing.
*/

#ifndef US_IMAGE_3D_H
#define US_IMAGE_3D_H

#include <cstring>
#include <iostream>
#include <vector>
#include <algorithm>

#include <visp3/core/vpDebug.h>
#include <visp3/core/vpException.h>

/**
* @class usImage3D
* @brief Representation of a physical image volume.
* @ingroup module_ustk_core
*
* This class is used to represent 3D data with physical information.
*/
template <class Type>
class usImage3D
{
public:
  /**
  * Constructor.
  */
  usImage3D();

  /**
  * Constructor. Set the dimensions and element spacing of the volume.
  * @param dimX Volume width.
  * @param dimY Volume height.
  * @param dimZ Volume size in the third dimension (orthogonal to ultrasound 2D frames).
  */
  usImage3D(unsigned int dimX, unsigned int dimY, unsigned int dimZ);

  /**
  * Constructor. Set the dimensions and element spacing of the volume.
  * @param dimX Volume width.
  * @param dimY Volume height.
  * @param dimZ Volume size in the third dimension (orthogonal to ultrasound 2D frames).
  * @param spacingX distancee (in meters) between two voxels on X-axis
  * @param spacingY distancee (in meters) between two voxels on Y-axis
  * @param spacingZ distancee (in meters) between two voxels on Z-axis
  */
  usImage3D(unsigned int dimX, unsigned int dimY, unsigned int dimZ, double spacingX, double spacingY, double spacingZ);

  /**
  * Copy constructor. By default performs a deep copy.
  * @param image other 3D-image to copy
  * @param copy boolean to select if deep copy is performed or not (deep copy by default)
  */
  usImage3D(const usImage3D<Type> &image, const bool copy = true);

  /**
  * Destructor.
  */
  virtual ~usImage3D();

  /** @name Inherited functionalities from usImage3D */
  //@{

  /**
  * Get the pointer to the const data container.
  * @return The pointer to the const data container.
  */
  Type* getConstData() const { return bitmap; }

  /**
  * Get the pointer to the data container.
  * @return The pointer to the data container.
  */
  Type* getData() { return bitmap; }

  /**
  * Get the volume width.
  * @return The volume width, in number of voxels.
  */
  unsigned int getDimX() const { return m_dimX; }

  /**
  * Get the volume height.
  * @return The volume height, in number of voxels.
  */
  unsigned int getDimY() const { return m_dimY; }

  /**
  * Get the volume size along the z-axis.
  * @return The z-axis size in voxels, in number of voxels.
  */
  unsigned int getDimZ() const { return m_dimZ; }

  /**
  * Get the element spacing along the x-axis.
  * @return The element spacing along the x-axis, in meters.
  */
  double getElementSpacingX() const { return m_elementSpacingX; }

  /**
  * Get the element spacing along the y-axis.
  * @return The element spacing along the y-axis, in meters.
  */
  double getElementSpacingY() const { return m_elementSpacingY; }

  /**
  * Get the element spacing along the z-axis.
  * @return The element spacing along the z-axis, in meters.
  */
  double getElementSpacingZ() const { return m_elementSpacingZ; }

  /**
  * Get the volume size.
  * @return The number of voxels in the volume.
  */
  unsigned int getSize() const { return m_size; }

  /**
  * Initiation of the image.
  * @param dimX Volume width (number of voxels).
  * @param dimY Volume height (number of voxels).
  * @param dimZ Volume size (number of voxels) in the third dimension (orthogonal to ultrasound 2D frames).
  */
  void init(unsigned int dimX, unsigned int dimY, unsigned int dimZ);

  /**
  * Initiation of the image.
  * @param dimX Volume width.
  * @param dimY Volume height.
  * @param dimZ Volume size in the third dimension (orthogonal to ultrasound 2D frames).
  * @param spacingX Element spacing in x axis, in meters.
  * @param spacingY Element spacing in x axis, in meters.
  * @param spacingZ Element spacing in x axis, in meters.
  */
  void init(unsigned int dimX, unsigned int dimY, unsigned int dimZ, double spacingX, double spacingY, double spacingZ);

  /**
  * Initialize the data container with the specified value.
  * @param value The data
  * @param numberOfVoxels number of voxels in the volume
  */
  void initData(Type value,int numberOfVoxels);

  /**
  * Assignment operator.
  * @param other other 3D-image to copy
  */
  usImage3D<Type> &operator=(const usImage3D<Type> &other);

  /**
  * Comparison operator.
  * @param other The 3d image to compare. Comparing image parameters (size and spacing), and all volume voxel by voxel.
  */
  bool operator==(const usImage3D<Type> &other);

  /**
  * Access operator.
  * @param index Index of the data to acess.
  */
  inline Type operator()(unsigned int index) const
  {
    return bitmap[index];
  }

  //! operator[] allows operation like I[i] = x.
  Type& operator[]( const unsigned int i)   { if(i<m_size) {return bitmap[i];} throw(vpException::badValue);}
  Type& operator[]( const int i) { if(i<m_size) {return bitmap[i];} throw(vpException::badValue);}

  //! operator[] allows operation like x = I[i]
  const  Type& operator[](unsigned int i) const { if(i<m_size) {return bitmap[i];} throw(vpException::badValue);}
  const  Type& operator[](int i) const { if(i<m_size) {return bitmap[i];} throw(vpException::badValue);}

  /**
  * Modification operator.
  * @param index Index of the data to modify.
  * @param value New value to set.
  */
  inline void operator()(unsigned int index, Type value)
  {
    bitmap[index] = value;
  }

  /**
  * Access operator.
  * @param indexX Index on x-axis to acess
  * @param indexY Index on y-axis to acess
  * @param indexZ Index on z-axis to acess
  */
  inline Type operator()(unsigned int indexX, unsigned int indexY, unsigned int indexZ) const
  {
    return bitmap[(m_dimX * m_dimY) * indexZ + m_dimX*indexY + indexX];
  }

  /**
  * Modification operator.
  * @param indexX Index on x-axis to modify
  * @param indexY Index on y-axis to modify
  * @param indexZ Index on z-axis to modify
  * @param value Value to insert at the desired index
  */
  inline void operator()(unsigned int indexX, unsigned int indexY, unsigned int indexZ, Type value)
  {
    bitmap[(m_dimX * m_dimY)*indexZ + m_dimX*indexY + indexX] = value;
  }

  /**
  * Resize the image if needed (if new dimensions differ from old ones).
  * @param dimX The volume size along x axis.
  * @param dimY The volume size along y axis.
  * @param dimZ The volume size along z axis.
  */
  void resize(unsigned int dimX,unsigned int dimY,unsigned int dimZ);

  /**
  * Set the data container.
  * @param data The data container.
  * @param numberOfVoxels The number of voxels in the image.
  */
  void setData(Type* data, int numberOfVoxels);

  /**
  * Set the element spacing along the x-axis.
  * @param elementSpacingX The element spacing along the x-axis, in meters.
  */
  void setElementSpacingX(double elementSpacingX) { m_elementSpacingX = elementSpacingX; }

  /**
  * Set the element spacing along the y-axis.
  * @param elementSpacingY The element spacing along the y-axis, in meters.
  */
  void setElementSpacingY(double elementSpacingY) { m_elementSpacingY = elementSpacingY; }

  /**
  * Set the element spacing along the z-axis.
  * @param elementSpacingZ The element spacing along the z-axis, in meters.
  */
  void setElementSpacingZ(double elementSpacingZ) { m_elementSpacingZ = elementSpacingZ; }

  //@}

protected:

private:
  unsigned int m_dimX; /**< Volume width in pixels (number of pixels on the x-axis)*/
  unsigned int m_dimY; /**< Volume height in pixels (number of pixels on the y-axis)*/
  unsigned int m_dimZ; /**< Volume size in 3d dimension (number of pixels on the z-axis)*/
  double m_elementSpacingX; /**< Element spacing along the x-axis, in meters */
  double m_elementSpacingY; /**< Element spacing along the y-axis, in meters */
  double m_elementSpacingZ; /**< Element spacing along the z-axis, in meters */
  unsigned int m_size; /**< Volume size : number of voxels in the whole volume*/

  Type* bitmap; /**< Data container */
  Type **planesIndex ;
};

/****************************************************************************
* Template implementations.
****************************************************************************/
/*!
  \brief Image initialization

  Allocate memory for an [dimX x dimY x dimZ] image.
  \param dimX : Width of the 2D planes contained in the volume.
  \param dimY : Height of the 2D planes contained in the volume.
  \param dimZ : Volume dimension in the 3rd dimension.

  Element of the bitmap are not initialized

  If the image has been already initialized, memory allocation is done
  only if the new image size is different, else we re-use the same
  memory space.

  \exception vpException::memoryAllocationError

*/
template<class Type>
inline void usImage3D<Type>::init(unsigned int dimX, unsigned int dimY, unsigned int dimZ)
{
  if ((dimX != this->m_dimX) || (dimY != this->m_dimY)) {
    if (planesIndex != NULL) {
      vpDEBUG_TRACE(10,"Destruction index[]");
      delete [] planesIndex;
      planesIndex = NULL;
    }
  }

  if ((dimX != this->m_dimX) || (dimY != this->m_dimY) || (dimZ != this->m_dimZ))
  {
    if (bitmap != NULL) {
      vpDEBUG_TRACE(10,"Destruction bitmap[]") ;
      delete [] bitmap;
      bitmap = NULL;
    }
  }

  this->m_dimX = dimX;
  this->m_dimY = dimY;
  this->m_dimZ = dimZ;

  m_size=m_dimX*m_dimY*m_dimZ;

  if (bitmap == NULL)  bitmap = new  Type[m_size] ;

  //  vpERROR_TRACE("Allocate bitmap %p",bitmap) ;
  if (bitmap == NULL)
  {
    vpERROR_TRACE("cannot allocate bitmap ") ;
    throw(vpException(vpException::memoryAllocationError,
                      "cannot allocate bitmap ")) ;
  }

  if (planesIndex == NULL)  planesIndex = new  Type*[m_dimX];
  //  vpERROR_TRACE("Allocate row %p",row) ;
  if (planesIndex == NULL)
  {
    vpERROR_TRACE("cannot allocate index ") ;
    throw(vpException(vpException::memoryAllocationError,
                      "cannot allocate index ")) ;
  }

  //filling planesIndex
  unsigned int i ;
  for ( i =0  ; i < m_dimZ ; i++)
    planesIndex[i] = bitmap + i*m_dimX*m_dimY ;
}

template<class Type>
void
usImage3D<Type>::init(unsigned int dimX, unsigned int dimY, unsigned int dimZ, double spacingX, double spacingY, double spacingZ)
{
  init(dimX, dimY, dimZ);
  m_elementSpacingX = spacingX;
  m_elementSpacingY = spacingY;
  m_elementSpacingZ = spacingZ;
}

template<class Type>
usImage3D<Type>::usImage3D() : m_dimX(0), m_dimY(0), m_dimZ(0),
m_elementSpacingX(1.0f), m_elementSpacingY(1.0f), m_elementSpacingZ(1.0f),
  m_size(0),bitmap(NULL), planesIndex(NULL)
{

}

template<class Type>
usImage3D<Type>::usImage3D(unsigned int dimX, unsigned int dimY, unsigned int dimZ)
: m_dimX(dimX), m_dimY(dimY), m_dimZ(dimZ), m_size(dimX * dimY * dimZ),
  m_elementSpacingX(1.0f), m_elementSpacingY(1.0f), m_elementSpacingZ(1.0f),
  bitmap(NULL), planesIndex(NULL)
{
  init(dimX, dimY, dimZ);
}

template<class Type>
usImage3D<Type>::usImage3D(unsigned int dimX, unsigned int dimY, unsigned int dimZ,
                           double elementSpacingX, double elementSpacingY, double elementSpacingZ)
                           : m_dimX(dimX), m_dimY(dimY), m_dimZ(dimZ), m_size(dimX * dimY * dimZ),
                             m_elementSpacingX(elementSpacingX), m_elementSpacingY(elementSpacingY),
                             m_elementSpacingZ(elementSpacingZ), bitmap(NULL), planesIndex(NULL)
{
  init(dimX, dimY, dimZ);
}

template<class Type>
usImage3D<Type>::usImage3D(const usImage3D<Type> &volume, const bool copy)
{
  init(volume.getDimX(),volume.getDimY(),volume.getDimZ());

  m_size = m_dimX * m_dimY * m_dimZ;
  m_elementSpacingX = volume.getElementSpacingX();
  m_elementSpacingY = volume.getElementSpacingY();
  m_elementSpacingZ = volume.getElementSpacingZ();

  //deep copy
  if(copy)
    memcpy(bitmap, volume.bitmap, m_size * sizeof(Type));
}

template<class Type>
usImage3D<Type>::~usImage3D()
{
  if (planesIndex)
  {
    delete[] planesIndex;
    planesIndex = NULL;
  }
  if (bitmap)
  {
    delete[] bitmap;
    bitmap = NULL;
  }
}

template<class Type>
usImage3D<Type> &usImage3D<Type>::operator=(const usImage3D<Type> &other)
{
  if (m_dimX != other.m_dimX
      || m_dimY != other.m_dimY
      || m_dimZ != other.m_dimZ)
  {
    m_dimX = other.m_dimX;
    m_dimY = other.m_dimY;
    m_dimZ = other.m_dimZ;
    m_size = m_dimX * m_dimY * m_dimZ;
    if (bitmap) delete[]bitmap;
    bitmap = new Type[m_size];
  }

  m_elementSpacingX = other.m_elementSpacingX;
  m_elementSpacingY = other.m_elementSpacingY;
  m_elementSpacingZ = other.m_elementSpacingZ;
  memcpy(bitmap, other.bitmap, m_size * sizeof(Type));
  return *this;
}

template<class Type>
bool usImage3D<Type>::operator==(const usImage3D<Type> &other)
{
  bool settingsOk = this->getDimX() == other.getDimX() &&
      this->getDimY() == other.getDimY() &&
      this->getDimZ() == other.getDimZ() &&
      this->getElementSpacingX() == other.getElementSpacingX() &&
      this->getElementSpacingY() == other.getElementSpacingY() &&
      this->getElementSpacingZ() == other.getElementSpacingZ();

  if(settingsOk) {
    for (unsigned int i=0 ; i < m_size ; i++) {
      if (bitmap[i] != other[i])
        return false;
    }
  }
  return true;
}

template<class Type> std::ostream& operator<<(std::ostream& out, const usImage3D<Type> &other)
{
  return out << "number of A-samples in a scanline : " << other.getDimX() << std::endl
             << "number of scanlines in a frame : " << other.getDimY() << std::endl
             << "number of frames : " << other.getDimZ() << std::endl;
}

template<class Type>
void usImage3D<Type>::setData(Type* data, int numberOfVoxels)
{
  try {
    m_size =numberOfVoxels;
    memcpy(bitmap, data, m_size * sizeof(Type));
  }
  catch (std::exception e)
  {
    std::cout << e.what() << std::endl;
    std::cout << "Bad allocation using std::fill_n() method." << std::endl;
  }
}

template<class Type>
void usImage3D<Type>::initData(Type value, int numberOfVoxels)
{
  try {
    m_size =numberOfVoxels;
    std::fill_n(bitmap, m_size, value);
  }
  catch (std::exception e)
  {
    std::cout << e.what() << std::endl;
    std::cout << "Bad allocation using std::fill_n() method." << std::endl;
  }
}

template<class Type>
void usImage3D<Type>::resize(unsigned int dimx,unsigned int dimy,unsigned int dimz)
{
  init(dimx,dimy,dimz);
}

#endif //US_IMAGE_3D_H
