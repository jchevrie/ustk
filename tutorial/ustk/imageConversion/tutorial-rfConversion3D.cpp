#include <visp3/ustk_core/usConfig.h>

#ifdef USTK_HAVE_FFTW

#include <visp3/core/vpTime.h>
#include <visp3/ustk_core/usRFToPreScan3DConverter.h>
#include <visp3/ustk_io/usImageIo.h>

int main()
{
  std::string filename;

  // Get the ustk-dataset package path or USTK_DATASET_PATH environment variable value
  std::string env_ipath = us::getDataSetPath();
  if (!env_ipath.empty())
    filename = env_ipath + "/rf/signal.rf";
  else {
    std::cout << "You should set USTK_DATASET_PATH environment var to access to ustk dataset" << std::endl;
    return 0;
  }

  usImageRF2D<short int> rfImage;
  usImageRF3D<short int> rfImage3D;
  rfImage3D.resize(192, 3840, 10);

  usImagePreScan3D<unsigned char> prescanImage;

  char buffer[400];
  for (int i = 0; i < 10; i++) {
    sprintf(buffer, filename.c_str(), i);
    usImageIo::read(rfImage, filename);
    rfImage3D.insertFrame(rfImage, i);
  }

  // settings used for rf file in ustk-dataset
  rfImage3D.setScanLinePitch(0.010625);
  rfImage3D.setTransducerRadius(0.0398);
  rfImage3D.setDepth(0.15);
  rfImage3D.setMotorType(usMotorSettings::TiltingMotor);
  rfImage3D.setMotorRadius(0.06);
  rfImage3D.setFramePitch(0.02);

  std::cout << "end reading" << std::endl;

  // scan-conversion
  usRFToPreScan3DConverter converter;

  double startTime = vpTime::measureTimeMs();

  std::cout << "converting..." << std::endl;

  converter.convert(rfImage3D, prescanImage);

  std::cout << prescanImage;

  double endConvertTime = vpTime::measureTimeMs();
  std::cout << "convert time (sec) = " << (endConvertTime - startTime) / 1000.0 << std::endl;

  std::cout << "writing pre-scan..." << std::endl;
  std::string outFileName = "preScan.mhd";
  usImageIo::write(prescanImage, outFileName);

  return 0;
}

#else
#include <iostream>
int main()
{
  std::cout << "You should install FFTW library to run this tutorial" << std::endl;
  return 0;
}

#endif
