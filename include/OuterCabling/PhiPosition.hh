#ifndef PHIPOSITION_HH
#define PHIPOSITION_HH

#include "OuterCabling/outer_cabling_functions.hh"


/* This class contains several useful Phi identifiers.
   A phi Slice is used to name a phiSegment, a phiRegion, or a phiSector.
   * phiSegment : Phi slice delimited by 2 consecutive (centers of) modules in Phi.
   * phiRegion : Phi Slice contaning modules which can be connected to the same bundle.
   * phiSector : Phi slice of size outer_cabling_nonantWidth.

   There is the following classification, by the phi angle which is covered : phiSegment < phiRegion < phiSector.

   NB: These phi slices correspond to purely geometrical delimitations only.
   In exceptional cases, the modules can be artificially placed in a phi slice which does not correspond to its geometrical position : staggering.
   This is used to avoid that a bundle (or cable) connects to more modules than possible.
*/
class PhiPosition {
public:
  PhiPosition(const double phi, const int numPhiSegments, const bool isBarrel, const int layerDiskNumber, const std::string subDetectorName = "", const Category& bundleType = Category::UNDEFINED, const bool isTilted = false, const bool isPositiveCablingSide = true);

  // PHI SEGMENT
  const double phiSegmentWidth() const { return phiSegmentWidth_; }
  const double phiSegmentStart() const { return phiSegmentStart_; }
  const int phiSegmentRef() const { return phiSegmentRef_; }
  
  // PHI REGION
  const double phiRegionWidth() const { return phiRegionWidth_; }
  const double phiRegionStart() const { return phiRegionStart_; }
  const int phiRegionRef() const { return phiRegionRef_; }

  // PHI SECTOR
  const double phiSectorWidth() const { return phiSectorWidth_; }
  const double phiSectorStart() const { return phiSectorStart_; }
  const int phiSectorRef() const { return phiSectorRef_; }


  // STEREO PHI SEGMENT
  const double stereoPhiSegmentStart() const { return stereoPhiSegmentStart_; }
  const int stereoPhiSegmentRef() const { return stereoPhiSegmentRef_; }

  // STEREO PHI REGION
  const double stereoPhiRegionStart() const { return stereoPhiRegionStart_; }
  const int stereoPhiRegionRef() const { return stereoPhiRegionRef_; }

  // STEREO PHI SECTOR
  const double stereoPhiSectorStart() const { return stereoPhiSectorStart_; }
  const int stereoPhiSectorRef() const { return stereoPhiSectorRef_; }

private:
  const std::pair<int, double> computePhiRegionRefAndWidth (const int numRods,
							    const int phiSegmentRef, const double phiSegmentWidth,  
							    const int phiSectorRef, 
							    const std::string subDetectorName, const int layerDiskNumber, const bool isTilted,
							    const bool isPositiveCablingSide
							    ) const;

  // PHI SEGMENT
  double phiSegmentWidth_;
  double phiSegmentStart_;
  int phiSegmentRef_;

  // PHI REGION
  double phiRegionWidth_;
  double phiRegionStart_;
  int phiRegionRef_;

  // PHI SECTOR
  const double phiSectorWidth_ = outer_cabling_nonantWidth;
  double phiSectorStart_;
  int phiSectorRef_;


  // STEREO PHI SEGMENT
  double stereoPhiSegmentStart_;
  int stereoPhiSegmentRef_;

  // STEREO PHI REGION
  double stereoPhiRegionWidth_;
  double stereoPhiRegionStart_;
  int stereoPhiRegionRef_;

  // STEREO PHI SECTOR
  double stereoPhiSectorStart_;
  int stereoPhiSectorRef_;
};


#endif  // PHIPOSITION_HH
