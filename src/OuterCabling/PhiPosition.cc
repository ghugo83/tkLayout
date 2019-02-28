#include "OuterCabling/PhiPosition.hh"

#include <iostream>

PhiPosition::PhiPosition(const double phi, const int numPhiSegments, const bool isBarrel, const int layerDiskNumber, const std::string subDetectorName, const Category& bundleType, const bool isTilted, const bool isPositiveCablingSide) {

  // BARREL
  if (isBarrel) {
    double rodPhi = phi;
    int numRods = numPhiSegments;

    // PHI SEGMENT
    phiSegmentWidth_ = (2.*M_PI) / numRods;
    phiSegmentStart_ = computePhiSegmentStart(rodPhi, phiSegmentWidth_);
    phiSegmentRef_ = computePhiSegmentRef(rodPhi, phiSegmentStart_, phiSegmentWidth_);

    // PHI SECTOR
    phiSectorStart_ = 0.;
    phiSectorRef_ = computePhiSliceRef(rodPhi, phiSectorStart_, phiSectorWidth_);

    // PHI REGION  
    phiRegionStart_ = phiSegmentStart_;
    const std::pair<int, double> phiRegionRefAndWidth = computePhiRegionRefAndWidth(numRods,
										    phiSegmentRef_, phiSegmentWidth_,
										    phiSectorRef_, 
										    subDetectorName, layerDiskNumber, isTilted,
										    isPositiveCablingSide);
    phiRegionRef_ = phiRegionRefAndWidth.first;
    phiRegionWidth_ = phiRegionRefAndWidth.second;
							  


    // STEREO PHI SEGMENT
    double stereoRodPhi = femod(M_PI - rodPhi, 2.*M_PI);
    stereoPhiSegmentStart_ = computePhiSegmentStart(stereoRodPhi, phiSegmentWidth_);
    stereoPhiSegmentRef_ = computePhiSegmentRef(stereoRodPhi, stereoPhiSegmentStart_, phiSegmentWidth_);

    // STEREO PHI SECTOR
    stereoPhiSectorStart_ = phiSectorStart_;
    stereoPhiSectorRef_ = computePhiSliceRef(stereoRodPhi, stereoPhiSectorStart_, phiSectorWidth_);

    // STEREO PHI REGION
    stereoPhiRegionStart_ = stereoPhiSegmentStart_;
    const bool stereoIsPositiveCablingSide = !isPositiveCablingSide;
    const std::pair<int, double> stereoPhiRegionRefAndWidth = computePhiRegionRefAndWidth(numRods,
											  stereoPhiSegmentRef_, phiSegmentWidth_,  
											  stereoPhiSectorRef_, 
											  subDetectorName, layerDiskNumber, isTilted,
											  stereoIsPositiveCablingSide);
    stereoPhiRegionRef_ = stereoPhiRegionRefAndWidth.first;
    stereoPhiRegionWidth_ = stereoPhiRegionRefAndWidth.second;


  }

  // ENDCAPS
  else {
    double modPhi = phi;
    int numModulesInRing = numPhiSegments;

    // PHI SEGMENT
    phiSegmentWidth_ = (2.*M_PI) / numModulesInRing;
    phiSegmentStart_ = computePhiSegmentStart(modPhi, phiSegmentWidth_);
    phiSegmentRef_ = computePhiSegmentRef(modPhi, phiSegmentStart_, phiSegmentWidth_);

    // STEREO PHI SEGMENT
    double stereoModPhi =  femod(M_PI - modPhi, 2.*M_PI);
    stereoPhiSegmentStart_ = computePhiSegmentStart(stereoModPhi, phiSegmentWidth_);
    stereoPhiSegmentRef_ = computePhiSegmentRef(stereoModPhi, stereoPhiSegmentStart_, phiSegmentWidth_);
	
    // PHI REGION
    // Depending on the disk number and cabling type, different phiRegionWidth are assigned.
    // This is because for several cases, there can be too many modules per bundle, hence the phi width is defined smaller.
    phiRegionWidth_ = 0;	  
    phiRegionStart_ = 0.;
    // PS10GA, PS10GB
    if (bundleType == Category::PS10GA || bundleType == Category::PS10GB ) {
      phiRegionWidth_ = outer_cabling_nonantWidth;
    }
    // PS5G
    else if (bundleType == Category::PS5G ) {
      phiRegionWidth_ = outer_cabling_semiNonantWidth;
    }
    // 2S
    else if (bundleType == Category::SS ) {
      phiRegionWidth_ = outer_cabling_endcapStripStripPhiRegionWidth;
      // Use an offset to define these phiRegions 
      // (so that number of modules per phiRegion end up consistent with connection to 1 bundle only).
      if (subDetectorName == outer_cabling_tedd1) phiRegionStart_ = outer_cabling_tedd1StripStripPhiRegionStart;
      else phiRegionStart_ = outer_cabling_tedd2StripStripPhiRegionStart;
    }
    phiRegionRef_ = computePhiSliceRef(modPhi, phiRegionStart_, phiRegionWidth_);
    
    // PHI SECTOR
    phiSectorStart_ = 0.;
    phiSectorRef_ = computePhiSliceRef(modPhi, phiSectorStart_, phiSectorWidth_);
  }
}


const std::pair<int, double> PhiPosition::computePhiRegionRefAndWidth (const int numRods,
								       const int phiSegmentRef, const double phiSegmentWidth,  
								       const int phiSectorRef, 
								       const std::string subDetectorName, const int layerDiskNumber, const bool isTilted,
								       const bool isPositiveCablingSide
								       ) const {
  
  int phiRegionRef = phiSegmentRef;
  double phiRegionWidth = phiSegmentWidth;

  if (subDetectorName == outer_cabling_tbps && !isTilted) {

    //const int phiSegmentRefInPhiSector =  
    //computePhiSegmentRef(femod(rodPhi, 2. * M_PI) - phiSectorRef * phiSectorWidth, phiSegmentStart, phiSegmentWidth);
    const double numRodsPerPhiSector = (double)numRods / outer_cabling_numNonants;
    const double phiSegmentRefDouble = (double)phiSegmentRef;
    //const double rodPhiInPhiSector = femod(phiSegmentRefDouble, numRodsPerPhiSector);
    //const int phiSegmentRefInPhiSector = computePhiSliceRef(rodPhiInPhiSector, phiSegmentWidth);
    const int phiSegmentRefInPhiSector = round(phiSegmentRefDouble - phiSectorRef * numRodsPerPhiSector);

    if (layerDiskNumber == 1) {

      if (phiSegmentRefInPhiSector >= 2 || phiSegmentRefInPhiSector <= -1) {	
	std::cout << "CACAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA" << phiSectorRef << std::endl;
      }

      if (isPositiveCablingSide) {
	if (phiSegmentRefInPhiSector == 1) {
	  phiRegionRef = computePreviousPhiSliceRef(phiSegmentRef, numRods);
	}
	if (phiSegmentRefInPhiSector == 0 || phiSegmentRefInPhiSector == 1) {
	  phiRegionWidth = 2. * phiSegmentWidth;
	}
      }
      else {
	if (phiSegmentRefInPhiSector == 0) {
	  phiRegionRef = computeNextPhiSliceRef(phiSegmentRef, numRods);
	}
	if (phiSegmentRefInPhiSector == 0 || phiSegmentRefInPhiSector == 1) {
	  phiRegionWidth = 2. * phiSegmentWidth;
	}
      }
    }


    else if (layerDiskNumber == 2) {


      //if (phiSegmentRefInPhiSector >= 3 || phiSegmentRefInPhiSector <= -1) {	
	std::cout << "phiSegmentRefInPhiSector = " << phiSegmentRefInPhiSector << std::endl;
	std::cout << "layerDiskNumber = " << layerDiskNumber << std::endl;
	std::cout << "isPositiveCablingSide = " << isPositiveCablingSide << std::endl;
	std::cout << "phiSectorRef = " << phiSectorRef << std::endl;
	std::cout << "phiSegmentRef = " << phiSegmentRef << std::endl;
	//}

      
      if (isPositiveCablingSide) {
	if (phiSegmentRefInPhiSector == 0) {
	  phiRegionRef = computeNextPhiSliceRef(phiSegmentRef, numRods);
	}
	if (phiSegmentRefInPhiSector == 0 || phiSegmentRefInPhiSector == 1) {
	  phiRegionWidth = 2. * phiSegmentWidth;
	}
      }
      else {
	if (phiSegmentRefInPhiSector == 2) {
	  phiRegionRef = computePreviousPhiSliceRef(phiSegmentRef, numRods);
	}
	if (phiSegmentRefInPhiSector == 1 || phiSegmentRefInPhiSector == 2) {
	  phiRegionWidth = 2. * phiSegmentWidth;
	}
      }
    }


  }


  return std::make_pair(phiRegionRef, phiRegionWidth);
}
